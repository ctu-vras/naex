#pragma once

#include "naex/grid/grid.h"
#include "naex/iterators.h"
#include <boost/graph/graph_traits.hpp>
#include <boost/graph/properties.hpp>
#include <boost/property_map/property_map.hpp>
#include <limits>
#include <vector>

namespace naex {
namespace grid {

typedef CellId VertexId;
typedef CellId EdgeId;

typedef ValueIterator<VertexId> VertexIter;
typedef ValueIterator<EdgeId> EdgeIter;

/**
 * Sentinel for "no vertex".  Distinct from naex::INVALID_VERTEX, which is a
 * signed Index and compares badly against the unsigned VertexId.
 */
inline constexpr VertexId INVALID_VERTEX_ID =
    std::numeric_limits<VertexId>::max();

/**
 * Boost.Graph adapter over a Grid, with the neighbourhood fixed at compile
 * time.
 *
 * @tparam N 4 or 8.  A compile-time N turns the per-edge `e / N` and `e % N`
 * of the Dijkstra inner loop into a shift and a mask, and drops the
 * 8-neighbourhood distance multiplication entirely for N == 4.  The runtime
 * `neighborhood` parameter selects between the two instantiations in
 * ShortestPaths, so nothing above this class has to know.
 *
 * Edges are implicit: edge e belongs to vertex `e / N` and is its `e % N`-th
 * neighbour direction.  Targets come from Grid's flat neighbour table, so the
 * inner loop does no hash lookup at all; the per-vertex total cost (INF for a
 * cell that is out of bounds) is cached here, so it does not recompute
 * Costs::total() either.  The cache is built once in the constructor and is
 * therefore only valid while the grid's costs do not change -- construct the
 * graph after the ad-hoc layer has been written, never before.
 *
 * https://www.boost.org/doc/libs/1_75_0/libs/graph/doc/adjacency_list.html
 */
template <uint8_t N> class GraphN {
  static_assert(N == 4 || N == 8, "neighborhood must be 4 or 8");

public:
  static constexpr Cost INF = std::numeric_limits<Cost>::infinity();
  /**
   * Cost of crossing a cell that costs nothing at all.
   *
   * An edge costs (kBaseCellCost + mean of the two cell costs) * its length,
   * so a free cell still costs its length, the search prefers short routes
   * among equally cheap ones, and the plain-distance A* heuristic stays
   * admissible (no edge is ever cheaper than its length).
   */
  static constexpr Cost kBaseCellCost = 1;

  explicit GraphN(const Grid &grid, const Costs &max_costs = Costs())
      : grid_(grid), max_costs_(max_costs), cell_size_(grid.cell_size()) {
    const size_t n = grid_.size();
    total_.resize(n);
    for (size_t v = 0; v < n; ++v) {
      const Costs &c = grid_.costs(static_cast<CellId>(v));
      // An out-of-bounds cell is stored as INF rather than in a second flag
      // array: 1 + (INF + x) / 2, scaled, is still exactly INF, which is what
      // cost() returned for it before, and the inner loop keeps one branch.
      total_[v] = grid::costs_in_bounds(c, max_costs_)
                      ? c.total()
                      : std::numeric_limits<Cost>::infinity();
    }
  }
  /// Compatibility overload; @p neighborhood must be N.
  GraphN(const Grid &grid, const uint8_t neighborhood,
         const Costs &max_costs = Costs())
      : GraphN(grid, max_costs) {
    assert(neighborhood == N);
    (void)neighborhood;
  }

  inline VertexId num_vertices() const { return grid_.size(); }
  inline std::pair<VertexIter, VertexIter> vertices() const {
    return {VertexIter(0), VertexIter(num_vertices())};
  }
  inline std::pair<EdgeIter, EdgeIter> out_edges(const VertexId &u) const {
    return {EdgeIter(N * u), EdgeIter(N * (u + 1))};
  }
  inline EdgeId out_degree(const VertexId &) const { return N; }
  inline VertexId source(const EdgeId &e) const { return e / N; }
  inline VertexId target_index(const EdgeId &e) const { return e % N; }

  /**
   * Index into Grid's 8-slot neighbour row for direction @p i of this
   * neighbourhood: neighbor8(c, 2 * i) == neighbor4(c, i).
   */
  static inline int dir_index(const VertexId i) {
    return static_cast<int>(N == 8 ? i : 2 * i);
  }

  /**
   * Target of edge @p e, or its source when that neighbour does not exist.
   *
   * The source fallback is unchanged from the hash-based version; the edge is
   * inert either way because cost() returns INF for it.
   */
  inline VertexId target(const EdgeId &e) const {
    const VertexId s = source(e);
    const CellId t = grid_.neighbor_id(s, dir_index(target_index(e)));
    return t == INVALID_CELL_ID ? s : t;
  }

  /// True if every bounded layer of @p costs is within max_costs.
  bool costs_in_bounds(const Costs &costs) const {
    return grid::costs_in_bounds(costs, max_costs_);
  }

  /**
   * Cost of edge @p e, or INF when the edge does not exist or either endpoint
   * is out of bounds.
   *
   * A missing neighbour is INF rather than the old finite self-edge: Dijkstra
   * can relax neither (a self-edge never improves the source's own distance),
   * so path costs are bit-for-bit what they were, and the table lookup is all
   * the work an absent edge costs now.
   */
  inline Cost cost(const EdgeId &e) const {
    const VertexId u = source(e);
    const VertexId i = target_index(e);
    const CellId v = grid_.neighbor_id(u, dir_index(i));
    if (v == INVALID_CELL_ID) {
      return INF;
    }
    // Expression and multiplication order kept byte for byte; the exact-cost
    // planning tests pin the accumulated float.
    Cost cost = kBaseCellCost + (total_[u] + total_[v]) / 2;
    cost *= cell_size_;
    if constexpr (N == 8) {
      cost *= kDist8[i];
    }
    return cost;
  }

protected:
  const Grid &grid_;
  const Costs max_costs_;
  const float cell_size_;
  /// Per-vertex Costs::total(), or INF when the cell is out of bounds.
  std::vector<Cost> total_;
};

/**
 * Number of existing neighbours of @p v in the @p neighborhood-connected grid
 * that satisfy @p accept(CellId).
 *
 * Table-based, like GraphN::target(): this replaces the out_edge walk over a
 * boost::filtered_graph the frontier detection used to do (one hash lookup per
 * examined edge).  A missing neighbour is INVALID_CELL_ID here, which is the
 * same edge the old code recognised by target(e) == source(e) and skipped, so
 * the degree is unchanged.
 */
template <typename Accept>
inline int neighbor_degree(const Grid &grid, CellId v, uint8_t neighborhood,
                           Accept accept) {
  const int count = (neighborhood == 4) ? 4 : 8;
  const int step = (neighborhood == 4) ? 2 : 1;
  int degree = 0;
  for (int k = 0; k < count; ++k) {
    const CellId t = grid.neighbor_id(v, k * step);
    if (t != INVALID_CELL_ID && accept(t)) {
      ++degree;
    }
  }
  return degree;
}

/// The 8-neighbourhood graph; the default everywhere.
typedef GraphN<8> Graph;

template <uint8_t N> class EdgeCostsN {
public:
  EdgeCostsN(const GraphN<N> &graph) : graph_(graph) {}
  inline Cost operator[](const EdgeId &e) const { return graph_.cost(e); }

protected:
  const GraphN<N> &graph_;
};

// Boost.Graph free functions found via ADL must live in naex::grid.
template <uint8_t N>
inline std::pair<VertexIter, VertexIter> vertices(const GraphN<N> &g) {
  return g.vertices();
}

template <uint8_t N> inline VertexId source(EdgeId e, const GraphN<N> &g) {
  return g.source(e);
}

template <uint8_t N> inline VertexId target(EdgeId e, const GraphN<N> &g) {
  return g.target(e);
}

template <uint8_t N>
inline std::pair<EdgeIter, EdgeIter> out_edges(VertexId u, const GraphN<N> &g) {
  return g.out_edges(u);
}

// num_vertices() and out_degree() are what boost::filtered_graph and
// breadth_first_search() need on top of the DijkstraGraph concept; they were
// commented out before A* was added.
template <uint8_t N> inline VertexId num_vertices(const GraphN<N> &g) {
  return g.num_vertices();
}

template <uint8_t N> inline EdgeId out_degree(VertexId u, const GraphN<N> &g) {
  return g.out_degree(u);
}

template <uint8_t N>
inline Cost get(const EdgeCostsN<N> &map, const EdgeId &key) {
  return map[key];
}

} // namespace grid
} // namespace naex

namespace boost {

template <uint8_t N> struct graph_traits<naex::grid::GraphN<N>> {
  typedef naex::grid::VertexId vertex_descriptor;
  typedef naex::grid::VertexId vertices_size_type;
  typedef naex::grid::EdgeId edge_descriptor;
  typedef naex::grid::EdgeId edges_size_type;

  typedef directed_tag directed_category;
  typedef disallow_parallel_edge_tag edge_parallel_category;

  // incidence_graph_tag, not the bidirectional_traversal_tag this used to
  // carry: A* (and boost::filtered_graph under it) requires IncidenceGraph,
  // and Dijkstra is happy with it too.
  typedef incidence_graph_tag traversal_category;
  typedef naex::grid::VertexIter vertex_iterator;
  typedef naex::grid::EdgeIter out_edge_iterator;
  // filtered_graph typedefs these unconditionally, so they have to exist even
  // though the graph is not bidirectional and has no edge list.
  typedef naex::grid::EdgeIter in_edge_iterator;
  typedef naex::grid::EdgeIter edge_iterator;

  typedef naex::grid::EdgeId degree_size_type;
};

template <uint8_t N> class property_traits<naex::grid::EdgeCostsN<N>> {
public:
  typedef naex::grid::EdgeId key_type;
  typedef naex::grid::Cost value_type;
  typedef naex::grid::Cost reference;
  typedef readable_property_map_tag category;
};

} // namespace boost
