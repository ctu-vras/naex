#pragma once

#include "naex/grid/grid.h"
#include "naex/iterators.h"
#include <boost/graph/graph_traits.hpp>
#include <limits>
#include <boost/property_map/property_map.hpp>

namespace naex {
namespace grid {

typedef CellId VertexId;
typedef CellId EdgeId;

typedef ValueIterator<VertexId> VertexIter;
typedef ValueIterator<EdgeId> EdgeIter;

/// Sentinel for "no vertex".  Distinct from naex::INVALID_VERTEX, which is a
/// signed Index and compares badly against the unsigned VertexId.
inline constexpr VertexId INVALID_VERTEX_ID =
    std::numeric_limits<VertexId>::max();

// neighbor4(), neighbor8(), distance8() and kDist8 moved to grid.h with P2:
// Grid itself has to resolve neighbours now that it keeps the flat neighbour
// table.  They are still naex::grid::neighbor8 etc. for every caller.

/**
 * Boost.Graph adapter over a Grid, with the neighbourhood fixed at compile
 * time (P2).
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
 * therefore only valid while the grid's costs do not change — construct the
 * graph after the ad-hoc layer has been written, never before.
 *
 * https://www.boost.org/doc/libs/1_75_0/libs/graph/doc/adjacency_list.html
 */
template <uint8_t N> class GraphN {
  static_assert(N == 4 || N == 8, "neighborhood must be 4 or 8");

public:
  static constexpr Cost INF = std::numeric_limits<Cost>::infinity();
  /// Compile-time neighbourhood, 4 or 8.
  static constexpr uint8_t kNeighborhood = N;

  explicit GraphN(const Grid &grid, const Costs &max_costs = Costs())
      : grid_(grid), max_costs_(max_costs), cell_size_(grid.cellSize()) {
    const size_t n = grid_.size();
    total_.resize(n);
    for (size_t v = 0; v < n; ++v) {
      const Costs &c = grid_.costs(static_cast<CellId>(v));
      // An out-of-bounds cell is stored as INF rather than in a second flag
      // array: 1 + (INF + x) / 2, scaled, is still exactly INF, which is what
      // cost() returned for it before, and the inner loop keeps one branch.
      total_[v] = grid::costsInBounds(c, max_costs_)
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
  inline EdgeId num_edges() const { return N * num_vertices(); }
  inline std::pair<VertexIter, VertexIter> vertices() const {
    return {VertexIter(0), VertexIter(num_vertices())};
  }
  inline std::pair<EdgeIter, EdgeIter> out_edges(const VertexId &u) const {
    return {EdgeIter(N * u), EdgeIter(N * (u + 1))};
  }
  inline EdgeId out_degree(const VertexId &) const { return N; }
  inline VertexId source(const EdgeId &e) const { return e / N; }
  inline VertexId target_index(const EdgeId &e) const { return e % N; }

  /// Index into Grid's 8-slot neighbour row for direction @p i of this
  /// neighbourhood: neighbor8(c, 2 * i) == neighbor4(c, i).
  static inline int dirIndex(const VertexId i) {
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
    const CellId t = grid_.neighborId(s, dirIndex(target_index(e)));
    return t == INVALID_CELL_ID ? s : t;
  }

  /// True if every bounded layer of @p costs is within max_costs.
  bool costsInBounds(const Costs &costs) const {
    return grid::costsInBounds(costs, max_costs_);
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
    const CellId v = grid_.neighborId(u, dirIndex(i));
    if (v == INVALID_CELL_ID) {
      return INF;
    }
    // Expression and multiplication order kept byte for byte; the exact-cost
    // planning tests pin the accumulated float.
    Cost cost = 1 + (total_[u] + total_[v]) / 2;
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

/// The 8-neighbourhood graph; the default everywhere.
typedef GraphN<8> Graph;
/// The 4-neighbourhood graph.
typedef GraphN<4> Graph4;

template <uint8_t N> class EdgeCostsN {
public:
  EdgeCostsN(const GraphN<N> &graph) : graph_(graph) {}
  inline Cost operator[](const EdgeId &e) const { return graph_.cost(e); }

protected:
  const GraphN<N> &graph_;
};

typedef EdgeCostsN<8> EdgeCosts;

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

  typedef bidirectional_traversal_tag traversal_category;
  typedef naex::grid::VertexIter vertex_iterator;
  typedef naex::grid::EdgeIter out_edge_iterator;
};

template <uint8_t N> class property_traits<naex::grid::EdgeCostsN<N>> {
public:
  typedef naex::grid::EdgeId key_type;
  typedef naex::grid::Cost value_type;
  typedef naex::grid::Cost reference;
  typedef readable_property_map_tag category;
};

} // namespace boost
