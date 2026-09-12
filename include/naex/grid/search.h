#pragma once

#include "naex/grid/graph.h"
#include "naex/grid/grid.h"
#include <boost/graph/dijkstra_shortest_paths_no_color_map.hpp>
#include <functional>
#include <limits>
#include <vector>

namespace naex {
namespace grid {

/**
 * Single-source shortest paths over a Grid.
 *
 * Reusable (P2): compute() keeps the predecessor and path-cost buffers across
 * calls, so a repeated search on a grid of the same size allocates nothing —
 * Boost initialises both maps itself, so resizing them is all that is needed.
 * The runtime @p neighborhood selects between the two compile-time GraphN
 * instantiations; everything else is shared.
 */
class ShortestPaths {
public:
  ShortestPaths() = default;
  ShortestPaths(const Grid &grid, VertexId start, uint8_t neighborhood = 8,
                const Costs &max_costs = Costs(0.0)) {
    compute(grid, start, neighborhood, max_costs);
  }

  /// Run the search; anything but neighborhood == 4 is the 8-neighbourhood,
  /// as before.
  void compute(const Grid &grid, VertexId start, uint8_t neighborhood = 8,
               const Costs &max_costs = Costs(0.0)) {
    if (neighborhood == 4) {
      run<4>(grid, start, max_costs);
    } else {
      run<8>(grid, start, max_costs);
    }
  }

  const std::vector<VertexId> &predecessors() const { return predecessor_; }
  const std::vector<Cost> &pathCosts() const { return path_costs_; }

  const VertexId &predecessor(VertexId v) const { return predecessor_[v]; }
  const Cost &pathCost(VertexId v) const { return path_costs_[v]; }

protected:
  template <uint8_t N>
  void run(const Grid &grid, VertexId start, const Costs &max_costs) {
    const GraphN<N> graph(grid, max_costs);
    const EdgeCostsN<N> edge_costs(graph);
    // dijkstra_shortest_paths_no_color_map() sets every distance to infinity
    // and every predecessor to the vertex itself before it starts, so the
    // buffers only have to be large enough.
    predecessor_.resize(graph.num_vertices());
    path_costs_.resize(graph.num_vertices());
    boost::typed_identity_property_map<VertexId> index_map;
    boost::dijkstra_shortest_paths_no_color_map(
        graph, start, predecessor_.data(), path_costs_.data(), edge_costs,
        index_map, std::less<Cost>(), boost::closed_plus<Cost>(),
        std::numeric_limits<Cost>::infinity(), Cost(0.),
        boost::dijkstra_visitor<boost::null_visitor>());
  }

  std::vector<VertexId> predecessor_;
  std::vector<Cost> path_costs_;
};

} // namespace grid
} // namespace naex
