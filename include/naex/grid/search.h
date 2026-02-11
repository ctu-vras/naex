#include "naex/grid/graph.h"
#include "naex/grid/grid.h"
#include <boost/graph/dijkstra_shortest_paths_no_color_map.hpp>

namespace naex {
namespace grid {


bool is_obstacle(VertexId v, const Grid &grid, const std::vector<double> &cost_thresholds, const std::vector<double> &cloud_weights) {
  for (int i{0}; i < cost_thresholds.size(); ++i) {
    Cost c = grid.costs(v).data[i];
    // RCLCPP_INFO(nh_->get_logger(), "filter_obstacles %u cost being %f with weight %f and threshold %f", filter_obstacles_, c, cloud_weights_[i], cost_thresholds_[i]);
    if ((c > cloud_weights[i] * cost_thresholds[i])) {
      // RCLCPP_INFO(nh_->get_logger(), "node thresholded due to level %d cost being %f with weight %f and threshold %f", i, c, cloud_weights_[i], cost_thresholds_[i]);
      return true;
    }
  }
  return false;
}

class ShortestPaths {
public:
  ShortestPaths(const Grid &grid, VertexId start, uint8_t neighborhood = 8,
                const Costs &max_costs_ = Costs(0.0))
      : graph_(grid, neighborhood, max_costs_), edge_costs_(graph_),
        predecessor_(graph_.num_vertices(),
                     std::numeric_limits<VertexId>::max()),
        path_costs_(graph_.num_vertices(),
                    std::numeric_limits<Cost>::infinity()) {
    boost::typed_identity_property_map<VertexId> index_map;
    boost::dijkstra_shortest_paths_no_color_map(
        graph_, start, predecessor_.data(), path_costs_.data(), edge_costs_,
        index_map, std::less<Cost>(), boost::closed_plus<Cost>(),
        std::numeric_limits<Cost>::infinity(), Cost(0.),
        boost::dijkstra_visitor<boost::null_visitor>());
  }

  const std::vector<VertexId> &predecessors() const { return predecessor_; }
  const std::vector<Cost> &pathCosts() const { return path_costs_; }

  const VertexId &predecessor(VertexId v) const { return predecessor_[v]; }
  const Cost &pathCost(VertexId v) const { return path_costs_[v]; }

protected:
  Graph graph_;
  EdgeCosts edge_costs_;
  std::vector<VertexId> predecessor_;
  std::vector<Cost> path_costs_;
};

} // namespace grid
} // namespace naex
