#include "naex/grid/graph.h"
#include "naex/grid/grid.h"
#include <boost/graph/dijkstra_shortest_paths_no_color_map.hpp>
#include <boost/graph/astar_search.hpp>
#include <boost/graph/filtered_graph.hpp>

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

struct VertexFilter
{
  const Graph* g_;
  float max_range_;
  VertexId start_;
  bool filter_obstacles_;
  const std::vector<double>* cost_thresholds_;
  const std::vector<double>* cloud_weights_;
  int *num_vertices_out_of_range_;
  int *num_obstacle_vertices_;

  VertexFilter() = default;

  VertexFilter(const Graph& graph,
               float max_range,
               VertexId start,
               bool filter_obstacles,
               const std::vector<double>& cost_thresholds,
               const std::vector<double>& cloud_weights,
               int *num_vertices_out_of_range,
               int *num_obstacle_vertices
              )
    : g_(&graph)
    , max_range_(max_range)
    , start_(start)
    , filter_obstacles_(filter_obstacles)
    , cost_thresholds_(&cost_thresholds)
    , cloud_weights_(&cloud_weights)
    , num_vertices_out_of_range_(num_vertices_out_of_range)
    , num_obstacle_vertices_(num_obstacle_vertices)
  {}

  bool operator()(VertexId v) const
  {
    auto pu = g_->grid().point(v);
    auto ps = g_->grid().point(start_);
    double dx = pu.x - ps.x;
    double dy = pu.y - ps.y;
    double dist = std::sqrt(dx*dx + dy*dy);

    if (dist > max_range_) {
      (*num_vertices_out_of_range_)++;
      return false;
    }

    if (filter_obstacles_) {
      bool is_obstacle_b = is_obstacle(v, g_->grid(), *cost_thresholds_, *cloud_weights_);
  
      if (is_obstacle_b) {
        (*num_obstacle_vertices_)++;
        return false;
      }
    }

    return true;
  }
};


// Heuristic function for A*
class AStarHeuristic : public boost::astar_heuristic<Graph, Cost> {
public:
  AStarHeuristic(const rclcpp::Node::SharedPtr nh, const Graph& graph, VertexId goal) 
    : nh_(nh), graph_(graph), goal_(goal) {}
  
  Cost operator()(VertexId u) {
    // Euclidean distance heuristic
    auto point_u = graph_.grid().point(u);
    auto point_goal = graph_.grid().point(goal_);
    double dx = point_u.x - point_goal.x;
    double dy = point_u.y - point_goal.y;
    return std::sqrt(dx*dx + dy*dy);
  }
  
private:
  const rclcpp::Node::SharedPtr nh_;
  const Graph& graph_;
  VertexId goal_;
};

// Custom visitor to catch goal
struct astar_goal_visitor : public boost::default_astar_visitor {
  astar_goal_visitor(VertexId goal)
    : goal_(goal) {}
  
  void examine_vertex(VertexId u, const boost::filtered_graph<Graph, boost::keep_all, VertexFilter>&) {
    if (u == goal_) {
      throw goal_found();
    }
  }
  
  struct goal_found {};
  
private:
  VertexId goal_;
};

class ShortestPaths {
public:
  ShortestPaths(const rclcpp::Node::SharedPtr nh,
                const Grid &grid,
                VertexId start,
                VertexId goal,
                bool use_astar,
                float astar_max_range,
                std::vector<double> cost_thresholds,
                std::vector<double> cloud_weights,
                uint8_t neighborhood = 8,
                const Costs &max_costs_ = Costs(0.0))
      : graph_(grid, neighborhood, max_costs_), edge_costs_(graph_),
        predecessor_(graph_.num_vertices(),
                     std::numeric_limits<VertexId>::max()),
        path_costs_(graph_.num_vertices(),
                    std::numeric_limits<Cost>::infinity()),
        f_values_(graph_.num_vertices(),
                    std::numeric_limits<Cost>::infinity()) {
    
    bool filter_obstacles = false;
    for (Cost c : cost_thresholds) {
      if (c >= 0. && c<= 1.) {
        filter_obstacles = true;
      }
    }

    int num_vertices_out_of_range;
    int num_obstacle_vertices;

    VertexFilter vf(
        graph_,
        astar_max_range,
        start,
        filter_obstacles,
        cost_thresholds,
        cloud_weights,
        &num_vertices_out_of_range,
        &num_obstacle_vertices
    );

    auto filtered_graph = boost::make_filtered_graph(
      graph_,
      boost::keep_all(), // edge filter
      vf
    );

    boost::typed_identity_property_map<VertexId> index_map;
    if (use_astar) {
      AStarHeuristic heuristic(nh, graph_, goal);
      astar_goal_visitor visitor(goal);
      
      // Run A*
      try {
        boost::astar_search(
          filtered_graph, start, heuristic,
          boost::predecessor_map(predecessor_.data())
            .distance_map(path_costs_.data())
            .weight_map(edge_costs_)
            .rank_map(f_values_.data())
            .vertex_index_map(index_map)
            .visitor(visitor)
        );
        RCLCPP_INFO(nh->get_logger(), "AStar did not find goal.");
      } catch (astar_goal_visitor::goal_found&) {
        // Goal found successfully
        // Reconstruct path using predecessor_map if needed
        RCLCPP_INFO(nh->get_logger(), "AStar found goal.");
        astar_found_goal_ = true;
      }
    } else {
      boost::dijkstra_shortest_paths_no_color_map(
          filtered_graph, start, predecessor_.data(), path_costs_.data(), edge_costs_,
          index_map, std::less<Cost>(), boost::closed_plus<Cost>(),
          std::numeric_limits<Cost>::infinity(), Cost(0.),
          boost::dijkstra_visitor<boost::null_visitor>());      
    }
    RCLCPP_INFO(nh->get_logger(), "Filtered out %d out of range and %d obstacle vertices.", num_vertices_out_of_range, num_obstacle_vertices);
  }

  const std::vector<VertexId> &predecessors() const { return predecessor_; }
  const std::vector<Cost> &pathCosts() const { return path_costs_; }
  const std::vector<Cost> &fValues() const { return f_values_; }

  const VertexId &predecessor(VertexId v) const { return predecessor_[v]; }
  const Cost &pathCost(VertexId v) const { return path_costs_[v]; }
  const Cost &fValue(VertexId v) const { return f_values_[v]; }

  const bool astar_found_goal() const {return astar_found_goal_; }
  const Graph &graph() const {return graph_; }

  const Value cheapest_path_euclidean_dist(VertexId v_start, VertexId v_goal) const {
    // iterate all predecessors
    VertexId v_pred_previous = INVALID_VERTEX;
    Value path_to_goal_dist{0.};

    auto predecessors = this->predecessors();

    assert(predecessors[v_start] == v_start);
    Vertex v = v_goal;
    while (v != v_start) {
      if (v_pred_previous != INVALID_VERTEX) {
        float x1 = graph_.grid().point(v).x;
        float x2 = graph_.grid().point(v_pred_previous).x;
        float y1 = graph_.grid().point(v).y;
        float y2 = graph_.grid().point(v_pred_previous).y;
        path_to_goal_dist += std::sqrt((x1-x2)*(x1-x2) + (y1-y2)*(y1-y2));
      }
      v_pred_previous = v;
      v = predecessors[v];
    }
      
    return path_to_goal_dist;
  }


protected:
  Graph graph_;
  EdgeCosts edge_costs_;
  std::vector<VertexId> predecessor_;
  std::vector<Cost> path_costs_;
  std::vector<Cost> f_values_;
  bool astar_found_goal_{false};
};

} // namespace grid
} // namespace naex
