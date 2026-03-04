#include "naex/grid/graph.h"
#include "naex/grid/grid.h"
#include <boost/graph/dijkstra_shortest_paths_no_color_map.hpp>
#include <boost/graph/astar_search.hpp>
#include <boost/graph/filtered_graph.hpp>

namespace naex {
namespace grid {


// Used to filter vertices too far away from the starting position.
// This is meant to speed things up in case the map grows too big.
struct VertexFilter
{
  const Grid* grid_;
  float max_range_;
  VertexId start_;
  int *num_vertices_out_of_range_;

  VertexFilter() = default;

  VertexFilter(const Grid& grid,
               float max_range,
               VertexId start,
               int *num_vertices_out_of_range
              )
    : grid_(&grid)
    , max_range_(max_range)
    , start_(start)
    , num_vertices_out_of_range_(num_vertices_out_of_range)
  {}

  bool operator()(VertexId v) const
  {
    auto pu = grid_->point(v);
    auto ps = grid_->point(start_);
    double dx = pu.x - ps.x;
    double dy = pu.y - ps.y;
    double dist = std::sqrt(dx*dx + dy*dy);

    if (dist > max_range_) {
      (*num_vertices_out_of_range_)++;
      return false;
    }
    return true;
  }
};


// Heuristic function for A*
// Euclidean distance to goal.
class AStarHeuristic : public boost::astar_heuristic<Graph, Cost> {
public:
  AStarHeuristic(const Grid& grid, Vec3 goal_point) 
    : grid_(grid), goal_point_(goal_point) {}
  
  Cost operator()(VertexId u) {
    // Euclidean distance heuristic
    auto point_u = grid_.point(u);
    double dx = point_u.x - goal_point_.x();
    double dy = point_u.y - goal_point_.y();
    return std::sqrt(dx*dx + dy*dy);
  }
  
private:
  const Grid& grid_;
  Vec3 goal_point_;
};

// Custom visitor to catch goal (if it is reachable) and to stop the
// search if we hit an obstacle vertex (which means we have explored all the traversable vertices).
struct astar_goal_visitor : public boost::default_astar_visitor {
  astar_goal_visitor(VertexId goal, bool is_goal_explored, const Grid &grid, const Costs &max_costs_absolute)
    : goal_(goal), is_goal_explored_(is_goal_explored), grid_(grid), max_costs_absolute_(max_costs_absolute) {}
  
  // The function called when a vertex is popped.
  void examine_vertex(VertexId u, const boost::filtered_graph<Graph, boost::keep_all, VertexFilter>&) {
    if (!naex::grid::costsInBounds(grid_.costs(u), max_costs_absolute_)) {
      // Assume that the first time you visit an infinite cost vertex (obstacle),
      // you have already visited all finite cost vertices (traversable).
      throw goal_not_found();
    }
    else if (is_goal_explored_ && u == goal_) {
      throw goal_found();
    }
  }
  
  // Exceptions to return.
  struct goal_found {};
  struct goal_not_found {};
  
  private:
    VertexId goal_;
    bool is_goal_explored_;
    const Grid &grid_;
    const Costs &max_costs_absolute_;
};

class ShortestPaths {
public:
  ShortestPaths(const rclcpp::Node::SharedPtr nh,
                const Grid &grid,
                VertexId start,
                Vec3 goal_point,
                bool is_goal_explored,
                bool use_astar,
                float astar_max_range,
                uint8_t neighborhood = 8,
                const Costs &max_costs_absolute = Costs(0.0))
      : graph_(grid, neighborhood, max_costs_absolute), edge_costs_(graph_),
        predecessor_(graph_.num_vertices(),
                     std::numeric_limits<VertexId>::max()),
        path_costs_(graph_.num_vertices(),
                    std::numeric_limits<Cost>::infinity()),
        f_values_(graph_.num_vertices(),
                    std::numeric_limits<Cost>::infinity()),
        visited_(graph_.num_vertices(), 0) {


    // Filter graph vertices that are too far from the start to speed up search.
    int num_vertices_out_of_range{0};
    VertexFilter vf(
        graph_.grid(),
        astar_max_range,
        start,
        &num_vertices_out_of_range
    );

    filtered_graph_ = std::make_shared<boost::filtered_graph<Graph, boost::keep_all, VertexFilter>>(
      boost::make_filtered_graph(graph_, boost::keep_all(), vf)
    );

    boost::typed_identity_property_map<VertexId> index_map;

    if (use_astar) {

      // Prepare visitor. We only want to stop on goal if the goal is in the explored part of the grid,
      // so figure that out here.
      VertexId goal = INVALID_VERTEX;
      if (is_goal_explored) {
        // Only get the goal vertex if we want to stop searching on hitting goal.
        goal = graph_.grid().cellId(graph_.grid().pointToCell({goal_point.x(), goal_point.y()}));
      }
      astar_goal_visitor visitor(goal, is_goal_explored, graph_.grid(), max_costs_absolute);

      // Prepare color map for A* to keep note of visited vertices.
      colors_.assign(boost::num_vertices(graph_), boost::white_color);
      auto color_map = boost::make_iterator_property_map(
          colors_.begin(),
          boost::identity_property_map()
      );

      // Prepare the (euclidean dist) heuristic.
      AStarHeuristic heuristic(graph_.grid(), goal_point);

      // Run A*
      try {
        boost::astar_search(
          *filtered_graph_, start, heuristic,
          boost::predecessor_map(predecessor_.data())
            .distance_map(path_costs_.data())
            .weight_map(edge_costs_)
            .rank_map(f_values_.data())
            .vertex_index_map(index_map)
            .visitor(visitor)
            .color_map(color_map)
        );
        RCLCPP_INFO(nh->get_logger(), "AStar did not find goal.");
      } catch (astar_goal_visitor::goal_found&) {
        // Goal found successfully
        // Reconstruct path using predecessor_map if needed
        RCLCPP_INFO(nh->get_logger(), "AStar found goal.");
        astar_found_goal_ = true;
      } catch (astar_goal_visitor::goal_not_found&) {
        // Obstacle vertex visited, goal cannot be found.
        RCLCPP_INFO(nh->get_logger(), "AStar did not find goal.");
        astar_found_goal_ = false;
      }

      for (std::size_t i = 0; i < colors_.size(); ++i) {
        visited_[i] = (colors_[i] != boost::white_color) ? 1 : 0;
      }
      RCLCPP_INFO(nh->get_logger(), "Filtered out %d out of range (> %f m) points.", num_vertices_out_of_range, astar_max_range);//, num_obstacle_vertices);

    } else {
      boost::dijkstra_shortest_paths_no_color_map(
          *filtered_graph_, start, predecessor_.data(), path_costs_.data(), edge_costs_,
          index_map, std::less<Cost>(), boost::closed_plus<Cost>(),
          std::numeric_limits<Cost>::infinity(), Cost(0.),
          boost::dijkstra_visitor<boost::null_visitor>());      
    }    
  }

  const std::vector<VertexId> &predecessors() const { return predecessor_; }
  const std::vector<Cost> &pathCosts() const { return path_costs_; }
  const std::vector<Cost> &fValues() const { return f_values_; }
  const std::vector<std::uint8_t>& visited() const { return visited_; }

  const VertexId &predecessor(VertexId v) const { return predecessor_[v]; }
  const Cost &pathCost(VertexId v) const { return path_costs_[v]; }
  const Cost &fValue(VertexId v) const { return f_values_[v]; }

  const bool astar_found_goal() const {return astar_found_goal_; }
  const Graph &full_graph() const {return graph_; }
  const boost::filtered_graph<Graph,boost::keep_all,VertexFilter> &filtered_graph() const {return *filtered_graph_; }

  // This is used for the case where we do find a path to goal, but it is very long which prompts
  // us to also consider traveling to some frontier instead in search of a more efficient path.
  // This solves the "Traveling in a circle with the end being next to the start in an explored
  // part of the grid" edge case, which causes the robot to travel to the final point by going
  // back through the explored part of the grid, instead of pushing through the unexplored.
  const Value cheapest_path_euclidean_dist(const rclcpp::Node::SharedPtr nh, VertexId v_start, VertexId v_goal) const {
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
  std::shared_ptr<boost::filtered_graph<Graph,boost::keep_all,VertexFilter>> filtered_graph_;
  EdgeCosts edge_costs_;
  std::vector<VertexId> predecessor_;
  std::vector<Cost> path_costs_;
  std::vector<Cost> f_values_;

  // For AStar require knowing which vertices we have visited.
  std::vector<boost::default_color_type> colors_;
  std::vector<std::uint8_t> visited_;

  bool astar_found_goal_{false};
};

// This is used to find connected frontier components in search of the optimal frontier to travel to,
// if we cannot plan straight to the goal.
class BFS {
  public:
    BFS(const rclcpp::Node::SharedPtr nh,
        const Grid &grid,
        VertexId nearest_frontier,
        const Costs &max_costs_absolute = Costs(0.0))
      : nh_(nh),
        graph_(grid, 8, max_costs_absolute),
        visited_(boost::num_vertices(graph_), 0) {

      colors_.assign(boost::num_vertices(graph_), boost::white_color);
      auto color_map = boost::make_iterator_property_map(
        colors_.begin(),
        boost::identity_property_map()
      );

      boost::breadth_first_search(
        graph_,
        nearest_frontier,
        boost::visitor(boost::default_bfs_visitor())
          .vertex_index_map(boost::identity_property_map())
          .color_map(color_map)
      );

      // Populate visited_ from color map
      for (std::size_t i = 0; i < colors_.size(); ++i) {
        visited_[i] = (colors_[i] != boost::white_color) ? 1 : 0;
      }

      RCLCPP_WARN(nh_->get_logger(), "BFS finished visiting %lu vertices.", visited_.size());
    }

    const std::vector<std::uint8_t>& visited() const { return visited_; }

  protected:
    rclcpp::Node::SharedPtr nh_;
    Graph graph_;
    std::vector<std::uint8_t> visited_;
    std::vector<boost::default_color_type> colors_;
};

} // namespace grid
} // namespace naex
