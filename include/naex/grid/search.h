#pragma once

#include "naex/grid/graph.h"
#include "naex/grid/grid.h"
#include "naex/types.h"
#include <boost/graph/astar_search.hpp>
#include <boost/graph/breadth_first_search.hpp>
#include <boost/graph/dijkstra_shortest_paths_no_color_map.hpp>
#include <boost/graph/filtered_graph.hpp>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <limits>
#include <rclcpp/rclcpp.hpp>
#include <vector>

namespace naex {
namespace grid {

/// Euclidean distance between the centres of cells @p u and @p v.
inline Value cell_distance(const Grid &grid, VertexId u, VertexId v) {
  const Point2f a = grid.point(u);
  const Point2f b = grid.point(v);
  const float dx = a.x - b.x;
  const float dy = a.y - b.y;
  return std::sqrt(dx * dx + dy * dy);
}

/**
 * True if the centre of cell @p v is within @p max_range of the centre of
 * cell @p start.
 *
 * This is the A* range crop; it is exposed because the frontier detection in
 * the planner has to agree with it cell for cell.
 */
inline bool within_range(const Grid &grid, VertexId v, VertexId start,
                         float max_range) {
  return !(cell_distance(grid, v, start) > max_range);
}

/**
 * Vertex predicate of the A* search: drops cells farther than @p max_range_
 * from the start cell so the search does not walk a map that has grown far
 * beyond the current neighbourhood.
 */
struct MaxRangeVertexFilter {
  const Grid *grid_{nullptr};
  float max_range_{std::numeric_limits<float>::infinity()};
  VertexId start_{0};
  size_t *num_vertices_out_of_range_{nullptr};

  MaxRangeVertexFilter() = default;

  MaxRangeVertexFilter(const Grid &grid, float max_range, VertexId start,
                       size_t *num_vertices_out_of_range)
      : grid_(&grid), max_range_(max_range), start_(start),
        num_vertices_out_of_range_(num_vertices_out_of_range) {}

  bool operator()(VertexId v) const {
    if (within_range(*grid_, v, start_, max_range_)) {
      return true;
    }
    if (num_vertices_out_of_range_) {
      ++(*num_vertices_out_of_range_);
    }
    return false;
  }
};

/**
 * Heuristic of the A* search: straight-line distance from a cell centre to the
 * goal point.
 *
 * Admissible for the edge costs of GraphN: an edge costs
 * (1 + mean cell cost) * its Euclidean length, and cell costs are
 * non-negative, so no edge is ever cheaper than its length.
 */
template <uint8_t N>
class AStarHeuristic : public boost::astar_heuristic<GraphN<N>, Cost> {
public:
  AStarHeuristic(const Grid &grid, const Vec3 &goal_point)
      : grid_(grid), goal_point_(goal_point) {}

  Cost operator()(VertexId u) const {
    const Point2f p = grid_.point(u);
    const double dx = p.x - goal_point_.x();
    const double dy = p.y - goal_point_.y();
    return static_cast<Cost>(std::sqrt(dx * dx + dy * dy));
  }

private:
  const Grid &grid_;
  Vec3 goal_point_;
};

/**
 * A* visitor that stops the search on the goal cell, and also as soon as the
 * first untraversable cell is popped.
 *
 * The second condition is what makes "visited" mean "reachable": because the
 * queue is ordered by f and every traversable cell has a finite f, popping an
 * out-of-bounds cell means every traversable cell in range has already been
 * expanded.
 */
struct AstarGoalVisitor : public boost::default_astar_visitor {
  AstarGoalVisitor(VertexId goal, bool stop_on_goal, const Grid &grid,
                   const Costs &max_costs)
      : goal_(goal), stop_on_goal_(stop_on_goal), grid_(&grid),
        max_costs_(max_costs) {}

  /// Exceptions used to leave boost::astar_search early.
  struct GoalFound {};
  struct GoalNotFound {};

  template <typename G> void examine_vertex(VertexId u, const G &) const {
    if (!naex::grid::costs_in_bounds(grid_->costs(u), max_costs_)) {
      throw GoalNotFound();
    }
    if (stop_on_goal_ && u == goal_) {
      throw GoalFound();
    }
  }

private:
  VertexId goal_;
  bool stop_on_goal_;
  const Grid *grid_;
  Costs max_costs_;
};

/**
 * Shortest paths over a Grid, by Dijkstra (whole grid) or by A* (to a goal
 * point, cropped to a radius around the start).
 *
 * One class for both searches, so that the planner and the mule planner can
 * hold it as a value and hand it to the same map-cloud publisher.  It is
 * reusable (P2): compute() and compute_astar() keep the predecessor, path-cost,
 * f-value and colour buffers across calls, so a repeated search on a grid of
 * the same size allocates nothing.  The runtime @p neighborhood selects
 * between the two compile-time GraphN instantiations; everything else is
 * shared.
 *
 * The graph (and with it the per-vertex cost cache of GraphN) is built inside
 * every call, so a cost threshold changed by the parameter callback, an ad-hoc
 * cost layer rewritten just before the search, and a grid compacted by the P6
 * eviction are all picked up by the next search without any explicit
 * invalidation.
 */
class ShortestPaths {
public:
  static constexpr Cost INF = std::numeric_limits<Cost>::infinity();
  /**
   * Path costs and f values above this mean "not reached".
   *
   * boost::astar_search's default distance_inf is
   * std::numeric_limits<Cost>::max(), not infinity, so after an A* run an
   * unreached in-range cell carries a *finite* FLT_MAX (an out-of-range cell,
   * which boost never even initializes, keeps the INF this class filled in).
   * Test against this constant, never with std::isfinite, after an A* run.
   * After a Dijkstra run unreached cells are plain infinity and either test
   * works.
   */
  static constexpr Cost kUnreachableCost = 1e9f;

  ShortestPaths() = default;
  ShortestPaths(const Grid &grid, VertexId start, uint8_t neighborhood = 8,
                const Costs &max_costs = Costs(0.0)) {
    compute(grid, start, neighborhood, max_costs);
  }

  /// Dijkstra from @p start over the whole grid; anything but
  /// neighborhood == 4 is the 8-neighbourhood, as before.
  void compute(const Grid &grid, VertexId start, uint8_t neighborhood = 8,
               const Costs &max_costs = Costs(0.0)) {
    astar_ = false;
    found_goal_ = false;
    if (neighborhood == 4) {
      run<4>(grid, start, max_costs);
    } else {
      run<8>(grid, start, max_costs);
    }
  }

  /**
   * A* from @p start toward @p goal_point.
   *
   * @param is_goal_explored true when @p goal_point falls on an existing cell;
   *        only then can the search stop on the goal.
   * @param max_range cells farther than this from the start cell are not
   *        expanded at all.
   * @return true if the goal cell was reached (also available as found_goal()).
   */
  bool compute_astar(const rclcpp::Logger &log, const Grid &grid,
                     VertexId start, const Vec3 &goal_point,
                     bool is_goal_explored, float max_range,
                     uint8_t neighborhood = 8,
                     const Costs &max_costs = Costs(0.0)) {
    astar_ = true;
    found_goal_ = false;
    if (neighborhood == 4) {
      run_astar<4>(log, grid, start, goal_point, is_goal_explored, max_range,
                   max_costs);
    } else {
      run_astar<8>(log, grid, start, goal_point, is_goal_explored, max_range,
                   max_costs);
    }
    return found_goal_;
  }

  const std::vector<VertexId> &predecessors() const { return predecessor_; }
  const std::vector<Cost> &path_costs() const { return path_costs_; }
  /// A* f = g + h per cell; all zeros after a Dijkstra run.
  const std::vector<Cost> &f_values() const { return f_values_; }
  /// 1 for every cell the search expanded, i.e. every reachable cell.
  const std::vector<std::uint8_t> &visited() const { return visited_; }

  const VertexId &predecessor(VertexId v) const { return predecessor_[v]; }
  const Cost &path_cost(VertexId v) const { return path_costs_[v]; }
  const Cost &f_value(VertexId v) const { return f_values_[v]; }

  /// True if the last search was an A* search.
  bool is_astar() const { return astar_; }
  /// True if the last A* search reached its goal cell.
  bool found_goal() const { return found_goal_; }

  /**
   * Length, in metres, of the predecessor path from @p v_goal back to
   * @p v_start.
   *
   * Used to compare the cost-optimal route against the straight line to the
   * goal: a route that is much longer than the crow-flies distance is the
   * "drive all the way around an explored loop" case the frontier selection
   * exists to avoid.
   */
  Value cheapest_path_euclidean_dist(const Grid &grid, VertexId v_start,
                                     VertexId v_goal) const {
    if (v_goal == INVALID_VERTEX_ID || v_start == INVALID_VERTEX_ID) {
      return std::numeric_limits<Value>::infinity();
    }
    assert(predecessor_[v_start] == v_start);
    VertexId previous = INVALID_VERTEX_ID;
    Value dist = 0.;
    VertexId v = v_goal;
    while (v != v_start) {
      if (previous != INVALID_VERTEX_ID) {
        dist += cell_distance(grid, v, previous);
      }
      previous = v;
      const VertexId pred = predecessor_[v];
      if (pred == v || pred == INVALID_VERTEX_ID) {
        // Not connected to the start; the caller treats infinity as "too far".
        return std::numeric_limits<Value>::infinity();
      }
      v = pred;
    }
    return dist;
  }

protected:
  template <uint8_t N>
  void run(const Grid &grid, VertexId start, const Costs &max_costs) {
    const GraphN<N> graph(grid, max_costs);
    const EdgeCostsN<N> edge_costs(graph);
    const size_t n = graph.num_vertices();
    // dijkstra_shortest_paths_no_color_map() sets every distance to infinity
    // and every predecessor to the vertex itself before it starts, so the
    // buffers only have to be large enough.
    predecessor_.resize(n);
    path_costs_.resize(n);
    boost::typed_identity_property_map<VertexId> index_map;
    boost::dijkstra_shortest_paths_no_color_map(
        graph, start, predecessor_.data(), path_costs_.data(), edge_costs,
        index_map, std::less<Cost>(), boost::closed_plus<Cost>(),
        std::numeric_limits<Cost>::infinity(), Cost(0.),
        boost::dijkstra_visitor<boost::null_visitor>());
    // Dijkstra has no heuristic; f is zeroed so that the map cloud has the
    // same fields whichever search ran.  Reachable == finite path cost.
    f_values_.assign(n, Cost(0.));
    visited_.resize(n);
    for (size_t v = 0; v < n; ++v) {
      visited_[v] = std::isfinite(path_costs_[v]) ? 1 : 0;
    }
  }

  template <uint8_t N>
  void run_astar(const rclcpp::Logger &log, const Grid &grid, VertexId start,
                 const Vec3 &goal_point, bool is_goal_explored, float max_range,
                 const Costs &max_costs) {
    typedef boost::filtered_graph<GraphN<N>, boost::keep_all,
                                  MaxRangeVertexFilter>
        FilteredGraph;

    const GraphN<N> graph(grid, max_costs);
    const EdgeCostsN<N> edge_costs(graph);
    const size_t n = graph.num_vertices();

    // boost::astar_search initializes only the vertices the filter keeps, so
    // unlike the Dijkstra above these buffers must be filled here.
    predecessor_.assign(n, INVALID_VERTEX_ID);
    path_costs_.assign(n, INF);
    f_values_.assign(n, INF);
    colors_.assign(n, boost::white_color);
    visited_.assign(n, 0);

    size_t num_vertices_out_of_range = 0;
    const MaxRangeVertexFilter filter(grid, max_range, start,
                                      &num_vertices_out_of_range);
    const FilteredGraph filtered(graph, boost::keep_all(), filter);

    // Stopping on the goal only makes sense when the goal is a cell we have.
    VertexId goal = INVALID_VERTEX_ID;
    if (is_goal_explored) {
      goal =
          grid.find_cell(grid.point_to_cell({goal_point.x(), goal_point.y()}));
    }
    const AstarGoalVisitor visitor(goal, is_goal_explored, grid, max_costs);
    const AStarHeuristic<N> heuristic(grid, goal_point);

    boost::typed_identity_property_map<VertexId> index_map;
    auto color_map = boost::make_iterator_property_map(
        colors_.begin(), boost::identity_property_map());

    try {
      boost::astar_search(filtered, start, heuristic,
                          boost::predecessor_map(predecessor_.data())
                              .distance_map(path_costs_.data())
                              .weight_map(edge_costs)
                              .rank_map(f_values_.data())
                              .vertex_index_map(index_map)
                              .visitor(visitor)
                              .color_map(color_map));
      RCLCPP_INFO(log, "AStar exhausted the graph without reaching the goal.");
    } catch (const AstarGoalVisitor::GoalFound &) {
      RCLCPP_INFO(log, "AStar found goal.");
      found_goal_ = true;
    } catch (const AstarGoalVisitor::GoalNotFound &) {
      RCLCPP_INFO(log, "AStar did not find goal.");
      found_goal_ = false;
    }

    for (size_t v = 0; v < n; ++v) {
      visited_[v] = (colors_[v] != boost::white_color) ? 1 : 0;
    }
    RCLCPP_INFO(log, "Filtered out %lu out of range (> %f m) points.",
                static_cast<unsigned long>(num_vertices_out_of_range),
                static_cast<double>(max_range));
  }

  std::vector<VertexId> predecessor_;
  std::vector<Cost> path_costs_;
  std::vector<Cost> f_values_;
  std::vector<std::uint8_t> visited_;
  std::vector<boost::default_color_type> colors_;
  bool astar_{false};
  bool found_goal_{false};
};

/**
 * Breadth-first search over a Grid, used to group frontier cells into
 * connected components.
 *
 * Unlike ShortestPaths this ignores costs entirely; only the 8-neighbourhood
 * connectivity of the cells matters.  The graph is built once and run() may be
 * called for as many seeds as needed, which is what the component loop of the
 * frontier selection does (it used to rebuild the graph per seed).
 */
class BFS {
public:
  explicit BFS(const Grid &grid, const Costs &max_costs = Costs(0.0))
      : graph_(grid, max_costs) {}

  void run(VertexId seed) {
    const size_t n = graph_.num_vertices();
    colors_.assign(n, boost::white_color);
    visited_.assign(n, 0);
    auto color_map = boost::make_iterator_property_map(
        colors_.begin(), boost::identity_property_map());
    boost::breadth_first_search(
        graph_, seed,
        boost::visitor(boost::default_bfs_visitor())
            .vertex_index_map(boost::identity_property_map())
            .color_map(color_map));
    num_visited_ = 0;
    for (size_t v = 0; v < n; ++v) {
      if (colors_[v] != boost::white_color) {
        visited_[v] = 1;
        ++num_visited_;
      }
    }
  }

  const std::vector<std::uint8_t> &visited() const { return visited_; }
  size_t num_visited() const { return num_visited_; }

protected:
  Graph graph_;
  std::vector<std::uint8_t> visited_;
  std::vector<boost::default_color_type> colors_;
  size_t num_visited_{0};
};

} // namespace grid
} // namespace naex
