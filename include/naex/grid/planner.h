#pragma once

/**
 * @file
 * Declaration of naex::grid::Planner; the member bodies are in
 * src/grid/planner.cpp.  The free helpers that used to live here moved to
 * naex/grid/conversions.h (format/toVec3/isValid) and naex/grid/path.h
 * (tracePathVertices/appendPath), so that mule_planner.h can use them without
 * pulling the whole planner in.
 */

#include "naex/grid/graph.h"
#include "naex/grid/grid.h"
#include "naex/grid/search.h"
#include "naex/types.h"
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/srv/get_plan.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <tf2_ros/buffer.hpp>
#include <tf2_ros/transform_listener.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <cstddef>
#include <limits>
#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace naex {
namespace grid {

/**
 * @brief Per-cycle timing breakdown of Planner::plan().
 *
 * Filled by plan() and logged once per planning cycle by
 * Planner::logPlanSummary().  Purely observational: no field is read back by
 * the planner itself.
 */
struct PlanTimings {
  /// Cells in the grid at the end of the cycle (P6 growth monitor).
  size_t grid_size{0};
  /// TF lookup for the start pose (only when the request has no start).
  double start_tf{0.0};
  /// Ad-hoc (sidelobes) cost clear + apply.
  double adhoc{0.0};
  /// Graph construction + Dijkstra (ShortestPaths construction).
  double dijkstra{0.0};
  /// fillMapCloud + publish on the rviz-only "map" topic.
  double map_cloud{0.0};
  /// O(N) scan for the nearest traversable cell (only when v0 is blocked).
  double scan_traversable{0.0};
  /// O(N) scan for the nearest reachable cell to the goal.
  double scan_reachable{0.0};
  /// Frontier detection and component search (A* only).
  double frontier{0.0};
  /// Wall time of the whole planSafe() call, including the above.
  double total{0.0};
};

/**
 * @brief Global planner on 2D grid.
 *
 * It uses multi-level grid from multiple sources.
 * The first level may be constructed from a map and remain static.
 * The second level may be dynamic, updated from external traversability.
 */
class Planner {
public:
  Planner(rclcpp::Node::SharedPtr nh);

  void startPlanning();

  nav_msgs::msg::Path emptyPath();

  void stopPlanning();

  /// Visualize the connected frontier component the goal selection picked.
  void visualizeFrontiers(const std::vector<Point2f> &points);

  /**
   * Cheapest reachable frontier cell of the frontier component that comes
   * nearest to @p goal, or INVALID_VERTEX_ID.
   *
   * A frontier is a cell with at most frontier_max_neighbors existing
   * neighbours inside the A* range crop, i.e. a cell on the edge of what has
   * been mapped.  Frontiers are grouped into connected components on a grid of
   * their own; the component that gets nearest to the goal wins, and inside it
   * the cell with the lowest A* f value is returned.
   *
   * The neighbour count comes from Grid's flat neighbour table (P2) instead of
   * an out_edge walk over a boost::filtered_graph; the set of counted edges is
   * the same (an absent neighbour was the self-edge the old loop skipped) and
   * the range predicate is the one the A* filter used.
   */
  VertexId getCheapestFrontier(const ShortestPaths &sp, VertexId v_start,
                               const Vec3 &start, const Vec3 &goal,
                               const Value min_dist, const int max_neighbors);

  /**
   * Nearest cell to @p p0 whose costs are in bounds, and its distance.
   *
   * The O(N) scan only runs when the robot cell is blocked or unexplored (P9).
   */
  std::pair<float, VertexId> getNearestTraversableVertex(const Vec3 &p0);

  /// Fall-back plan when the start is nowhere near anything traversable.
  void returnStraightLinePlan(nav_msgs::srv::GetPlan::Response::SharedPtr res,
                              const geometry_msgs::msg::PoseStamped &start,
                              const geometry_msgs::msg::PoseStamped &goal);

  /**
   * Start cell of the search.
   *
   * @param straight_line set to true when the caller should answer with a
   *        straight line to the goal instead of planning.
   * @return the start cell, or INVALID_VERTEX_ID when planning must fail.
   */
  VertexId selectStartVertex(const Vec3 &p0, bool &straight_line);

  /// Goal cell for an A* search, or INVALID_VERTEX_ID when there is none.
  VertexId selectAstarGoalVertex(const ShortestPaths &sp, VertexId v0,
                                 VertexId v_goal, bool is_goal_explored,
                                 const Vec3 &p0, const Vec3 &p1);

  bool plan(nav_msgs::srv::GetPlan::Request::SharedPtr req,
            nav_msgs::srv::GetPlan::Response::SharedPtr res);

  void fillMapCloud(sensor_msgs::msg::PointCloud2 &cloud, const Grid &grid,
                    const std::vector<Cost> &path_costs,
                    const std::vector<Cost> &f_values);

  /**
   * Publish the rviz-only "map" cloud, if anybody is listening.
   *
   * Building it costs 20 B per cell (4.3 MB at 216 k cells) every cycle, so it
   * is skipped when the topic has no subscriber (P4).  Consequence: a late
   * joining subscriber (rviz, or a `ros2 bag record` started after the fact)
   * misses the cycles before it connected.
   */
  void createAndPublishMapCloud(const ShortestPaths &sp);

  /**
   * Rasterize the cost grid into a nav_msgs/OccupancyGrid centred on @p start.
   *
   * Cells that are in bounds are free (0), cells that are not are 127; cells
   * the planner has never seen stay unknown (-1).  This is what the nav2
   * global costmap consumes.
   */
  void fillMapOccupancyGrid(nav_msgs::msg::OccupancyGrid &occ_grid);

  /// Index of @p p in @p occ_grid, or -1 when it falls outside.
  int pointToOccupancyGridCell(const Point2f &p,
                               const nav_msgs::msg::OccupancyGrid &occ_grid);

  /// Bottom-left corner of an occupancy grid centred on @p robot_pose.
  geometry_msgs::msg::Point
  getOccupancyGridOrigin(const geometry_msgs::msg::Pose &robot_pose,
                         const nav_msgs::msg::OccupancyGrid &occ_grid);

  void createAndPublishMapOccupancyGrid(const geometry_msgs::msg::Pose &start);

  /**
   * Log the per-phase breakdown of the last planning cycle.
   *
   * One line per cycle, throttled to 1 Hz because planning_freq may be higher.
   * Format (single line):
   *   perf plan: cells=<N> map_range=<m> tf=<s> adhoc=<s> dijkstra=<s>
   *   map_cloud=<s> scan_trav=<s> scan_reach=<s> total=<s>
   *
   * map_range is the configured bound (0 = unbounded), repeated on every line
   * so that a bag says which regime "cells" was measured in (P6).
   */
  void logPlanSummary() const;

  bool planSafe(nav_msgs::srv::GetPlan::Request::SharedPtr req,
                nav_msgs::srv::GetPlan::Response::SharedPtr res);

  /// Service callback.  nav_msgs/GetPlan has no success field, so a failed
  /// plan is reported as a warning and an empty (but stamped) plan.
  void requestPlan(nav_msgs::srv::GetPlan::Request::SharedPtr req,
                   nav_msgs::srv::GetPlan::Response::SharedPtr res);

  /// Service callback: drop the whole grid.  std_srvs/Trigger, so a caller
  /// gets the cell count back; it used to be nav2_msgs/ClearEntireCostmap,
  /// whose empty response said nothing and dragged the whole nav2_msgs
  /// dependency in for one service type.
  void clearMap(std_srvs::srv::Trigger::Request::SharedPtr,
                std_srvs::srv::Trigger::Response::SharedPtr res);

  /**
   * Restore the ad-hoc layer of every cell the last apply touched.
   *
   * Equivalent to fillLayer() over the whole grid only because nothing else
   * ever writes adhoc_layer_: cells created since the last apply already carry
   * default_costs_[adhoc_layer_] (Grid::createCell), and a cloud cost field
   * mapped onto the ad-hoc layer is rejected by checkInputParameters().  Every
   * operation that invalidates CellIds (clearMap(), the P6 eviction) must
   * drop the dirty list.
   */
  void clearAdHocLayer();

  void applySidelobesCosts(const Vec3 &robot_pos, float robot_yaw);

  void applyAdHocCosts(const Vec3 &robot_pos, float robot_yaw);

  void planningTimer();

  void receiveCloud(
      const std::shared_ptr<const sensor_msgs::msg::PointCloud2> &input,
      int cloud_index);

  /**
   * Drop the cells farther than map_range_ from @p robot, if it is time (P6b).
   *
   * Called once per ingested cloud, but the O(N) compaction only runs when one
   * of three cheap triggers fires, so the amortised cost is negligible (a full
   * pass over 216 k cells is ~0.4 ms):
   *
   *  * the grid holds more than kEvictSizeFactor times the number of cells the
   *    bound retains -- this is the trigger that actually bounds N, whatever
   *    the map is growing from;
   *  * the robot has travelled more than kEvictMoveFraction * map_range_ since
   *    the last eviction, which bounds how far past map_range_ the retained
   *    region can extend;
   *  * evict_period_ has elapsed, which bounds a grid growing around a robot
   *    that stands still.
   *
   * Note that map_range_ bounds the map but not the ingestion: with
   * input_range_ larger than map_range_ (or disabled) every cloud would
   * re-create the cells the last eviction dropped.  That is correct but pure
   * churn, so the constructor clamps the crop it applies
   * (effective_input_range_) to map_range_ and warns once.
   *
   * Everything that caches a CellId must be invalidated here; see the
   * Eviction contract in grid.h.  Today that is only the ad-hoc dirty list,
   * which is restored *before* the compaction (its CellIds are still valid at
   * that point) so that no cell keeps a stale sidelobe cost forever.  P2's
   * neighbour table is remapped by Grid::eraseCells() itself, so no caller can
   * forget it.
   *
   * @return true if the compaction ran (whether or not it removed anything).
   */
  bool maybeEvictCells(const Point2f &robot);

  void receiveCloudSafe(
      const std::shared_ptr<const sensor_msgs::msg::PointCloud2> &input,
      int cloud_index);

protected:
  /**
   * Recompute max_costs_absolute_ from max_costs_ and max_costs_relative_.
   *
   * A finite max_costs_relative_[i] overrides max_costs_[i] with
   * max_costs_relative_[i] * cloud_weights_[i]; the grid stores the weighted
   * cost, so a bound expressed in the units of the input cloud has to be
   * scaled the same way.  Layers without a cloud weight (there are always
   * Costs::kSize layers but usually fewer cost fields) keep max_costs_.
   *
   * Nothing else has to be invalidated when this changes at runtime: GraphN
   * builds its per-vertex in-bounds cache inside every search.
   */
  void updateMaxCostsAbsolute(bool log);

  /**
   * Validate the per-cost-field parameter vectors.
   *
   * A vector left empty is padded with a sensible default and a warning is
   * logged; a non-empty vector of the wrong size is a configuration error and
   * throws std::runtime_error (B15).
   */
  void checkInputParameters(int num_input_clouds);

  rclcpp::Node::SharedPtr nh_;
  rclcpp::TimerBase::SharedPtr planning_timer_;

  // Runtime change of max_costs_relative (recovery behaviour).
  std::shared_ptr<rclcpp::ParameterEventHandler> param_subscriber_;
  std::shared_ptr<rclcpp::ParameterCallbackHandle> cb_handle_;

  // A* and frontier goal selection
  bool use_astar_{false};
  /// m; cells farther than this from the start cell are not expanded.
  float astar_max_range_{50.};
  /// m; frontier cells closer than this to the robot are ignored.
  float frontier_min_dist_{3.};
  /// Max number of existing in-range neighbours a cell may have and still
  /// count as a frontier.
  int frontier_max_neighbors_{5};
  /// A cost-optimal route longer than this multiple of the crow-flies
  /// distance makes the planner consider a frontier instead.
  float max_relative_dist_to_goal_{2.0};

  // Transforms and frames
  std::shared_ptr<tf2_ros::Buffer> tf_{};
  std::shared_ptr<tf2_ros::TransformListener> tf_sub_;
  /// Timeout of the once-per-cycle robot pose lookup in plan().
  float tf_timeout_{3.0};
  /// Timeout of the per-cloud lookup in receiveCloud(); short on purpose, see
  /// P5.  ~1 cloud period at 20 Hz, so a late transform drops one frame rather
  /// than parking the single-threaded executor.
  float cloud_tf_timeout_{0.05};
  std::string map_frame_{"map"};
  std::string robot_frame_{"base_footprint"};

  // Publishers
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr map_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr planning_freq_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr occ_grid_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr
      frontiers_pub_;
  bool publish_occupancy_grid_{true};
  int occupancy_grid_w_{500};
  int occupancy_grid_h_{500};
  float occupancy_grid_resolution_{0.4};
  float max_total_cost_{2.};

  // Subscribers
  std::vector<rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr>
      input_cloud_subs_;

  // Services
  rclcpp::Service<nav_msgs::srv::GetPlan>::SharedPtr get_plan_service_;
  nav_msgs::srv::GetPlan::Request::SharedPtr last_request_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr clear_map_service_;

  // Input
  std::string position_field_{"x"};
  std::vector<std::string> cost_fields_;
  std::vector<long int> which_cloud_;
  std::vector<double> cloud_weights_;
  std::vector<long int> cloud_levels_;
  float max_cloud_age_{5.0};
  /// Radius (m) around the sensor outside which input points are discarded;
  /// <= 0 or NaN disables the crop (P6a).  Bounds the per-cloud work, not the
  /// grid: driving on keeps creating cells.
  float input_range_{10.0};
  /// The crop receiveCloud() actually applies: input_range_, clamped to
  /// map_range_ when the map is bounded and the crop would reach past it (see
  /// the constructor).  Kept separate so the declared parameter still reports
  /// what the operator set.
  float effective_input_range_{10.0};
  bool sensor_data_qos_{false};

  // Map bound (P6b).  Radius (m) around the robot outside which cells are
  // evicted; 0 (the default) means the pre-P6 behaviour, an unbounded map.
  // The bound is a square (Chebyshev in cells), so it caps the grid at
  // (2*ceil(map_range/cell_size) + 1)^2 cells.
  float map_range_{0.0};
  /// Upper bound (s) on the interval between two evictions while the robot
  /// stands still; <= 0 evicts once per ingested cloud.  Eviction also
  /// triggers on movement and on growth, see kEvictMoveFraction /
  /// kEvictSizeFactor.
  float evict_period_{10.0};
  /// Fraction of map_range_ the robot may travel between two evictions; it
  /// bounds the overshoot of the cap to (1 + fraction) * map_range_.
  static constexpr float kEvictMoveFraction = 0.25f;
  /// How far the cell count may exceed what the bound retains before an
  /// eviction is forced; this is what makes the cap on the grid size hold
  /// however fast the map grows.
  static constexpr double kEvictSizeFactor = 1.5;
  /// Centre and time of the last eviction; NaN/0 until the first one.
  Point2f last_evict_at_{};
  double last_evict_time_{0.0};

  // Grid
  Grid grid_{};

  // Graph
  int neighborhood_{8};
  /// Absolute per-layer bounds from the 'max_costs' parameter.
  Costs max_costs_;
  /// Per-layer bounds relative to cloud_weights ('max_costs_relative').
  Costs max_costs_relative_;
  /// What the search actually uses: max_costs_, with every finite
  /// max_costs_relative_ entry overriding it as relative * cloud_weight.
  Costs max_costs_absolute_;
  Costs default_costs_;
  /// Reused search buffers (P2); see plan().  Holds no reference to the grid.
  ShortestPaths shortest_paths_;

  // Planning
  // Re-planning frequency, repeating the last request if positive.
  float planning_freq_{1.0};
  bool start_on_request_{true};
  bool stop_on_goal_{true};
  float goal_reached_dist_{std::numeric_limits<float>::quiet_NaN()};
  int mode_{2};
  /// If the start is farther than this from the nearest traversable cell we
  /// plan a straight line to the goal instead (this should only happen with
  /// navigate-through-poses, where the start need not be the robot).
  float max_start_to_traversable_dist_{2.0};

  // Ad-hoc costs
  std::vector<std::string> adhoc_costs_{};
  int adhoc_layer_{3};
  /// Cells whose ad-hoc layer the last applyAdHocCosts() wrote, so that
  /// clearAdHocLayer() restores those instead of sweeping the grid (P3).
  std::vector<CellId> adhoc_dirty_{};

  // Instrumentation (see PlanTimings); written by plan()/planSafe() only.
  PlanTimings plan_timings_{};

  // Sidelobes strategy
  float sidelobes_offset_distance_{1.0f};
  float sidelobes_radius_{0.5f};
  float sidelobes_cost_{10.0f};
  std::vector<double> sidelobes_angle_offsets_{-90.0, 90.0};
};

} // namespace grid
} // namespace naex
