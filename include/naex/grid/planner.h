#pragma once

/**
 * @file
 * Declaration of naex::grid::Planner; the member bodies are in
 * src/grid/planner.cpp.  The free helpers that used to live here moved to
 * naex/grid/conversions.h (format/to_vec3/is_valid) and naex/grid/path.h
 * (trace_path_vertices/append_path), so that mule_planner.h can use them
 * without pulling the whole planner in.
 */

#include "naex/grid/graph.h"
#include "naex/grid/grid.h"
#include "naex/grid/search.h"
#include "naex/types.h"
#include <cstddef>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <limits>
#include <memory>
#include <mutex>
#include <nav2_core/planner_exceptions.hpp>
#include <nav2_msgs/action/compute_path_to_pose.hpp>
#include <nav2_msgs/srv/clear_entire_costmap.hpp>
#include <nav2_util/robot_utils.hpp>
#include <nav2_util/simple_action_server.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/srv/get_plan.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <string>
#include <tf2_ros/buffer.hpp>
#include <tf2_ros/transform_listener.hpp>
#include <utility>
#include <vector>
#include <visualization_msgs/msg/marker_array.hpp>

namespace naex {
namespace grid {

/**
 * nav_msgs/OccupancyGrid cell values the planner writes.
 *
 * The message encodes occupancy as 0..100 plus -1 for "unknown".  127 is out
 * of that range on purpose and is what this planner has always published for a
 * cell whose costs are out of bounds; the nav2 global costmap reads anything
 * at or above its lethal threshold the same way.
 */
inline constexpr int8_t kOccupancyUnknown = -1;
/// Cell whose costs are within max_costs.
inline constexpr int8_t kOccupancyFree = 0;
/// Cell whose costs are out of bounds; see kOccupancyUnknown.
inline constexpr int8_t kOccupancyBlocked = 127;
/**
 * Return of Planner::point_to_occupancy_grid_cell() for a point outside the
 * published grid.
 */
inline constexpr int kOutsideOccupancyGrid = -1;

/**
 * Depth of the planner's own publisher queues (map cloud, path, planning
 * frequency).  The input-cloud depth is the `input_queue_size` parameter.
 */
inline constexpr size_t kPublisherQueueDepth = 2;

/**
 * Throttle period (ms) of the once-per-cycle "perf" lines; planning_freq may
 * be higher than 1 Hz.
 */
inline constexpr int kPerfLogThrottleMs = 1000;
/**
 * Throttle period (ms) of the request-frame mismatch warnings, which repeat
 * at the request rate.
 */
inline constexpr int kFrameWarnThrottleMs = 1000;
/**
 * Throttle period (ms) of the dropped-cloud warning, which would otherwise
 * repeat at the cloud rate for the whole TF outage.
 */
inline constexpr int kTfDropWarnThrottleMs = 2000;
/// Throttle period (ms) of the "robot is not a valid cell" eviction warning.
inline constexpr int kEvictWarnThrottleMs = 5000;

/**
 * Per-cycle timing breakdown of Planner::plan().
 *
 * Filled by plan() and logged once per planning cycle by
 * Planner::log_plan_summary().  Purely observational: no field is read back by
 * the planner itself.
 */
struct PlanTimings {
  /// Cells in the grid at the end of the cycle (growth monitor).
  size_t grid_size{0};
  /**
   * TF on the get_plan path: the start/goal frame transforms, the robot
   * pose lookup (only when the request has no start) and the robot-frame
   * check of the ad-hoc layer.  Logged as tf= and bounded by
   * request_tf_timeout per lookup.
   */
  double start_tf{0.0};
  /// Ad-hoc (sidelobes) cost clear + apply.
  double adhoc{0.0};
  /// Graph construction + Dijkstra (ShortestPaths construction).
  double dijkstra{0.0};
  /// fill_map_cloud + publish on the rviz-only "map" topic.
  double map_cloud{0.0};
  /// O(N) scan for the nearest traversable cell (only when v0 is blocked).
  double scan_traversable{0.0};
  /// O(N) scan for the nearest reachable cell to the goal.
  double scan_reachable{0.0};
  /// Frontier detection and component search (A* only).
  double frontier{0.0};
  /// Wall time of the whole plan_safe() call, including the above.
  double total{0.0};
};

/**
 * Global planner on a 2-D grid.
 *
 * Each input cloud writes its own cost layer (up to Costs::kSize of them), so
 * several traversability sources -- e.g. geometric and semantic -- can be
 * combined into one grid; every layer is updated continuously from its cloud,
 * none of them is a static base layer.
 */
class Planner {
public:
  Planner(rclcpp::Node::SharedPtr nh);

  void start_planning();

  nav_msgs::msg::Path empty_path();

  void stop_planning();

  /// Visualize the connected frontier component the goal selection picked.
  void visualize_frontiers(const std::vector<Point2f> &points);

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
   * The neighbour count comes from Grid's flat neighbour table instead of
   * an out_edge walk over a boost::filtered_graph; the set of counted edges is
   * the same (an absent neighbour was the self-edge the old loop skipped) and
   * the range predicate is the one the A* filter used.
   */
  VertexId get_cheapest_frontier(const ShortestPaths &sp, VertexId v_start,
                                 const Vec3 &start, const Vec3 &goal,
                                 const Value min_dist, const int max_neighbors);

  /**
   * Nearest cell to @p p0 whose costs are in bounds, and its distance.
   *
   * The O(N) scan only runs when the robot cell is blocked or unexplored.
   */
  std::pair<float, VertexId> get_nearest_traversable_vertex(const Vec3 &p0);

  /**
   * Fall-back plan when the start is nowhere near anything traversable.
   * Always exactly two poses (start, goal), regardless of append_goal_pose_.
   */
  nav_msgs::msg::Path
  return_straight_line_plan(const geometry_msgs::msg::PoseStamped &start,
                            const geometry_msgs::msg::PoseStamped &goal);

  /**
   * Start cell of the search.
   *
   * @param straight_line set to true when the caller should answer with a
   *        straight line to the goal instead of planning.
   * @return the start cell, or INVALID_VERTEX_ID when planning must fail.
   */
  VertexId select_start_vertex(const Vec3 &p0, bool &straight_line);

  /// Goal cell for an A* search, or INVALID_VERTEX_ID when there is none.
  VertexId select_astar_goal_vertex(const ShortestPaths &sp, VertexId v0,
                                    VertexId v_goal, bool is_goal_explored,
                                    const Vec3 &p0, const Vec3 &p1);

  /**
   * Core planner: search from @p start to @p goal and fill @p path.
   *
   * Shared by the GetPlan service/timer path (plan_from_request(), which
   * additionally remembers the request for periodic re-planning) and the
   * compute_path_to_pose action (compute_plan(), which does not become the
   * periodic request). This method does not lock mtx_ itself -- every
   * caller is a top-level entry point that already holds it for the whole
   * call (see mtx_).
   */
  bool plan(const geometry_msgs::msg::PoseStamped &start,
            const geometry_msgs::msg::PoseStamped &goal,
            nav_msgs::msg::Path &path);

  /**
   * GetPlan request wrapper around plan(): remembers @p req in
   * last_request_ so planning_timer() can repeat it, then delegates.
   */
  bool plan_from_request(nav_msgs::srv::GetPlan::Request::SharedPtr req,
                        nav_msgs::srv::GetPlan::Response::SharedPtr res);

  /**
   * Publish the rviz-only "map" cloud, if anybody is listening.
   *
   * Building it costs 20 B per cell (4.3 MB at 216 k cells) every cycle, so it
   * is skipped when the topic has no subscriber.  Consequence: a late
   * joining subscriber (rviz, or a `ros2 bag record` started after the fact)
   * misses the cycles before it connected.
   */
  void create_and_publish_map_cloud(const ShortestPaths &sp);

  /**
   * Rasterize the cost grid into a nav_msgs/OccupancyGrid centred on @p start.
   *
   * Cells that are in bounds are free (0), cells that are not are 127; cells
   * the planner has never seen stay unknown (-1).  This is what the nav2
   * global costmap consumes.
   */
  void fill_map_occupancy_grid(nav_msgs::msg::OccupancyGrid &occ_grid);

  /// Index of @p p in @p occ_grid, or -1 when it falls outside.
  int point_to_occupancy_grid_cell(
      const Point2f &p, const nav_msgs::msg::OccupancyGrid &occ_grid);

  /// Bottom-left corner of an occupancy grid centred on @p robot_pose.
  geometry_msgs::msg::Point
  get_occupancy_grid_origin(const geometry_msgs::msg::Pose &robot_pose,
                            const nav_msgs::msg::OccupancyGrid &occ_grid);

  void
  create_and_publish_map_occupancy_grid(const geometry_msgs::msg::Pose &start);

  /**
   * Log the per-phase breakdown of the last planning cycle.
   *
   * One line per cycle, throttled to 1 Hz because planning_freq may be higher.
   * Format (single line):
   *   perf plan: cells=<N> map_range=<m> tf=<s> adhoc=<s> dijkstra=<s>
   *   map_cloud=<s> scan_trav=<s> scan_reach=<s> total=<s>
   *
   * tf is the sum of every TF wait on the request path (start/goal frame
   * transforms included), so a TF stall cannot hide outside the measured
   * section.
   *
   * map_range is the configured bound (0 = unbounded), repeated on every line
   * so that a bag says which regime "cells" was measured in.
   */
  void log_plan_summary() const;

  /**
   * Service/timer entry point: holds mtx_ for the whole call (see mtx_),
   * then delegates to plan_from_request().
   */
  bool plan_safe(nav_msgs::srv::GetPlan::Request::SharedPtr req,
                 nav_msgs::srv::GetPlan::Response::SharedPtr res);

  /**
   * Service callback.  nav_msgs/GetPlan has no success field, so a failed
   * plan is reported as a warning and an empty (but stamped) plan.
   */
  void request_plan(nav_msgs::srv::GetPlan::Request::SharedPtr req,
                    nav_msgs::srv::GetPlan::Response::SharedPtr res);

  /**
   * Action execute callback for compute_path_to_pose
   * (nav2_msgs::action::ComputePathToPose), run by the SimpleActionServer on
   * its own thread. Holds mtx_ for its whole duration (see mtx_): the start
   * pose lookup, the goal transform and the plan() call.
   *
   * Adapted from upstream's computePlan(): the start pose comes from
   * nav2_util::getCurrentPose(map_frame_, robot_frame_, request_tf_timeout_)
   * instead of a hard-coded 0.1 s, and the goal is transformed with
   * nav2_util::transformPoseInTargetFrame(..., request_tf_timeout_) instead
   * of a hard-coded 1.0 s -- both now share the same TF budget as the
   * GetPlan path. Does not touch last_request_: the action is a one-off
   * request, not the periodic re-planning source.
   */
  void compute_plan();

  /// Log an exception caught by compute_plan() and fill in result->error_msg.
  void exception_warning(const geometry_msgs::msg::PoseStamped &start,
                        const geometry_msgs::msg::PoseStamped &goal,
                        const std::string &planner_id,
                        const std::exception &ex, std::string &error_msg);

  /**
   * Drop the whole grid; returns the cell count it held before the clear.
   * Shared by both service callbacks below; holds mtx_ for the whole call
   * (see mtx_) since it is their only entry point into grid_.
   */
  size_t clear_map_impl();

  /**
   * Service callback on clear_plan_map: nav2_msgs/ClearEntireCostmap, the
   * name and type upstream and the robots use. Its response carries nothing
   * back, unlike clear_map() below.
   */
  void
  clear_map_costmap(nav2_msgs::srv::ClearEntireCostmap::Request::SharedPtr,
                    nav2_msgs::srv::ClearEntireCostmap::Response::SharedPtr);

  /**
   * Service callback on clear_plan_map_trigger: std_srvs/Trigger, so a
   * caller gets the cell count back in the response message.
   */
  void clear_map(std_srvs::srv::Trigger::Request::SharedPtr,
                 std_srvs::srv::Trigger::Response::SharedPtr res);

  /**
   * Service callback on clear_distant_plan_map: nav2_msgs/ClearEntireCostmap,
   * the same type as clear_plan_map, but it drops only the cells farther than
   * clear_distance_ from the robot instead of the whole grid.
   *
   * A manual, on-demand version of the map_range eviction, for the operator
   * who wants the stale map behind the robot gone without losing what is in
   * front of it. It goes through evict_outside_range() for exactly the reason
   * maybe_evict_cells() does: that is the only compaction that honours the
   * Eviction contract in grid.h (the neighbour table, the structural version,
   * the renumbering), so nothing that caches a CellId is left holding one that
   * now means a different cell.
   *
   * The TF lookup runs before mtx_ is taken: the lock must not be held across
   * a blocking TF wait (see mtx_).
   */
  void
  clear_distant_map(nav2_msgs::srv::ClearEntireCostmap::Request::SharedPtr,
                    nav2_msgs::srv::ClearEntireCostmap::Response::SharedPtr);

  /**
   * Restore the ad-hoc layer of every cell the last apply touched.
   *
   * Equivalent to fill_layer() over the whole grid only because nothing else
   * ever writes adhoc_layer_: cells created since the last apply already carry
   * default_costs_[adhoc_layer_] (Grid::create_cell), and a cloud cost field
   * mapped onto the ad-hoc layer is rejected by check_input_parameters().
   * Every operation that invalidates CellIds (clear_map(), the map_range
   * eviction) must drop the dirty list.
   */
  void clear_ad_hoc_layer();

  void apply_sidelobes_costs(const Vec3 &robot_pos, float robot_yaw);

  void apply_ad_hoc_costs(const Vec3 &robot_pos, float robot_yaw);

  void planning_timer();

  void receive_cloud(
      const std::shared_ptr<const sensor_msgs::msg::PointCloud2> &input,
      int cloud_index);

  /**
   * Drop the cells farther than map_range_ from @p robot, if it is time.
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
   * that point) so that no cell keeps a stale sidelobe cost forever.  The
   * neighbour table is remapped by Grid::erase_cells() itself, so no caller can
   * forget it.
   *
   * @return true if the compaction ran (whether or not it removed anything).
   */
  bool maybe_evict_cells(const Point2f &robot);

  /**
   * Cloud-ingestion entry point: holds mtx_ for the whole call to
   * receive_cloud() (grid writes, eviction and the occupancy-grid publish;
   * see mtx_), then handles the exceptions receive_cloud() may throw.
   */
  void receive_cloud_safe(
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
  void update_max_costs_absolute(bool log);

  /**
   * Validate the per-cost-field parameter vectors.
   *
   * A vector left empty is padded with a sensible default and a warning is
   * logged; a non-empty vector of the wrong size is a configuration error and
   * throws std::runtime_error.
   */
  void check_input_parameters(int num_input_clouds);

  using ComputePathAction = nav2_msgs::action::ComputePathToPose;

  rclcpp::Node::SharedPtr nh_;
  rclcpp::TimerBase::SharedPtr planning_timer_;

  std::unique_ptr<nav2_util::SimpleActionServer<ComputePathAction>>
      action_server_;
  /**
   * Guards grid_ and every other piece of planner/search state a plan reads
   * or writes, held for the whole duration of one plan or one cloud
   * ingestion at each top-level entry point (plan_safe(), compute_plan(),
   * receive_cloud_safe(), clear_map_impl()) -- never inside a helper they
   * call, so nothing here locks recursively (upstream locked inside
   * applySidelobesCosts(), which is called from plan() and would deadlock
   * under this scheme).
   *
   * Replaces copying the grid per plan (upstream's `current_grid = grid_`
   * under a mutex): ours can be tens of MB, so a copy per plan is not
   * affordable.
   *
   * ponytail: one lock for the whole node, so cloud ingestion waits while a
   * plan runs (milliseconds with A*, see astar_max_range_). Upgrade path if
   * that ceiling ever matters: a grid snapshot/RCU instead of a shared mutex.
   */
  std::mutex mtx_;

  // Runtime change of max_costs_relative (recovery behaviour).
  std::shared_ptr<rclcpp::ParameterEventHandler> param_subscriber_;
  std::shared_ptr<rclcpp::ParameterCallbackHandle> cb_handle_;

  /**
   * m; radius around the robot that the clear_distant_plan_map service keeps,
   * everything beyond it is dropped. Unlike map_range_ this is never applied
   * automatically; it only runs when the service is called. <= 0 or NaN makes
   * the service a no-op rather than a full wipe.
   */
  float clear_distance_{5.f};

  // A* and frontier goal selection
  bool use_astar_{false};
  /// m; cells farther than this from the start cell are not expanded.
  float astar_max_range_{50.};
  /// m; frontier cells closer than this to the robot are ignored.
  float frontier_min_dist_{3.};
  /**
   * Max number of existing in-range neighbours a cell may have and still
   * count as a frontier.
   */
  int frontier_max_neighbors_{5};
  /**
   * A cost-optimal route longer than this multiple of the crow-flies
   * distance makes the planner consider a frontier instead.
   */
  float max_relative_dist_to_goal_{2.0};
  /**
   * Weight of the euclidean distance to the goal in a frontier's score:
   * get_cheapest_frontier() scores a cell as path_cost(v) + (f_value(v) -
   * path_cost(v)) * frontier_dist_from_goal_cost_. 1.0 (the default)
   * reproduces f_value(v) exactly, i.e. unchanged behaviour; > 1.0 makes a
   * frontier further from the goal look more expensive than plain f_value
   * would.
   */
  float frontier_dist_from_goal_cost_{1.0};

  /**
   * Goal snapping (see snap_goal_cell() in grid.h): a goal not on a cell
   * whose goal_snap_level_ cost is at most goal_snap_max_cost_ moves to the
   * nearest such cell within goal_snap_radius_ (m). 0 (the default)
   * disables it.
   */
  float goal_snap_radius_{0.0};
  int goal_snap_level_{0};
  float goal_snap_max_cost_{0.0};

  // Transforms and frames
  std::shared_ptr<tf2_ros::Buffer> tf_{};
  std::shared_ptr<tf2_ros::TransformListener> tf_sub_;
  /**
   * Timeout of every TF lookup on the get_plan/plan path; short on purpose.
   * All of them are "latest available", so they can only wait when
   * TF is genuinely absent, and then failing fast beats parking the single
   * executor thread.
   */
  float request_tf_timeout_{0.5};
  /**
   * Timeout of the per-cloud lookup in receive_cloud(); short on purpose.
   * It has to cover one TF period (0.2 s = 2 periods of a 10 Hz TF), so
   * that a transform that is merely a few ms into the future does not drop
   * the frame, but still short enough that a TF dropout drops frames rather
   * than parking the single-threaded executor.
   */
  float cloud_tf_timeout_{0.2};
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

  // Subscribers
  std::vector<rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr>
      input_cloud_subs_;

  // Services
  rclcpp::Service<nav_msgs::srv::GetPlan>::SharedPtr get_plan_service_;
  nav_msgs::srv::GetPlan::Request::SharedPtr last_request_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr clear_map_service_;
  rclcpp::Service<nav2_msgs::srv::ClearEntireCostmap>::SharedPtr
      clear_map_costmap_service_;
  rclcpp::Service<nav2_msgs::srv::ClearEntireCostmap>::SharedPtr
      clear_distant_map_service_;

  // Input
  std::string position_field_{"x"};
  std::vector<std::string> cost_fields_;
  std::vector<long int> which_cloud_;
  std::vector<double> cloud_weights_;
  std::vector<long int> cloud_levels_;
  /**
   * Per-cost-field threshold (parallel to cost_fields_, checked in
   * check_input_parameters()): a point's cost field j only *creates* a cell
   * when its value is > min_cloud_values_[j]; an already-existing cell is
   * still updated regardless. -inf (the default) never gates creation, so
   * the default ingestion path does no extra work.
   */
  std::vector<double> min_cloud_values_;
  /**
   * Per-cost-field obstacle inflation radius in meters (parallel to
   * cost_fields_): an above-threshold point also stamps its cost onto every
   * other cell within this radius (see inflate_disc_cost() in grid.h). 0.0
   * (the default) disables inflation for that layer.
   */
  std::vector<double> inflation_radius_;
  float max_cloud_age_{5.0};
  /**
   * Radius (m) around the sensor outside which input points are discarded;
   * <= 0 or NaN disables the crop.  Bounds the per-cloud work, not the
   * grid: driving on keeps creating cells.
   */
  float input_range_{10.0};
  /**
   * The crop receive_cloud() actually applies: input_range_, clamped to
   * map_range_ when the map is bounded and the crop would reach past it (see
   * the constructor).  Kept separate so the declared parameter still reports
   * what the operator set.
   */
  float effective_input_range_{10.0};
  bool sensor_data_qos_{false};

  // Map bound.  Radius (m) around the robot outside which cells are
  // evicted; 0 (the default) means an unbounded map.
  // The bound is a square (Chebyshev in cells), so it caps the grid at
  // (2*ceil(map_range/cell_size) + 1)^2 cells.
  float map_range_{0.0};
  /**
   * Upper bound (s) on the interval between two evictions while the robot
   * stands still; <= 0 evicts once per ingested cloud.  Eviction also
   * triggers on movement and on growth, see kEvictMoveFraction /
   * kEvictSizeFactor.
   */
  float evict_period_{10.0};
  /**
   * Fraction of map_range_ the robot may travel between two evictions; it
   * bounds the overshoot of the cap to (1 + fraction) * map_range_.
   */
  static constexpr float kEvictMoveFraction = 0.25f;
  /**
   * How far the cell count may exceed what the bound retains before an
   * eviction is forced; this is what makes the cap on the grid size hold
   * however fast the map grows.
   */
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
  /**
   * What the search actually uses: max_costs_, with every finite
   * max_costs_relative_ entry overriding it as relative * cloud_weight.
   */
  Costs max_costs_absolute_;
  Costs default_costs_;
  /// Reused search buffers; see plan().  Holds no reference to the grid.
  ShortestPaths shortest_paths_;

  // Planning
  // Re-planning frequency, repeating the last request if positive.
  float planning_freq_{1.0};
  bool start_on_request_{true};
  bool stop_on_goal_{true};
  float goal_reached_dist_{std::numeric_limits<float>::quiet_NaN()};
  /**
   * If the start is farther than this from the nearest traversable cell we
   * plan a straight line to the goal instead (this should only happen with
   * navigate-through-poses, where the start need not be the robot).
   */
  float max_start_to_traversable_dist_{2.0};
  /**
   * If true (the default, unchanged helhest behaviour), the searched path
   * gets the requested goal pose appended as its last pose, so a downstream
   * goal checker sees the real goal even when the search itself stopped at
   * a frontier. husky/taros set this false. The straight-line fallback
   * (return_straight_line_plan()) always has exactly two poses regardless.
   */
  bool append_goal_pose_{true};

  // Ad-hoc costs
  std::vector<std::string> adhoc_costs_{};
  int adhoc_layer_{3};
  /**
   * Cells whose ad-hoc layer the last apply_ad_hoc_costs() wrote, so that
   * clear_ad_hoc_layer() restores those instead of sweeping the grid.
   */
  std::vector<CellId> adhoc_dirty_{};

  // Instrumentation (see PlanTimings); written by plan()/plan_safe() only.
  PlanTimings plan_timings_{};

  // Sidelobes strategy
  float sidelobes_offset_distance_{1.0f};
  float sidelobes_radius_{0.5f};
  float sidelobes_cost_{10.0f};
  std::vector<double> sidelobes_angle_offsets_{-90.0, 90.0};
};

} // namespace grid
} // namespace naex
