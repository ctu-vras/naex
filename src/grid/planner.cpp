#include "naex/grid/planner.h"

#include "naex/clouds.h"
#include "naex/grid/conversions.h"
#include "naex/grid/path.h"
#include "naex/timer.h"
#include "naex/transforms.h"
#include <algorithm>
#include <chrono>
#include <cmath>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <limits>
#include <memory>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <sstream>
#include <stdexcept>
#include <string>
#include <tf2/exceptions.hpp>
#include <tf2/time.hpp>
#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <utility>
#include <vector>

namespace naex {
namespace grid {

namespace {

/// Adds its lifetime to a PlanTimings field when it goes out of scope.
///
/// Used around the TF lookups of the request path: those are exactly the calls
/// that can block for the whole timeout and then throw, and a plain
/// "timer.seconds_elapsed() after the call" loses precisely that case -- the
/// wait that matters most -- from the perf line.
class ScopedTfTimer {
public:
  explicit ScopedTfTimer(double &sink) : sink_(sink) {}
  ScopedTfTimer(const ScopedTfTimer &) = delete;
  ScopedTfTimer &operator=(const ScopedTfTimer &) = delete;
  ~ScopedTfTimer() { sink_ += timer_.seconds_elapsed(); }

private:
  Timer timer_;
  double &sink_;
};

/**
 * The ad-hoc (sidelobes) costs are drawn around the *robot*, so they are only
 * applied when the request's start pose really is the robot pose: at most
 * kAdHocMaxStartOffset metres from the robot frame origin and within
 * 2 * acos(kAdHocMinStartOrientationW) ~ 51 deg of its orientation.
 * navigate-through-poses sends start poses that are neither.
 */
constexpr double kAdHocMaxStartOffset = 3.;
constexpr double kAdHocMinStartOrientationW = 0.9;

/// rviz-only marker for the selected frontier component: turquoise points
/// kFrontierMarkerScale metres wide.  The colour components are the 0-255
/// values of that turquoise rather than the 0-1 floats the message wants,
/// which rviz clamps; kept as they are so the marker keeps its look.
constexpr double kFrontierMarkerScale = 0.2;
constexpr double kFrontierMarkerColorR = 64.0;
constexpr double kFrontierMarkerColorG = 224.0;
constexpr double kFrontierMarkerColorB = 208.0;
constexpr double kFrontierMarkerAlpha = 1.0;

} // namespace

Planner::Planner(rclcpp::Node::SharedPtr nh) : nh_(nh) {
  // A NaN start is replaced with the robot position in plan(); a NaN goal
  // fails the request (there is no exploration mode).
  last_request_ = std::make_shared<nav_msgs::srv::GetPlan::Request>();
  last_request_->start.pose.position.x =
      std::numeric_limits<double>::quiet_NaN();
  last_request_->start.pose.position.y =
      std::numeric_limits<double>::quiet_NaN();
  last_request_->start.pose.position.z =
      std::numeric_limits<double>::quiet_NaN();
  last_request_->goal.pose.position.x =
      std::numeric_limits<double>::quiet_NaN();
  last_request_->goal.pose.position.y =
      std::numeric_limits<double>::quiet_NaN();
  last_request_->goal.pose.position.z =
      std::numeric_limits<double>::quiet_NaN();
  last_request_->tolerance = 2.0f;

  position_field_ =
      nh_->declare_parameter<std::string>("position_field", position_field_);
  cost_fields_ = nh_->declare_parameter<std::vector<std::string>>("cost_fields",
                                                                  cost_fields_);
  which_cloud_ = nh_->declare_parameter<std::vector<long int>>("which_cloud",
                                                               which_cloud_);
  cloud_weights_ = nh_->declare_parameter<std::vector<double>>("cloud_weights",
                                                               cloud_weights_);
  cloud_levels_ = nh_->declare_parameter<std::vector<long int>>("cloud_levels",
                                                                cloud_levels_);
  map_frame_ = nh_->declare_parameter<std::string>("map_frame", map_frame_);
  robot_frame_ =
      nh_->declare_parameter<std::string>("robot_frame", robot_frame_);
  // Cloud callbacks run on the only executor thread, so a long wait here
  // stalls the planning timer and the get_plan service; drop the frame
  // instead (P5).  It still has to cover one TF period: with a 10 Hz TF the
  // replayed drops were "extrapolation into the future" by 1-6 ms, i.e. a
  // transform that would have been there one period later.
  cloud_tf_timeout_ =
      nh_->declare_parameter<float>("cloud_tf_timeout", cloud_tf_timeout_);
  // The get_plan callback runs on the same single executor thread, so the
  // request path must not be able to park it for seconds either.  Every lookup
  // it makes is "latest available" (see plan()), so it can only ever wait when
  // TF is genuinely absent, and then failing fast is what the caller wants.
  request_tf_timeout_ =
      nh_->declare_parameter<float>("request_tf_timeout", request_tf_timeout_);

  max_cloud_age_ =
      nh_->declare_parameter<float>("max_cloud_age", max_cloud_age_);
  // P6a: crop of the input cloud around the sensor; <= 0 or NaN disables it.
  input_range_ = nh_->declare_parameter<float>("input_range", input_range_);
  // P6b: bound of the grid itself; 0 (the default) keeps the pre-P6
  // behaviour, i.e. an unbounded map that only ever grows.
  map_range_ = nh_->declare_parameter<float>("map_range", map_range_);
  evict_period_ = nh_->declare_parameter<float>("evict_period", evict_period_);

  float cell_size = nh_->declare_parameter<float>("cell_size", 1.0f);
  float forget_factor = nh_->declare_parameter<float>("forget_factor", 1.0f);

  // 4 or 8
  neighborhood_ = nh_->declare_parameter<int>("neighborhood", neighborhood_);

  int num_input_clouds = nh_->declare_parameter<int>("num_input_clouds", 1);
  num_input_clouds = std::max(1, num_input_clouds);
  int queue_size = nh_->declare_parameter<int>("input_queue_size", 2);
  queue_size = std::max(1, queue_size);
  sensor_data_qos_ =
      nh_->declare_parameter<bool>("sensor_data_qos", sensor_data_qos_);

  // Ad-hoc cost parameters.  Declared before check_input_parameters() because
  // it rejects a cost field mapped onto the ad-hoc layer (P3's dirty-list
  // clear assumes nothing else writes that layer).
  adhoc_costs_ = nh_->declare_parameter("adhoc_costs", adhoc_costs_);
  adhoc_layer_ = nh_->declare_parameter("adhoc_layer", adhoc_layer_);

  check_input_parameters(num_input_clouds);

  // Defaults: no cost bound and unit cost per input cloud.  Sizing the
  // vectors here (instead of reserving) used to prepend zeros, making every
  // nonzero-cost cell untraversable by default (B2).
  std::vector<float> max_costs;
  std::vector<float> default_costs;
  max_costs.reserve(static_cast<size_t>(num_input_clouds));
  default_costs.reserve(static_cast<size_t>(num_input_clouds));
  for (int i = 0; i < num_input_clouds; ++i) {
    max_costs.push_back(std::numeric_limits<float>::quiet_NaN());
    default_costs.push_back(1.f);
  }
  // Absolute per-layer bounds; NaN (the default) means "layer not bounded".
  max_costs_ =
      nh_->declare_parameter<std::vector<float>>("max_costs", max_costs);
  // Bounds relative to cloud_weights: max_costs_relative[i] is multiplied by
  // cloud_weights[i], because the cost stored in the grid is the weighted
  // one.  A finite entry overrides max_costs for that layer; this is the
  // parameter the robot sets, and the one the recovery callback below
  // changes at runtime.
  std::vector<float> max_costs_relative(max_costs);
  max_costs_relative_ = nh_->declare_parameter<std::vector<float>>(
      "max_costs_relative", max_costs_relative);
  update_max_costs_absolute(true);

  default_costs_ = nh_->declare_parameter<std::vector<float>>("default_costs",
                                                              default_costs);
  grid_ = Grid(cell_size, forget_factor, default_costs_);

  planning_freq_ =
      nh_->declare_parameter<float>("planning_freq", planning_freq_);
  start_on_request_ =
      nh_->declare_parameter<bool>("start_on_request", start_on_request_);
  stop_on_goal_ = nh_->declare_parameter<bool>("stop_on_goal", stop_on_goal_);
  goal_reached_dist_ =
      nh_->declare_parameter<float>("goal_reached_dist", goal_reached_dist_);
  max_start_to_traversable_dist_ = nh_->declare_parameter<float>(
      "max_start_to_traversable_dist", max_start_to_traversable_dist_);

  // max_costs_relative is changed at runtime as a recovery behaviour.  The
  // per-vertex cost cache of GraphN is rebuilt by every search, so nothing
  // else has to be invalidated here.
  param_subscriber_ = std::make_shared<rclcpp::ParameterEventHandler>(nh_);
  cb_handle_ = param_subscriber_->add_parameter_callback(
      "max_costs_relative", [this](const rclcpp::Parameter &p) {
        try {
          const auto values = p.as_double_array();
          max_costs_relative_ =
              std::vector<float>(values.begin(), values.end());
        } catch (const rclcpp::ParameterTypeException &ex) {
          RCLCPP_ERROR(nh_->get_logger(),
                       "Ignoring max_costs_relative update: %s", ex.what());
          return;
        }
        update_max_costs_absolute(true);
      });

  // A* and frontier goal selection.
  use_astar_ = nh_->declare_parameter<bool>("use_astar", use_astar_);
  astar_max_range_ =
      nh_->declare_parameter<float>("astar_max_range", astar_max_range_);
  frontier_min_dist_ =
      nh_->declare_parameter<float>("frontier_min_dist", frontier_min_dist_);
  frontier_max_neighbors_ = nh_->declare_parameter<int>(
      "frontier_max_neighbors", frontier_max_neighbors_);
  max_relative_dist_to_goal_ = nh_->declare_parameter<float>(
      "max_relative_dist_to_goal", max_relative_dist_to_goal_);

  // Occupancy grid for the nav2 global costmap.
  publish_occupancy_grid_ = nh_->declare_parameter<bool>(
      "publish_occupancy_grid", publish_occupancy_grid_);
  occupancy_grid_w_ =
      nh_->declare_parameter<int>("occupancy_grid_w", occupancy_grid_w_);
  occupancy_grid_h_ =
      nh_->declare_parameter<int>("occupancy_grid_h", occupancy_grid_h_);

  // Sidelobes strategy parameters
  sidelobes_offset_distance_ = nh_->declare_parameter(
      "sidelobes_offset_distance", sidelobes_offset_distance_);
  sidelobes_radius_ =
      nh_->declare_parameter("sidelobes_radius", sidelobes_radius_);
  sidelobes_cost_ = nh_->declare_parameter("sidelobes_cost", sidelobes_cost_);
  sidelobes_angle_offsets_ = nh_->declare_parameter("sidelobes_angle_offsets",
                                                    sidelobes_angle_offsets_);

  tf_ = std::make_shared<tf2_ros::Buffer>(nh_->get_clock());
  tf_sub_ = std::make_shared<tf2_ros::TransformListener>(*tf_);

  map_pub_ = nh_->create_publisher<sensor_msgs::msg::PointCloud2>(
      "map", kPublisherQueueDepth);
  path_pub_ =
      nh_->create_publisher<nav_msgs::msg::Path>("path", kPublisherQueueDepth);
  planning_freq_pub_ = nh_->create_publisher<std_msgs::msg::Float32>(
      "planning_freq", kPublisherQueueDepth);
  occ_grid_pub_ = nh_->create_publisher<nav_msgs::msg::OccupancyGrid>(
      "map_occupancy_grid", rclcpp::SystemDefaultsQoS());
  // Debug only.
  frontiers_pub_ = nh_->create_publisher<visualization_msgs::msg::MarkerArray>(
      "map_frontiers", rclcpp::SystemDefaultsQoS());

  if (publish_occupancy_grid_) {
    RCLCPP_INFO(nh_->get_logger(),
                "Publishing the occupancy grid on 'map_occupancy_grid' "
                "(%d x %d cells, resolution %.3f); it feeds the nav2 global "
                "costmap.",
                occupancy_grid_w_, occupancy_grid_h_, grid_.cell_size());
    RCLCPP_INFO(nh_->get_logger(),
                "Sending one empty msg to 'map_occupancy_grid' to "
                "initialize the nav2 global_costmap.");
    geometry_msgs::msg::Pose p{};
    p.orientation.w = 1.0;
    create_and_publish_map_occupancy_grid(p);
  }

  // Reliable by default; flip sensor_data_qos to talk to a best-effort
  // publisher without rebuilding (B10).
  const rclcpp::QoS input_qos =
      sensor_data_qos_
          ? rclcpp::QoS(rclcpp::SensorDataQoS(
                rclcpp::KeepLast(static_cast<size_t>(queue_size))))
          : rclcpp::QoS(rclcpp::KeepLast(static_cast<size_t>(queue_size)));
  RCLCPP_INFO(nh_->get_logger(), "Input cloud QoS: %s, depth %d.",
              sensor_data_qos_ ? "sensor data (best effort)" : "reliable",
              queue_size);
  for (int i = 0; i < num_input_clouds; ++i) {
    std::stringstream ss;
    ss << "input_cloud_" << i;
    input_cloud_subs_.push_back(
        nh_->create_subscription<sensor_msgs::msg::PointCloud2>(
            ss.str(), input_qos,
            [this, i](const std::shared_ptr<const sensor_msgs::msg::PointCloud2>
                          &msg) { this->receive_cloud_safe(msg, i); }));
  }

  if (planning_freq_ > 0.f) {
    RCLCPP_INFO(nh_->get_logger(),
                "Re-plan automatically at %.1f Hz using the last request.",
                planning_freq_);

    if (start_on_request_) {
      RCLCPP_WARN(nh_->get_logger(),
                  "Automatic re-planning will start on request.");
    } else {
      start_planning();
    }
  } else {
    RCLCPP_INFO(nh_->get_logger(),
                "Don't re-plan automatically using the last request.");
  }

  if (stop_on_goal_) {
    RCLCPP_WARN(nh_->get_logger(),
                "Automatic re-planning will stop on reaching goal.");
  }

  get_plan_service_ = nh_->create_service<nav_msgs::srv::GetPlan>(
      "get_plan", [this](const nav_msgs::srv::GetPlan::Request::SharedPtr req,
                         nav_msgs::srv::GetPlan::Response::SharedPtr res) {
        this->request_plan(req, res);
      });
  clear_map_service_ = nh_->create_service<std_srvs::srv::Trigger>(
      "clear_plan_map",
      [this](const std_srvs::srv::Trigger::Request::SharedPtr req,
             std_srvs::srv::Trigger::Response::SharedPtr res) {
        this->clear_map(req, res);
      });

  // Configuration trap, found by profiling the P2 build (2026-09-12):
  // map_range evicts the cells that a wider ingestion crop re-creates from
  // the very next cloud.  At map_range 30 with the crop disabled that is
  // ~100 k cells created and thrown away per eviction cycle, which put 48 %
  // of the process in the cell hash and 19 % in malloc/free and cost twice
  // the CPU of the cropped run -- for a map that is no larger.  The declared
  // parameter is left untouched (`ros2 param get input_range` still reports
  // what was configured); only the crop actually applied is clamped, and the
  // warning below is emitted once, at start-up.
  effective_input_range_ = input_range_;
  if (map_range_ > 0.f && !(input_range_ > 0.f && input_range_ <= map_range_)) {
    effective_input_range_ = map_range_;
    RCLCPP_WARN(nh_->get_logger(),
                "input_range (%.1f) exceeds map_range (%.1f) or is disabled: "
                "ingesting beyond the map bound only re-creates the cells "
                "the next eviction drops (~100 k cells per cycle in the "
                "profiled 30 m configuration), so the input crop is clamped "
                "to map_range %.1f. Set input_range <= map_range to silence "
                "this.",
                input_range_, map_range_, map_range_);
  }

  if (effective_input_range_ > 0.f) {
    RCLCPP_INFO(nh_->get_logger(),
                "Input clouds are cropped to %.1f m around the sensor.",
                effective_input_range_);
  } else {
    RCLCPP_INFO(nh_->get_logger(),
                "Input clouds are not cropped (input_range %.1f).",
                input_range_);
  }
  if (map_range_ > 0.f) {
    RCLCPP_WARN(nh_->get_logger(),
                "Map is bounded (P6): cells farther than map_range %.1f m "
                "(%d cells) from the robot are evicted, at most every %.1f s "
                "or after %.1f m of travel. A goal outside the bound "
                "degrades to the nearest reachable cell.",
                map_range_, cell_radius(grid_, map_range_), evict_period_,
                kEvictMoveFraction * map_range_);
  } else {
    RCLCPP_INFO(nh_->get_logger(),
                "Map is unbounded (map_range %.1f): it grows for the whole "
                "mission.",
                map_range_);
  }

  RCLCPP_INFO(nh_->get_logger(), "Node initialized.");
}

void Planner::start_planning() {
  if (!(planning_freq_ > 0.f)) {
    RCLCPP_ERROR(nh_->get_logger(),
                 "Invalid planning frequency (%.3f) specified.",
                 planning_freq_);
    return;
  }
  planning_timer_ = nh_->create_wall_timer(
      std::chrono::duration<double>(1.0 / planning_freq_),
      [this]() { this->planning_timer(); });
  std_msgs::msg::Float32 msg;
  msg.data = planning_freq_;
  planning_freq_pub_->publish(msg);
  RCLCPP_WARN(nh_->get_logger(), "Planning started.");
}

nav_msgs::msg::Path Planner::empty_path() {
  nav_msgs::msg::Path msg;
  msg.header.frame_id = map_frame_;
  msg.header.stamp = nh_->get_clock()->now();
  return msg;
}

void Planner::stop_planning() {
  if (planning_timer_) {
    planning_timer_->cancel();
  }
  path_pub_->publish(empty_path());
  std_msgs::msg::Float32 msg;
  msg.data = 0;
  planning_freq_pub_->publish(msg);
  RCLCPP_WARN(nh_->get_logger(), "Planning stopped.");
}

void Planner::visualize_frontiers(const std::vector<Point2f> &points) {
  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = map_frame_;
  marker.header.stamp = nh_->get_clock()->now();
  marker.ns = "points";
  marker.id = 0;
  marker.type = visualization_msgs::msg::Marker::POINTS;
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.pose.orientation.w = 1.0;
  marker.scale.x = kFrontierMarkerScale;
  marker.scale.y = kFrontierMarkerScale;
  marker.color.r = kFrontierMarkerColorR;
  marker.color.g = kFrontierMarkerColorG;
  marker.color.b = kFrontierMarkerColorB;
  marker.color.a = kFrontierMarkerAlpha;
  marker.points.reserve(points.size());
  for (const auto &p : points) {
    if (!std::isfinite(p.x) || !std::isfinite(p.y)) {
      continue;
    }
    geometry_msgs::msg::Point gp;
    gp.x = p.x;
    gp.y = p.y;
    gp.z = 0.0;
    marker.points.push_back(gp);
  }
  auto array = std::make_unique<visualization_msgs::msg::MarkerArray>();
  array->markers.push_back(std::move(marker));
  frontiers_pub_->publish(std::move(array));
}

VertexId Planner::get_cheapest_frontier(const ShortestPaths &sp,
                                        VertexId v_start, const Vec3 &start,
                                        const Vec3 &goal, const Value min_dist,
                                        const int max_neighbors) {
  const std::vector<std::uint8_t> &visited = sp.visited();
  const uint8_t nb = static_cast<uint8_t>(neighborhood_);
  const VertexId n = static_cast<VertexId>(grid_.size());

  // 1. Collect the frontier cells into a grid of their own.
  //    "traversable" == expanded by the search AND at least min_dist away.
  Grid frontiers_grid(grid_.cell_size(), 1.f, default_costs_);
  std::vector<Cost> frontier_costs;      // indexed by frontiers_grid cell id
  std::vector<std::uint8_t> traversable; // same index

  for (VertexId v = 0; v < n; ++v) {
    const bool is_traversable =
        visited[v] && (to_vec3(grid_.point(v)) - start).norm() >= min_dist;
    const int degree = neighbor_degree(grid_, v, nb, [&](CellId t) {
      return within_range(grid_, t, v_start, astar_max_range_);
    });
    if (degree <= max_neighbors) {
      frontiers_grid.create_cell(frontiers_grid.point_to_cell(grid_.point(v)));
      frontier_costs.push_back(sp.f_value(v));
      traversable.push_back(is_traversable ? 1 : 0);
    }
  }

  // 2. Frontier components; keep the one that gets nearest to the goal.
  const VertexId nf = static_cast<VertexId>(frontiers_grid.size());
  std::vector<std::uint8_t> explored(nf, 0);
  Value best_nearest_dist = std::numeric_limits<Value>::infinity();
  std::vector<VertexId> best_component;
  // One graph for all seeds (it used to be rebuilt per seed).
  BFS bfs(frontiers_grid);

  for (VertexId seed = 0; seed < nf; ++seed) {
    if (explored[seed]) {
      continue;
    }
    bfs.run(seed);
    const std::vector<std::uint8_t> &component_visited = bfs.visited();

    std::vector<VertexId> component;
    bool has_traversable = false;
    for (VertexId fv = 0; fv < nf; ++fv) {
      if (!component_visited[fv]) {
        continue;
      }
      explored[fv] = 1;
      component.push_back(fv);
      if (traversable[fv]) {
        has_traversable = true;
      }
    }
    if (!has_traversable) {
      // No path from the start reaches this component.
      continue;
    }

    Value nearest_dist = std::numeric_limits<Value>::infinity();
    for (const VertexId fv : component) {
      if (!traversable[fv]) {
        continue;
      }
      const Value d = (to_vec3(frontiers_grid.point(fv)) - goal).norm();
      if (d < nearest_dist) {
        nearest_dist = d;
      }
    }
    if (nearest_dist < best_nearest_dist) {
      best_nearest_dist = nearest_dist;
      best_component = std::move(component);
    }
  }

  // 3. Cheapest traversable cell of the winning component.
  VertexId cheapest_frontier = INVALID_VERTEX_ID;
  Cost cheapest_cost = std::numeric_limits<Cost>::infinity();
  std::vector<Point2f> connected_frontier_points;
  connected_frontier_points.reserve(best_component.size());

  for (const VertexId fv : best_component) {
    connected_frontier_points.push_back(frontiers_grid.point(fv));
    if (!traversable[fv] ||
        frontier_costs[fv] >= ShortestPaths::kUnreachableCost) {
      continue;
    }
    if (frontier_costs[fv] < cheapest_cost) {
      const CellId v =
          grid_.find_cell(grid_.point_to_cell(frontiers_grid.point(fv)));
      if (v == INVALID_CELL_ID) {
        continue;
      }
      cheapest_cost = frontier_costs[fv];
      cheapest_frontier = v;
    }
  }

  if (!connected_frontier_points.empty()) {
    visualize_frontiers(connected_frontier_points);
  }

  if (cheapest_frontier != INVALID_VERTEX_ID) {
    RCLCPP_WARN(nh_->get_logger(), "Cheapest frontier: %s, cost: %f",
                format(to_vec3(grid_.point(cheapest_frontier))).c_str(),
                cheapest_cost);
  } else {
    RCLCPP_WARN(nh_->get_logger(), "No admissible frontier found!");
  }
  return cheapest_frontier;
}

std::pair<float, VertexId>
Planner::get_nearest_traversable_vertex(const Vec3 &p0) {
  Timer t_scan;
  const auto result = naex::grid::get_nearest_traversable_vertex(
      nh_->get_logger(), grid_, max_costs_absolute_, p0);
  plan_timings_.scan_traversable = t_scan.seconds_elapsed();
  return result;
}

void Planner::return_straight_line_plan(
    nav_msgs::srv::GetPlan::Response::SharedPtr res,
    const geometry_msgs::msg::PoseStamped &start,
    const geometry_msgs::msg::PoseStamped &goal) {
  nav_msgs::msg::Path local_plan;
  local_plan.header.frame_id = map_frame_;
  local_plan.header.stamp = nh_->get_clock()->now();
  local_plan.poses.push_back(start);
  local_plan.poses.push_back(goal);
  res->plan = std::move(local_plan);
  RCLCPP_INFO(nh_->get_logger(), "Planning straight line.");
}

VertexId Planner::select_start_vertex(const Vec3 &p0, bool &straight_line) {
  straight_line = false;
  const Cell start_cell = grid_.point_to_cell({p0.x(), p0.y()});
  const CellId v_start = grid_.find_cell(start_cell);

  if (v_start != INVALID_CELL_ID &&
      costs_in_bounds(grid_.costs(v_start), max_costs_absolute_)) {
    // Start cell exists and is traversable: plan from there.
    RCLCPP_INFO(nh_->get_logger(), "Planning from start position %s.",
                format(to_vec3(grid_.point(v_start))).c_str());
    return v_start;
  }
  const bool explored = v_start != INVALID_CELL_ID;
  if (explored) {
    RCLCPP_WARN(nh_->get_logger(), "Start position %s is not traversable.",
                format(to_vec3(grid_.point(v_start))).c_str());
  } else {
    RCLCPP_WARN(nh_->get_logger(), "Start position %s is unexplored.",
                format(p0).c_str());
  }

  const auto nearest = get_nearest_traversable_vertex(p0);
  const float best_dist = nearest.first;
  const VertexId best_v = nearest.second;
  if (best_v == INVALID_VERTEX_ID) {
    return INVALID_VERTEX_ID;
  }
  if (best_dist <= max_start_to_traversable_dist_) {
    RCLCPP_INFO(nh_->get_logger(),
                "Planning from nearest traversable point %s.",
                format(to_vec3(grid_.point(best_v))).c_str());
    return best_v;
  }
  if (explored) {
    RCLCPP_ERROR(nh_->get_logger(),
                 "Start point is further than max_start_to_traversable_dist "
                 "(%.3f > %.3f m) from the closest traversable point %s. "
                 "Failed to plan!",
                 best_dist, max_start_to_traversable_dist_,
                 format(to_vec3(grid_.point(best_v))).c_str());
    return INVALID_VERTEX_ID;
  }
  RCLCPP_WARN(nh_->get_logger(),
              "Start point is further than max_start_to_traversable_dist "
              "(%.3f > %.3f m) from the closest traversable point %s. "
              "Planning straight line to goal!",
              best_dist, max_start_to_traversable_dist_,
              format(to_vec3(grid_.point(best_v))).c_str());
  straight_line = true;
  return INVALID_VERTEX_ID;
}

VertexId Planner::select_astar_goal_vertex(const ShortestPaths &sp, VertexId v0,
                                           VertexId v_goal,
                                           bool is_goal_explored,
                                           const Vec3 &p0, const Vec3 &p1) {
  VertexId v1 = INVALID_VERTEX_ID;
  bool consider_frontier = false;
  const Value euclidean_dist_to_goal = (to_vec3(grid_.point(v0)) - p1).norm();
  const bool is_goal_in_obstacle =
      is_goal_explored &&
      !costs_in_bounds(grid_.costs(v_goal), max_costs_absolute_);

  if (sp.found_goal()) {
    // The goal is reachable.  If the cost-optimal route is much longer than
    // the straight line, a frontier may still be the better target: this is
    // the "drive the whole explored loop backwards" case.
    const Value start_to_goal_dist =
        sp.cheapest_path_euclidean_dist(grid_, v0, v_goal);
    if (start_to_goal_dist >
        max_relative_dist_to_goal_ * euclidean_dist_to_goal) {
      consider_frontier = true;
      RCLCPP_INFO(nh_->get_logger(),
                  "Considering frontier: path dist %f, max relative dist "
                  "%f, start-goal euclidean dist %f",
                  start_to_goal_dist, max_relative_dist_to_goal_,
                  euclidean_dist_to_goal);
    } else {
      RCLCPP_INFO(nh_->get_logger(),
                  "Goal is close. Choosing it as plan target.");
      v1 = v_goal;
    }
  } else if (is_goal_in_obstacle) {
    RCLCPP_INFO(nh_->get_logger(),
                "Goal is in obstacle. Choosing nearest reachable point as "
                "target.");
    Timer t_scan;
    v1 = nearest_cell(grid_, Point2f(p1.x(), p1.y()), [&sp](CellId v) {
      // Unreached cells carry FLT_MAX (or INF when cropped out); see
      // ShortestPaths::kUnreachableCost.
      return !(sp.f_value(v) > ShortestPaths::kUnreachableCost);
    });
    plan_timings_.scan_reachable = t_scan.seconds_elapsed();
    if (v1 != INVALID_CELL_ID) {
      RCLCPP_INFO(nh_->get_logger(),
                  "Plan target is nearest reachable point to goal: %s",
                  format(to_vec3(grid_.point(v1))).c_str());
    }
  } else {
    // Goal unexplored, hence unreachable: head for the cheapest frontier.
    Timer t_frontier;
    v1 = get_cheapest_frontier(sp, v0, p0, p1, frontier_min_dist_,
                               frontier_max_neighbors_);
    plan_timings_.frontier = t_frontier.seconds_elapsed();
  }

  if (consider_frontier) {
    Timer t_frontier;
    const VertexId v_frontier = get_cheapest_frontier(
        sp, v0, p0, p1, frontier_min_dist_, frontier_max_neighbors_);
    plan_timings_.frontier = t_frontier.seconds_elapsed();
    if (v_frontier != INVALID_VERTEX_ID) {
      const Value frontier_to_goal_dist =
          (to_vec3(grid_.point(v_frontier)) - to_vec3(grid_.point(v_goal)))
              .norm();
      if (frontier_to_goal_dist < euclidean_dist_to_goal) {
        RCLCPP_INFO(nh_->get_logger(),
                    "Frontier set as temporary goal with dist to goal %f",
                    frontier_to_goal_dist);
        v1 = v_frontier;
      } else {
        RCLCPP_INFO(nh_->get_logger(),
                    "Frontier doesn't get us closer to goal, using path to "
                    "goal");
        v1 = v_goal;
      }
    } else {
      // Handled as "no feasible path" by the caller.
      v1 = v_frontier;
    }
  }
  return v1;
}

bool Planner::plan(nav_msgs::srv::GetPlan::Request::SharedPtr req,
                   nav_msgs::srv::GetPlan::Response::SharedPtr res) {
  Timer t;
  Timer t_part;
  RCLCPP_INFO(nh_->get_logger(),
              "Planning request from %s to %s with tolerance %.1f m.",
              format(req->start.pose.position).c_str(),
              format(req->goal.pose.position).c_str(), req->tolerance);
  last_request_ = req;

  if (grid_.empty()) {
    RCLCPP_WARN(nh_->get_logger(), "Cannot plan in empty grid.");
    return false;
  }

  // Transform start and goal into map_frame_.
  geometry_msgs::msg::PoseStamped start = req->start;
  geometry_msgs::msg::PoseStamped goal = req->goal;
  if (start.header.frame_id != goal.header.frame_id) {
    RCLCPP_WARN(nh_->get_logger(),
                "Start and goal frame_id do not match ('%s' vs '%s'). "
                "Taking start frame as the one for the response.",
                start.header.frame_id.c_str(), goal.header.frame_id.c_str());
  }
  const std::string request_frame = req->start.header.frame_id;
  // Stamp with zero time (tf2::TimePointZero, "latest available") rather than
  // with nh_->now(): a now() stamp asks the buffer for a transform it cannot
  // have yet, so tf_->transform() blocks the single executor thread until TF
  // catches up -- one TF period in the good case, the whole timeout on a TF
  // dropout, and the frames of the request can be dynamic on the robot.  The
  // request carries no usable stamp of its own (the goal topic publishes a
  // zero stamp) and the robot-pose lookup below is already "latest", so
  // "latest" is both the cheapest and the consistent choice.
  const rclcpp::Time latest(0, 0, nh_->get_clock()->get_clock_type());
  start.header.stamp = latest;
  goal.header.stamp = latest;

  // Timed, so that a TF wait here shows up in the perf line instead of hiding
  // outside the instrumented section.
  {
    ScopedTfTimer frame_tf_timer(plan_timings_.start_tf);
    if (!request_frame.empty() && request_frame != map_frame_) {
      RCLCPP_WARN_THROTTLE(nh_->get_logger(), *nh_->get_clock(),
                           kFrameWarnThrottleMs,
                           "Start pose frame_id '%s' does not match map "
                           "frame_id '%s'. Attempting to transform.",
                           request_frame.c_str(), map_frame_.c_str());
      start = tf_->transform(start, map_frame_,
                             tf2::durationFromSec(request_tf_timeout_));
    }
    if (!goal.header.frame_id.empty() && goal.header.frame_id != map_frame_) {
      RCLCPP_WARN_THROTTLE(nh_->get_logger(), *nh_->get_clock(),
                           kFrameWarnThrottleMs,
                           "Goal pose frame_id '%s' does not match map "
                           "frame_id '%s'. Attempting to transform.",
                           goal.header.frame_id.c_str(), map_frame_.c_str());
      goal = tf_->transform(goal, map_frame_,
                            tf2::durationFromSec(request_tf_timeout_));
    }
  }

  // If the start is not valid, use the robot position instead.
  // TODO: this could be dangerous with navigate-through-poses in theory.
  if (!is_valid(start.pose.position)) {
    // tf2::TimePointZero ("latest available") avoids the clock-type
    // mismatch of rclcpp::Time(0) (system time) against a ROS-time buffer.
    geometry_msgs::msg::TransformStamped tf;
    {
      ScopedTfTimer start_tf_timer(plan_timings_.start_tf);
      tf = tf_->lookupTransform(map_frame_, robot_frame_, tf2::TimePointZero,
                                tf2::durationFromSec(request_tf_timeout_));
    }
    transform_to_pose(tf, start);
    start.header.frame_id = map_frame_;
  }

  // The grid is 2-D, so the z of both poses is planned on the ground plane;
  // only x/y ever reach the grid.
  start.pose.position.z = 0.f;
  goal.pose.position.z = 0.f;

  const Vec3 p0 = to_vec3(start.pose.position);
  Vec3 p1 = to_vec3(goal.pose.position);

  if (stop_on_goal_) {
    // Stop if close to the goal.
    const float dist_to_goal = (p1 - p0).norm();
    RCLCPP_INFO(nh_->get_logger(), "Distance to goal: %.3f m.", dist_to_goal);
    if (dist_to_goal <= goal_reached_dist_) {
      stop_planning();
      return false;
    }
  }

  // Figure out the starting cell of the search.  No Graph here (P2): a Graph
  // caches the per-vertex total cost and would be stale by the time the
  // ad-hoc layer below is rewritten, so the bounds check is the free
  // function.
  bool straight_line = false;
  const VertexId v0 = select_start_vertex(p0, straight_line);
  if (straight_line) {
    return_straight_line_plan(res, start, goal);
    return true;
  }
  if (v0 == INVALID_VERTEX_ID) {
    return false;
  }

  // Apply ad-hoc costs if enabled.
  if (!adhoc_costs_.empty()) {
    Timer t_adhoc;
    clear_ad_hoc_layer();

    // Make sure the start pose is close enough to the robot pose (it is not
    // with navigate-through-poses).  Not ideal, but it keeps the ad-hoc
    // banana from appearing where it should not.  This blocks for up to
    // request_tf_timeout_, so it is reported as TF time and not as ad-hoc
    // time.
    const double start_tf_before = plan_timings_.start_tf;
    geometry_msgs::msg::PoseStamped start_in_robot_frame;
    {
      ScopedTfTimer robot_tf_timer(plan_timings_.start_tf);
      start_in_robot_frame = tf_->transform(
          start, robot_frame_, tf2::durationFromSec(request_tf_timeout_));
    }
    const double tf_robot_seconds = plan_timings_.start_tf - start_tf_before;
    if (std::fabs(start_in_robot_frame.pose.position.x) >
            kAdHocMaxStartOffset ||
        std::fabs(start_in_robot_frame.pose.position.y) >
            kAdHocMaxStartOffset ||
        std::fabs(start_in_robot_frame.pose.orientation.w) <
            kAdHocMinStartOrientationW) {
      RCLCPP_WARN(nh_->get_logger(),
                  "Start pose in robot frame is not close to the origin: %s, "
                  "orientation w %f. Ad-hoc costs not applied!",
                  format(start_in_robot_frame.pose.position).c_str(),
                  start_in_robot_frame.pose.orientation.w);
    } else {
      // Extract robot yaw from start pose orientation.
      const auto &q = start.pose.orientation;
      const float robot_yaw = std::atan2(2.0f * (q.w * q.z + q.x * q.y),
                                         1.0f - 2.0f * (q.y * q.y + q.z * q.z));
      apply_ad_hoc_costs(p0, robot_yaw);
      RCLCPP_DEBUG(nh_->get_logger(),
                   "Applied ad-hoc costs at robot position %s, yaw %.3f rad.",
                   format(p0).c_str(), robot_yaw);
    }
    // Timing is reported by log_plan_summary().
    plan_timings_.adhoc = t_adhoc.seconds_elapsed() - tf_robot_seconds;
  }

  if (!is_valid(goal.pose.position)) {
    RCLCPP_WARN(nh_->get_logger(), "Goal not valid.");
    return false;
  }

  // Is the goal inside the mapped area?  Only then can A* stop on it.
  const CellId v_goal = grid_.find_cell(grid_.point_to_cell({p1.x(), p1.y()}));
  const bool is_goal_explored = v_goal != INVALID_CELL_ID;

  // Run the search.  Reused across requests (P2): the predecessor, path-cost
  // and f-value buffers keep their capacity, so a steady-state cycle
  // allocates nothing here.
  Timer t_search;
  ShortestPaths &sp = shortest_paths_;
  if (use_astar_) {
    sp.compute_astar(nh_->get_logger(), grid_, v0, p1, is_goal_explored,
                     astar_max_range_, static_cast<uint8_t>(neighborhood_),
                     max_costs_absolute_);
  } else {
    sp.compute(grid_, v0, static_cast<uint8_t>(neighborhood_),
               max_costs_absolute_);
  }
  plan_timings_.dijkstra = t_search.seconds_elapsed();
  // Unchanged on purpose: t_part still runs from the top of plan(), so this
  // line stays comparable with logs recorded before the instrumentation.
  RCLCPP_INFO(nh_->get_logger(), "%s (%lu pts): %.3f s.",
              use_astar_ ? "AStar" : "Dijkstra",
              static_cast<unsigned long>(grid_.size()),
              t_part.seconds_elapsed());

  Timer t_map;
  create_and_publish_map_cloud(sp);
  plan_timings_.map_cloud = t_map.seconds_elapsed();

  // Pick the cell the path ends in.
  t_part.reset();
  VertexId v1 = INVALID_VERTEX_ID;
  if (use_astar_) {
    v1 = select_astar_goal_vertex(sp, v0, v_goal, is_goal_explored, p0, p1);
  } else {
    // Path to the reachable cell closest to the goal.
    p1.z() = 0.f;
    Timer t_scan;
    // p1.z() is zeroed above, so the 2-D distance used by nearest_cell() is
    // the same value the 3-D norm used to produce.
    v1 = nearest_cell(grid_, Point2f(p1.x(), p1.y()), [&sp](CellId v) {
      return std::isfinite(sp.path_cost(v));
    });
    plan_timings_.scan_reachable = t_scan.seconds_elapsed();
  }

  if (v1 == INVALID_VERTEX_ID) {
    RCLCPP_ERROR(nh_->get_logger(),
                 "No feasible path towards %s was found (%.6f, %.3f s).",
                 format(p1).c_str(), t_part.seconds_elapsed(),
                 t.seconds_elapsed());
    return false;
  }

  RCLCPP_DEBUG(nh_->get_logger(), "v0 %u x %f y %f start", v0,
               grid_.point(v0).x, grid_.point(v0).y);
  RCLCPP_DEBUG(nh_->get_logger(), "v1 %u x %f y %f goal", v1, grid_.point(v1).x,
               grid_.point(v1).y);

  const auto path_vertices = trace_path_vertices(v0, v1, sp.predecessors());
  nav_msgs::msg::Path local_plan;
  local_plan.header.frame_id = map_frame_;
  local_plan.header.stamp = nh_->get_clock()->now();
  local_plan.poses.push_back(start);
  append_path(path_vertices, grid_, local_plan);
  // helhest 01/2026: add the actual goal point to the end of the path for
  // the goal checker down the path.
  local_plan.poses.push_back(goal);
  res->plan = std::move(local_plan);

  RCLCPP_INFO(nh_->get_logger(),
              "Path with %lu poses toward goal %s planned (%.3f s).",
              res->plan.poses.size(), format(p1).c_str(), t.seconds_elapsed());
  return true;
}

void Planner::create_and_publish_map_cloud(const ShortestPaths &sp) {
  if (map_pub_->get_subscription_count() == 0) {
    return;
  }
  auto cloud = std::make_unique<sensor_msgs::msg::PointCloud2>();
  cloud->header.frame_id = map_frame_;
  cloud->header.stamp = nh_->get_clock()->now();
  fill_map_cloud(*cloud, grid_, sp.path_costs(), sp.f_values());
  map_pub_->publish(std::move(cloud));
}

void Planner::fill_map_occupancy_grid(nav_msgs::msg::OccupancyGrid &occ_grid) {
  occ_grid.data.assign(static_cast<size_t>(occ_grid.info.width) *
                           occ_grid.info.height,
                       kOccupancyUnknown);
  const VertexId n = static_cast<VertexId>(grid_.size());
  for (VertexId v = 0; v < n; ++v) {
    const int data_idx = point_to_occupancy_grid_cell(grid_.point(v), occ_grid);
    if (data_idx == kOutsideOccupancyGrid) {
      continue;
    }
    occ_grid.data[static_cast<size_t>(data_idx)] =
        costs_in_bounds(grid_.costs(v), max_costs_absolute_)
            ? kOccupancyFree
            : kOccupancyBlocked;
  }
}

int Planner::point_to_occupancy_grid_cell(
    const Point2f &p, const nav_msgs::msg::OccupancyGrid &occ_grid) {
  const double dx = p.x - occ_grid.info.origin.position.x;
  const double dy = p.y - occ_grid.info.origin.position.y;
  const int64_t cell_x =
      static_cast<int64_t>(std::floor(dx / occ_grid.info.resolution));
  const int64_t cell_y =
      static_cast<int64_t>(std::floor(dy / occ_grid.info.resolution));
  if (cell_x < 0 || cell_x >= static_cast<int64_t>(occ_grid.info.width)) {
    return kOutsideOccupancyGrid;
  }
  if (cell_y < 0 || cell_y >= static_cast<int64_t>(occ_grid.info.height)) {
    return kOutsideOccupancyGrid;
  }
  return static_cast<int>(cell_x +
                          static_cast<int64_t>(occ_grid.info.width) * cell_y);
}

geometry_msgs::msg::Point Planner::get_occupancy_grid_origin(
    const geometry_msgs::msg::Pose &robot_pose,
    const nav_msgs::msg::OccupancyGrid &occ_grid) {
  geometry_msgs::msg::Point origin;
  origin.x = robot_pose.position.x - static_cast<double>(occ_grid.info.width) /
                                         2. * occ_grid.info.resolution;
  origin.y = robot_pose.position.y - static_cast<double>(occ_grid.info.height) /
                                         2. * occ_grid.info.resolution;
  return origin;
}

void Planner::create_and_publish_map_occupancy_grid(
    const geometry_msgs::msg::Pose &start) {
  auto occ_grid = std::make_unique<nav_msgs::msg::OccupancyGrid>();
  occ_grid->header.frame_id = map_frame_;
  const auto now = nh_->get_clock()->now();
  occ_grid->header.stamp = now;
  occ_grid->info.map_load_time = now;
  occ_grid->info.resolution = grid_.cell_size();
  occ_grid->info.width = static_cast<uint32_t>(std::max(0, occupancy_grid_w_));
  occ_grid->info.height = static_cast<uint32_t>(std::max(0, occupancy_grid_h_));
  // Assume the start pose of the request is the robot's current pose.
  occ_grid->info.origin.position = get_occupancy_grid_origin(start, *occ_grid);
  occ_grid->info.origin.orientation.w = 1.0;
  fill_map_occupancy_grid(*occ_grid);
  occ_grid_pub_->publish(std::move(occ_grid));
}

void Planner::log_plan_summary() const {
  RCLCPP_INFO_THROTTLE(
      nh_->get_logger(), *nh_->get_clock(), kPerfLogThrottleMs,
      "perf plan: cells=%lu map_range=%.1f astar=%d tf=%.4f adhoc=%.4f "
      "dijkstra=%.4f map_cloud=%.4f scan_trav=%.4f scan_reach=%.4f "
      "frontier=%.4f total=%.4f",
      static_cast<unsigned long>(plan_timings_.grid_size), map_range_,
      use_astar_ ? 1 : 0, plan_timings_.start_tf, plan_timings_.adhoc,
      plan_timings_.dijkstra, plan_timings_.map_cloud,
      plan_timings_.scan_traversable, plan_timings_.scan_reachable,
      plan_timings_.frontier, plan_timings_.total);
}

bool Planner::plan_safe(nav_msgs::srv::GetPlan::Request::SharedPtr req,
                        nav_msgs::srv::GetPlan::Response::SharedPtr res) {
  // Reset here rather than in plan() so that every exit path of plan(),
  // including the tf2 exception below, still produces a summary line.
  plan_timings_ = PlanTimings();
  Timer t_total;
  bool ok = false;
  try {
    ok = plan(req, res);
  } catch (const tf2::TransformException &ex) {
    RCLCPP_ERROR(nh_->get_logger(), "Transform failed: %s.", ex.what());
    ok = false;
  }
  plan_timings_.total = t_total.seconds_elapsed();
  plan_timings_.grid_size = grid_.size();
  log_plan_summary();
  return ok;
}

void Planner::request_plan(nav_msgs::srv::GetPlan::Request::SharedPtr req,
                           nav_msgs::srv::GetPlan::Response::SharedPtr res) {
  RCLCPP_INFO(nh_->get_logger(), "Planning request received.");
  if (start_on_request_) {
    start_planning();
  }
  if (!plan_safe(req, res)) {
    RCLCPP_WARN(nh_->get_logger(), "Planning failed; returning an empty plan.");
    res->plan = empty_path();
  }
}

void Planner::clear_map(std_srvs::srv::Trigger::Request::SharedPtr,
                        std_srvs::srv::Trigger::Response::SharedPtr res) {
  const size_t cells = grid_.size();
  grid_.clear();
  // Every CellId is invalidated, so the ad-hoc dirty list cannot be replayed.
  adhoc_dirty_.clear();
  std::stringstream ss;
  ss << "map cleared: " << cells << " cells";
  if (res) {
    res->success = true;
    res->message = ss.str();
  }
  RCLCPP_WARN(nh_->get_logger(), "%s.", ss.str().c_str());
}

void Planner::clear_ad_hoc_layer() {
  if (!is_valid_layer(adhoc_layer_)) {
    adhoc_dirty_.clear();
    return;
  }
  const Cost def = default_costs_[adhoc_layer_];
  for (const CellId v : adhoc_dirty_) {
    grid_.costs(v)[static_cast<size_t>(adhoc_layer_)] = def;
  }
  adhoc_dirty_.clear();
}

void Planner::apply_sidelobes_costs(const Vec3 &robot_pos, float robot_yaw) {
  if (adhoc_layer_ < 0 || static_cast<size_t>(adhoc_layer_) >= Costs::kSize) {
    return;
  }

  for (const auto &angle_offset : sidelobes_angle_offsets_) {
    const float angle_rad = radians(static_cast<float>(angle_offset));
    const Point2f center(robot_pos.x() + sidelobes_offset_distance_ *
                                             std::cos(robot_yaw + angle_rad),
                         robot_pos.y() + sidelobes_offset_distance_ *
                                             std::sin(robot_yaw + angle_rad));
    apply_disc_cost(grid_, adhoc_layer_, center, sidelobes_radius_,
                    sidelobes_cost_, &adhoc_dirty_);
  }
}

void Planner::apply_ad_hoc_costs(const Vec3 &robot_pos, float robot_yaw) {
  for (const auto &strategy : adhoc_costs_) {
    if (strategy == "sidelobes") {
      apply_sidelobes_costs(robot_pos, robot_yaw);
    }
  }
}

void Planner::planning_timer() {
  RCLCPP_INFO(nh_->get_logger(), "Planning timer callback.");
  Timer t;
  auto req = last_request_;
  auto res = std::make_shared<nav_msgs::srv::GetPlan::Response>();
  if (!plan_safe(req, res)) {
    return;
  }
  // Move the path out instead of copying it into the publisher (P4); the
  // response is local to this callback and is not used afterwards.
  auto path = std::make_unique<nav_msgs::msg::Path>(std::move(res->plan));
  const size_t num_poses = path->poses.size();
  path_pub_->publish(std::move(path));
  RCLCPP_INFO(nh_->get_logger(),
              "Planning robot %s path (%lu poses) in map %s: %.3f s.",
              robot_frame_.c_str(), num_poses, map_frame_.c_str(),
              t.seconds_elapsed());
}

void Planner::receive_cloud(
    const std::shared_ptr<const sensor_msgs::msg::PointCloud2> &input,
    int cloud_index) {
  const auto age = (nh_->get_clock()->now() - input->header.stamp).seconds();
  if (age > max_cloud_age_) {
    RCLCPP_INFO(nh_->get_logger(),
                "Skipping old input cloud from %s, age %.1f s > %.1f s.",
                input->header.frame_id.c_str(), age, max_cloud_age_);
    return;
  }

  Timer t_tf;
  geometry_msgs::msg::TransformStamped cloud_to_map;
  // Short timeout on purpose (P5): this runs on the only executor thread.
  cloud_to_map = tf_->lookupTransform(
      map_frame_, input->header.frame_id, input->header.stamp,
      rclcpp::Duration::from_seconds(cloud_tf_timeout_));
  const double tf_seconds = t_tf.seconds_elapsed();

  Eigen::Isometry3f transform(tf2::transformToEigen(cloud_to_map.transform));
  sensor_msgs::PointCloud2ConstIterator<float> x_it(*input, position_field_);

  // Sizes of which_cloud_, cloud_weights_ and cloud_levels_ are checked
  // against cost_fields_ in the constructor (B15).
  std::vector<int> levels;
  std::vector<float> weights;
  std::vector<sensor_msgs::PointCloud2ConstIterator<float>> cost_iters;

  for (size_t j = 0; j < cost_fields_.size(); ++j) {
    if (which_cloud_[j] == cloud_index) {
      levels.push_back(static_cast<int>(cloud_levels_[j]));
      weights.push_back(static_cast<float>(cloud_weights_[j]));
      cost_iters.push_back(sensor_msgs::PointCloud2ConstIterator<float>(
          *input, cost_fields_[j]));
    }
  }

  // Sensor origin in the map frame: the centre of the input_range crop and,
  // below, of the map_range eviction.  It is the sensor pose rather than the
  // robot pose, which is what the crop should be relative to anyway and
  // costs no extra TF lookup (P6).
  const auto &t = cloud_to_map.transform.translation;
  const Point2f origin(static_cast<float>(t.x), static_cast<float>(t.y));

  Timer t_loop;
  const size_t num_pts = num_points(*input);
  if (grid_.empty()) {
    // First cloud: almost every point becomes a cell, and it is the only
    // time the hash map rehashes from nothing to its final size (P10).
    // Later clouds add few cells, so no per-cloud reservation is made.
    grid_.reserve(num_pts);
  }
  size_t skipped = 0;
  // One-entry "last cell" cache.  A cloud is spatially coherent, so a run of
  // consecutive points usually falls into the same cell; resolving that cell
  // once turns the per-point (and, with several cost fields, per-field) hash
  // lookup into an int16 pair comparison.  Grid::cell_id() is 20 % of the
  // process at the 216 k-cell operating point, all of it under receive_cloud.
  // Kept valid for the whole loop because nothing here erases cells: the
  // eviction runs after it.
  Cell last_cell{};
  CellId last_id = INVALID_CELL_ID;
  for (size_t pt = 0; pt < num_pts; ++pt, ++x_it) {
    // Non-finite input must be rejected before the cast in point_to_cell()
    // (undefined behaviour, phantom cells), and the crop keeps the per-cloud
    // work bounded by input_range instead of by the size of the cloud (P6a).
    const Vec3 raw(x_it[0], x_it[1], x_it[2]);
    bool keep = is_valid(raw);
    Point2f p(0.f, 0.f);
    if (keep) {
      const Vec3 q = transform * raw;
      p = Point2f(q.x(), q.y());
      keep = accept_input_point(grid_, p, origin, effective_input_range_);
    }
    if (!keep) {
      ++skipped;
      // The cost iterators are advanced in lockstep with the position
      // iterator; skipping that would misalign every following point.
      for (size_t j = 0; j < levels.size(); ++j) {
        ++cost_iters[j];
      }
      continue;
    }
    // Resolved lazily, so a point whose every cost field is non-finite
    // still creates no cell, exactly as before the cache.
    bool resolved = false;
    for (size_t j = 0; j < levels.size(); ++j) {
      if (std::isfinite(cost_iters[j][0])) {
        if (!resolved) {
          const Cell c = grid_.point_to_cell(p);
          if (last_id == INVALID_CELL_ID || !(c == last_cell)) {
            last_cell = c;
            last_id = grid_.cell_id(c);
          }
          resolved = true;
        }
        grid_.update_cost_at(last_id, levels[j], weights[j] * cost_iters[j][0]);
      }
      ++cost_iters[j];
    }
  }
  const double loop_seconds = t_loop.seconds_elapsed();

  Timer t_evict;
  const bool evicted = maybe_evict_cells(origin);
  // One line per cloud; see log_plan_summary() for the planning-side line.
  RCLCPP_DEBUG(nh_->get_logger(),
               "perf cloud[%d]: pts=%lu skipped=%lu tf=%.4f points=%.4f "
               "evict=%.4f cells=%lu",
               cloud_index, static_cast<unsigned long>(num_pts),
               static_cast<unsigned long>(skipped), tf_seconds, loop_seconds,
               evicted ? t_evict.seconds_elapsed() : 0.0,
               static_cast<unsigned long>(grid_.size()));

  if (publish_occupancy_grid_) {
    // Centred on the robot, not on the sensor.  The lookup runs in the
    // cloud callback, so it uses the short cloud_tf_timeout_ (P5) and the
    // latest available transform (B11) rather than blocking the only
    // executor thread for a request-path timeout.
    geometry_msgs::msg::PoseStamped robot_pose;
    const auto robot_to_map =
        tf_->lookupTransform(map_frame_, robot_frame_, tf2::TimePointZero,
                             tf2::durationFromSec(cloud_tf_timeout_));
    transform_to_pose(robot_to_map, robot_pose);
    create_and_publish_map_occupancy_grid(robot_pose.pose);
  }
}

bool Planner::maybe_evict_cells(const Point2f &robot) {
  if (!(map_range_ > 0.f) || grid_.empty()) {
    return false;
  }
  if (!in_cell_range(grid_, robot)) {
    RCLCPP_WARN_THROTTLE(nh_->get_logger(), *nh_->get_clock(),
                         kEvictWarnThrottleMs,
                         "Not evicting: robot position (%.1f, %.1f) is not a "
                         "valid grid cell.",
                         robot.x, robot.y);
    return false;
  }
  const double now = nh_->get_clock()->now().seconds();
  const float dx = robot.x - last_evict_at_.x;
  const float dy = robot.y - last_evict_at_.y;
  const bool moved =
      !std::isfinite(last_evict_at_.x) || !std::isfinite(last_evict_at_.y) ||
      std::sqrt(dx * dx + dy * dy) >= kEvictMoveFraction * map_range_;
  // Not (now - last < period), so that a clock jump backwards evicts rather
  // than blocks eviction forever.
  const bool due = !(now - last_evict_time_ < evict_period_);
  const bool grown =
      static_cast<double>(grid_.size()) >
      kEvictSizeFactor * bounded_cell_count(cell_radius(grid_, map_range_));
  if (!moved && !due && !grown) {
    return false;
  }

  // The dirty list holds CellIds; replay it while they still mean something.
  clear_ad_hoc_layer();
  const Eviction ev = evict_outside_range(grid_, robot, map_range_);
  last_evict_at_ = robot;
  last_evict_time_ = now;
  if (ev.changed()) {
    RCLCPP_INFO_THROTTLE(
        nh_->get_logger(), *nh_->get_clock(), kPerfLogThrottleMs,
        "perf evict: map_range=%.1f center=(%.1f, %.1f) "
        "cells_before=%lu cells_after=%lu removed=%lu",
        map_range_, robot.x, robot.y, static_cast<unsigned long>(ev.before),
        static_cast<unsigned long>(ev.after),
        static_cast<unsigned long>(ev.removed));
  }
  return true;
}

void Planner::receive_cloud_safe(
    const std::shared_ptr<const sensor_msgs::msg::PointCloud2> &input,
    int cloud_index) {
  try {
    receive_cloud(input, cloud_index);
  } catch (const tf2::TransformException &ex) {
    // Expected whenever TF is late: the frame is dropped rather than waited
    // for (P5).  Throttled so a persistent TF outage stays visible without
    // flooding the log at the cloud rate.
    RCLCPP_WARN_THROTTLE(nh_->get_logger(), *nh_->get_clock(),
                         kTfDropWarnThrottleMs,
                         "Dropping input cloud from %s: no transform to %s "
                         "within %.3f s: %s.",
                         input->header.frame_id.c_str(), map_frame_.c_str(),
                         cloud_tf_timeout_, ex.what());
    return;
  } catch (const std::runtime_error &ex) {
    RCLCPP_ERROR(nh_->get_logger(), "Input cloud processing failed: %s",
                 ex.what());
  } catch (...) {
    RCLCPP_ERROR(nh_->get_logger(),
                 "Input cloud processing failed with an unknown exception.");
  }
}

void Planner::update_max_costs_absolute(bool log) {
  max_costs_absolute_ = max_costs_;
  for (size_t i = 0; i < Costs::kSize; ++i) {
    if (!std::isfinite(max_costs_relative_[i]) || i >= cloud_weights_.size()) {
      continue;
    }
    const float old_value = max_costs_absolute_[i];
    max_costs_absolute_[i] =
        max_costs_relative_[i] * static_cast<float>(cloud_weights_[i]);
    if (log) {
      RCLCPP_INFO(nh_->get_logger(),
                  "max_costs_absolute[%lu]: %f -> %f (relative %f, cloud "
                  "weight %f)",
                  static_cast<unsigned long>(i), old_value,
                  max_costs_absolute_[i], max_costs_relative_[i],
                  cloud_weights_[i]);
    }
  }
}

void Planner::check_input_parameters(int num_input_clouds) {
  const size_t n = cost_fields_.size();
  const auto check = [&](const char *name, size_t size) {
    if (size != n) {
      std::stringstream ss;
      ss << "Parameter '" << name << "' has " << size << " element(s) but '"
         << "cost_fields' has " << n << "; sizes must match.";
      throw std::runtime_error(ss.str());
    }
  };

  if (which_cloud_.empty() && n > 0) {
    RCLCPP_WARN(nh_->get_logger(),
                "Parameter 'which_cloud' not set; assuming input_cloud_0 "
                "for all %lu cost fields.",
                n);
    which_cloud_.assign(n, 0);
  }
  check("which_cloud", which_cloud_.size());

  if (cloud_weights_.empty() && n > 0) {
    RCLCPP_WARN(nh_->get_logger(),
                "Parameter 'cloud_weights' not set; using 1.0 for all %lu "
                "cost fields.",
                n);
    cloud_weights_.assign(n, 1.0);
  }
  check("cloud_weights", cloud_weights_.size());

  if (cloud_levels_.empty() && n > 0) {
    cloud_levels_.resize(n);
    for (size_t j = 0; j < n; ++j) {
      cloud_levels_[j] = static_cast<long int>(j);
    }
  }
  check("cloud_levels", cloud_levels_.size());

  for (size_t j = 0; j < n; ++j) {
    if (which_cloud_[j] < 0 || which_cloud_[j] >= num_input_clouds) {
      std::stringstream ss;
      ss << "which_cloud[" << j << "] = " << which_cloud_[j]
         << " is out of range [0, " << num_input_clouds << ").";
      throw std::runtime_error(ss.str());
    }
    if (cloud_levels_[j] < 0 ||
        static_cast<size_t>(cloud_levels_[j]) >= Costs::kSize) {
      std::stringstream ss;
      ss << "cloud_levels[" << j << "] = " << cloud_levels_[j]
         << " is out of range [0, " << Costs::kSize << ").";
      throw std::runtime_error(ss.str());
    }
    // P3: clear_ad_hoc_layer() restores only the cells the last apply touched,
    // which is equivalent to a full sweep only if no other writer touches
    // that layer.
    if (!adhoc_costs_.empty() &&
        cloud_levels_[j] == static_cast<long int>(adhoc_layer_)) {
      std::stringstream ss;
      ss << "cloud_levels[" << j << "] = " << cloud_levels_[j]
         << " collides with adhoc_layer; the ad-hoc layer must not be "
            "written by an input cost field.";
      throw std::runtime_error(ss.str());
    }
  }
}
} // namespace grid
} // namespace naex
