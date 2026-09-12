#pragma once

#include "naex/clouds.h"
#include "naex/grid/graph.h"
#include "naex/grid/grid.h"
#include "naex/iterators.h"
#include "naex/grid/search.h"
#include "naex/timer.h"
#include "naex/transforms.h"
#include "naex/types.h"
#include <functional>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav2_msgs/srv/clear_entire_costmap.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/srv/get_plan.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <std_msgs/msg/float32.hpp>
#include <tf2/exceptions.hpp>
#include <tf2/time.hpp>
#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_ros/buffer.hpp>
#include <tf2_ros/transform_listener.hpp>
#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <vector>

namespace naex {
namespace grid {

template <typename T> inline std::string format(T x, T y, T z) {
  std::stringstream s;
  s << "(" << x << ", " << y << ", " << z << ")";
  return s.str();
}
inline std::string format(const geometry_msgs::msg::Vector3 &v) {
  return format(v.x, v.y, v.z);
}
inline std::string format(const geometry_msgs::msg::Point &v) {
  return format(v.x, v.y, v.z);
}
inline std::string format(const Vec3 &v) {
  return format(v.x(), v.y(), v.z());
}

inline Vec3 toVec3(const geometry_msgs::msg::Point &p) {
  return Vec3(p.x, p.y, p.z);
}
inline Vec3 toVec3(const geometry_msgs::msg::Vector3 &v) {
  return Vec3(v.x, v.y, v.z);
}
inline Vec3 toVec3(const Point2f &v) { return Vec3(v.x, v.y, 0.f); }

inline void tracePathVertices(VertexId v0, VertexId v1,
                       const std::vector<VertexId> &predecessor,
                       std::vector<VertexId> &path_vertices) {
  assert(predecessor[v0] == v0);
  VertexId v = v1;
  while (v != v0) {
    path_vertices.push_back(v);
    v = predecessor[v];
  }
  path_vertices.push_back(v);
  std::reverse(path_vertices.begin(), path_vertices.end());
}

inline std::vector<VertexId>
tracePathVertices(VertexId v0, VertexId v1,
                  const std::vector<VertexId> &predecessor) {
  std::vector<VertexId> path_vertices;
  tracePathVertices(v0, v1, predecessor, path_vertices);
  return path_vertices;
}

inline void appendPath(const std::vector<VertexId> &path_vertices,
                       const Grid &grid,
                nav_msgs::msg::Path &path) {
  if (path_vertices.empty()) {
    return;
  }
  path.poses.reserve(path.poses.size() + path_vertices.size());
  for (const auto &v : path_vertices) {
    geometry_msgs::msg::PoseStamped pose;
    pose.pose.position.x = grid.point(v).x;
    pose.pose.position.y = grid.point(v).y;
    pose.pose.position.z = 0.0;
    pose.pose.orientation.w = 1.;
    if (!path.poses.empty()) {
      Vec3 x(pose.pose.position.x - path.poses.back().pose.position.x,
             pose.pose.position.y - path.poses.back().pose.position.y,
             pose.pose.position.z - path.poses.back().pose.position.z);
      x.normalize();
      Vec3 z(0, 0, 1);
      Mat3 m;
      m.col(0) = x;
      m.col(1) = z.cross(x);
      m.col(2) = z;
      Quat q;
      q = m;
      pose.pose.orientation.x = q.x();
      pose.pose.orientation.y = q.y();
      pose.pose.orientation.z = q.z();
      pose.pose.orientation.w = q.w();
    }
    path.poses.push_back(pose);
  }
}

template <typename T> inline bool isValid(T x, T y, T z) {
  return std::isfinite(x) && std::isfinite(y) && std::isfinite(z);
}
inline bool isValid(const Vec3 &p) { return isValid(p.x(), p.y(), p.z()); }
inline bool isValid(const geometry_msgs::msg::Point &p) {
  return isValid(p.x, p.y, p.z);
}
inline bool isValid(const geometry_msgs::msg::Vector3 &p) {
  return isValid(p.x, p.y, p.z);
}

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
  Planner(rclcpp::Node::SharedPtr nh) : nh_(nh) {
    // Invalid position invokes exploration mode.
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
    cost_fields_ = nh_->declare_parameter<std::vector<std::string>>(
        "cost_fields", cost_fields_);
    which_cloud_ = nh_->declare_parameter<std::vector<long int>>("which_cloud",
                                                                 which_cloud_);
    cloud_weights_ = nh_->declare_parameter<std::vector<double>>(
        "cloud_weights", cloud_weights_);
    cloud_levels_ = nh_->declare_parameter<std::vector<long int>>(
        "cloud_levels", cloud_levels_);
    map_frame_ = nh_->declare_parameter<std::string>("map_frame", map_frame_);
    robot_frame_ =
        nh_->declare_parameter<std::string>("robot_frame", robot_frame_);
    tf_timeout_ = nh_->declare_parameter<float>("tf_timeout", tf_timeout_);
    // Cloud callbacks run on the only executor thread, so a long wait here
    // stalls the planning timer and the get_plan service; drop the frame
    // instead (P5).  Kept separate from tf_timeout_, which still governs the
    // once-per-cycle robot pose lookup in plan().
    cloud_tf_timeout_ =
        nh_->declare_parameter<float>("cloud_tf_timeout", cloud_tf_timeout_);

    max_cloud_age_ =
        nh_->declare_parameter<float>("max_cloud_age", max_cloud_age_);
    input_range_ = nh_->declare_parameter<float>("input_range", input_range_);

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

    // Ad-hoc cost parameters.  Declared before checkInputParameters() because
    // it rejects a cost field mapped onto the ad-hoc layer (P3's dirty-list
    // clear assumes nothing else writes that layer).
    adhoc_costs_ = nh_->declare_parameter("adhoc_costs", adhoc_costs_);
    adhoc_layer_ = nh_->declare_parameter("adhoc_layer", adhoc_layer_);

    checkInputParameters(num_input_clouds);

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
    max_costs_ =
        nh_->declare_parameter<std::vector<float>>("max_costs", max_costs);
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
    mode_ = nh_->declare_parameter<int>("mode", mode_);

    // Sidelobes strategy parameters
    sidelobes_offset_distance_ = nh_->declare_parameter(
        "sidelobes_offset_distance", sidelobes_offset_distance_);
    sidelobes_radius_ = nh_->declare_parameter(
        "sidelobes_radius", sidelobes_radius_);
    sidelobes_cost_ = nh_->declare_parameter(
        "sidelobes_cost", sidelobes_cost_);
    sidelobes_angle_offsets_ = nh_->declare_parameter(
        "sidelobes_angle_offsets", sidelobes_angle_offsets_);

    tf_ = std::make_shared<tf2_ros::Buffer>(nh_->get_clock());
    tf_sub_ = std::make_shared<tf2_ros::TransformListener>(*tf_);

    map_pub_ = nh_->create_publisher<sensor_msgs::msg::PointCloud2>("map", 2);
    path_pub_ = nh_->create_publisher<nav_msgs::msg::Path>("path", 2);
    planning_freq_pub_ =
        nh_->create_publisher<std_msgs::msg::Float32>("planning_freq", 2);

    // Reliable by default; flip sensor_data_qos to talk to a best-effort
    // publisher without rebuilding (B10).
    const rclcpp::QoS input_qos =
        sensor_data_qos_
            ? rclcpp::QoS(rclcpp::SensorDataQoS(rclcpp::KeepLast(
                  static_cast<size_t>(queue_size))))
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
              [this,
               i](const std::shared_ptr<const sensor_msgs::msg::PointCloud2>
                      &msg) { this->receiveCloudSafe(msg, i); }));
    }

    if (planning_freq_ > 0.f) {
      RCLCPP_INFO(nh_->get_logger(),
                  "Re-plan automatically at %.1f Hz using the last request.",
                  planning_freq_);

      if (start_on_request_) {
        RCLCPP_WARN(nh_->get_logger(),
                    "Automatic re-planning will start on request.");
      } else {
        startPlanning();
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
        "get_plan",
        [this](const nav_msgs::srv::GetPlan::Request::SharedPtr req,
               nav_msgs::srv::GetPlan::Response::SharedPtr res) {
          this->requestPlan(req, res);
        });
    clear_map_service_ =
        nh_->create_service<nav2_msgs::srv::ClearEntireCostmap>(
            "clear_plan_map",
            [this](
                const nav2_msgs::srv::ClearEntireCostmap::Request::SharedPtr req,
                nav2_msgs::srv::ClearEntireCostmap::Response::SharedPtr res) {
              this->clearMap(req, res);
            });

    RCLCPP_INFO(nh_->get_logger(), "Node initialized.");
  }

  void startPlanning() {
    if (!(planning_freq_ > 0.f)) {
      RCLCPP_ERROR(nh_->get_logger(),
                   "Invalid planning frequency (%.3f) specified.",
                   planning_freq_);
      return;
    }
    planning_timer_ = nh_->create_wall_timer(
        std::chrono::duration<double>(1.0 / planning_freq_),
        [this]() { this->planningTimer(); });
    std_msgs::msg::Float32 msg;
    msg.data = planning_freq_;
    planning_freq_pub_->publish(msg);
    RCLCPP_WARN(nh_->get_logger(), "Planning started.");
  }

  nav_msgs::msg::Path emptyPath() {
    nav_msgs::msg::Path msg;
    msg.header.frame_id = map_frame_;
    msg.header.stamp = nh_->get_clock()->now();
    return msg;
  }

  void stopPlanning() {
    if (planning_timer_) {
      planning_timer_->cancel();
    }
    path_pub_->publish(emptyPath());
    std_msgs::msg::Float32 msg;
    msg.data = 0;
    planning_freq_pub_->publish(msg);
    RCLCPP_WARN(nh_->get_logger(), "Planning stopped.");
  }

  bool plan(nav_msgs::srv::GetPlan::Request::SharedPtr req,
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

    geometry_msgs::msg::PoseStamped start = req->start;
    if (!isValid(start.pose.position)) {
      // tf2::TimePointZero ("latest available") avoids the clock-type
      // mismatch of rclcpp::Time(0) (system time) against a ROS-time buffer.
      Timer t_tf;
      const auto tf = tf_->lookupTransform(map_frame_, robot_frame_,
                                           tf2::TimePointZero,
                                           tf2::durationFromSec(tf_timeout_));
      plan_timings_.start_tf = t_tf.seconds_elapsed();
      transform_to_pose(tf, start);
    }

    if (mode_ == 2) {
      start.pose.position.z = 0.f;
      req->goal.pose.position.z = 0.f;
    }

    Vec3 p0 = toVec3(start.pose.position);
    Vec3 p1 = toVec3(req->goal.pose.position);
    if (stop_on_goal_) {
      // Stop if close to the goal.
      const float dist_to_goal = (p1 - p0).norm();
      RCLCPP_INFO(nh_->get_logger(), "Distance to goal: %.3f m.", dist_to_goal);
      if (dist_to_goal <= goal_reached_dist_) {
        stopPlanning();
        return false;
      }
    }

    Graph graph(grid_, neighborhood_, max_costs_);
    VertexId v0 = grid_.cellId(grid_.pointToCell({p0.x(), p0.y()}));
    if (!graph.costsInBounds(grid_.costs(v0))) {
      RCLCPP_WARN(nh_->get_logger(), "Robot position %s is not traversable.",
                  format(toVec3(grid_.point(v0))).c_str());
      // Fall back to the nearest traversable cell.  This O(N) scan only runs
      // when the robot cell is actually blocked (P9).
      Timer t_scan;
      const VertexId n = static_cast<VertexId>(grid_.size());
      VertexId v_best = INVALID_VERTEX_ID;
      float best_dist = std::numeric_limits<float>::infinity();
      for (VertexId v = 0; v < n; ++v) {
        if (!graph.costsInBounds(grid_.costs(v))) {
          continue;
        }
        const Value dist = (toVec3(grid_.point(v)) - p0).norm();
        if (dist < best_dist) {
          v_best = v;
          best_dist = dist;
        }
      }
      plan_timings_.scan_traversable = t_scan.seconds_elapsed();
      if (v_best == INVALID_VERTEX_ID) {
        RCLCPP_WARN(nh_->get_logger(),
                    "No traversable cell in the grid; planning from the "
                    "(untraversable) robot cell.");
      } else {
        v0 = v_best;
        RCLCPP_INFO(nh_->get_logger(),
                    "Closest traversable point to start: %s (%.3f).",
                    format(toVec3(grid_.point(v0))).c_str(), best_dist);
      }
    }

    // Apply ad-hoc costs if enabled
    if (!adhoc_costs_.empty()) {
      Timer t_adhoc;
      clearAdHocLayer();
      
      // Extract robot yaw from start pose orientation
      auto &q = start.pose.orientation;
      float robot_yaw = atan2(2.0f * (q.w * q.z + q.x * q.y),
                              1.0f - 2.0f * (q.y * q.y + q.z * q.z));
      
      applyAdHocCosts(p0, robot_yaw);
      // Timing is reported by logPlanSummary(); keep only the pose here.
      plan_timings_.adhoc = t_adhoc.seconds_elapsed();
      RCLCPP_DEBUG(nh_->get_logger(),
                   "Applied ad-hoc costs at robot position %s, yaw %.3f rad.",
                   format(p0).c_str(), robot_yaw);
    }

    Timer t_dijkstra;
    ShortestPaths sp(grid_, v0, neighborhood_, max_costs_);
    plan_timings_.dijkstra = t_dijkstra.seconds_elapsed();
    // Unchanged on purpose: t_part still runs from the top of plan(), so this
    // line stays comparable with logs recorded before the instrumentation.
    RCLCPP_INFO(nh_->get_logger(), "Dijkstra (%lu pts): %.3f s.", grid_.size(),
                t_part.seconds_elapsed());
    Timer t_map;
    createAndPublishMapCloud(sp);
    plan_timings_.map_cloud = t_map.seconds_elapsed();

    // If planning for a given goal, return path to the closest reachable
    // point from the goal.
    t_part.reset();
    if (isValid(req->goal.pose.position)) {
      Vec3 p1 = toVec3(req->goal.pose.position);
      p1.z() = 0.f;

      Timer t_scan;
      // p1.z() is zeroed above, so the 2-D distance used by nearestCell() is
      // the same value the 3-D norm used to produce.
      const VertexId v1 =
          nearestCell(grid_, Point2f(p1.x(), p1.y()), [&sp](CellId v) {
            return std::isfinite(sp.pathCost(v));
          });
      plan_timings_.scan_reachable = t_scan.seconds_elapsed();
      if (v1 == INVALID_CELL_ID) {
        RCLCPP_ERROR(nh_->get_logger(),
                     "No feasible path towards %s was found (%.6f, %.3f s).",
                     format(p1).c_str(), t_part.seconds_elapsed(),
                     t.seconds_elapsed());
        return false;
      }
      auto path_vertices = tracePathVertices(v0, v1, sp.predecessors());
      res->plan.header.frame_id = map_frame_;
      res->plan.header.stamp = nh_->get_clock()->now();
      res->plan.poses.push_back(start);
      appendPath(path_vertices, grid_, res->plan);
      // helhest 01/2026: add the actual goal point to the end of the path for goal checker down the path
      res->plan.poses.push_back(req->goal);

      RCLCPP_INFO(nh_->get_logger(),
                  "Path with %lu poses toward goal %s planned (%.3f s).",
                  res->plan.poses.size(), format(p1).c_str(),
                  t.seconds_elapsed());
      return true;
    }
    RCLCPP_WARN(nh_->get_logger(), "Goal not valid.");

    // TODO: Return random path in exploration mode.
    return false;
  }

  void fillMapCloud(sensor_msgs::msg::PointCloud2 &cloud, const Grid &grid,
                    const std::vector<Cost> &path_costs) {
    // TODO: Allow sending local map.
    append_field<float>("x", 1, cloud);
    append_field<float>("y", 1, cloud);
    append_field<float>("z", 1, cloud);
    append_field<float>("cost", 1, cloud);
    append_field<float>("path_cost", 1, cloud);
    const VertexId num_cells = static_cast<VertexId>(grid.size());
    resize_cloud(cloud, 1, num_cells);

    sensor_msgs::PointCloud2Iterator<float> x_it(cloud, "x");
    sensor_msgs::PointCloud2Iterator<float> cost_it(cloud, "cost");
    sensor_msgs::PointCloud2Iterator<float> path_cost_it(cloud, "path_cost");
    for (VertexId v = 0; v < num_cells; ++v, ++x_it, ++cost_it, ++path_cost_it) {
      const auto p = grid.point(v);
      x_it[0] = p.x;
      x_it[1] = p.y;
      x_it[2] = 0.f;
      cost_it[0] = grid.costs(v).total();
      path_cost_it[0] = path_costs[v];
    }
  }

  /**
   * Publish the rviz-only "map" cloud, if anybody is listening.
   *
   * Building it costs 20 B per cell (4.3 MB at 216 k cells) every cycle, so it
   * is skipped when the topic has no subscriber (P4).  Consequence: a late
   * joining subscriber (rviz, or a `ros2 bag record` started after the fact)
   * misses the cycles before it connected.
   */
  void createAndPublishMapCloud(const ShortestPaths &sp) {
    if (map_pub_->get_subscription_count() == 0) {
      return;
    }
    auto cloud = std::make_unique<sensor_msgs::msg::PointCloud2>();
    cloud->header.frame_id = map_frame_;
    cloud->header.stamp = nh_->get_clock()->now();
    fillMapCloud(*cloud, grid_, sp.pathCosts());
    map_pub_->publish(std::move(cloud));
  }

  /**
   * Log the per-phase breakdown of the last planning cycle.
   *
   * One line per cycle, throttled to 1 Hz because planning_freq may be higher.
   * Format (single line):
   *   perf plan: cells=<N> tf=<s> adhoc=<s> dijkstra=<s> map_cloud=<s>
   *   scan_trav=<s> scan_reach=<s> total=<s>
   */
  void logPlanSummary() const {
    RCLCPP_INFO_THROTTLE(
        nh_->get_logger(), *nh_->get_clock(), 1000,
        "perf plan: cells=%lu tf=%.4f adhoc=%.4f dijkstra=%.4f "
        "map_cloud=%.4f scan_trav=%.4f scan_reach=%.4f total=%.4f",
        static_cast<unsigned long>(plan_timings_.grid_size),
        plan_timings_.start_tf, plan_timings_.adhoc, plan_timings_.dijkstra,
        plan_timings_.map_cloud, plan_timings_.scan_traversable,
        plan_timings_.scan_reachable, plan_timings_.total);
  }

  bool planSafe(nav_msgs::srv::GetPlan::Request::SharedPtr req,
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
    logPlanSummary();
    return ok;
  }

  /// Service callback.  nav_msgs/GetPlan has no success field, so a failed
  /// plan is reported as a warning and an empty (but stamped) plan.
  void requestPlan(nav_msgs::srv::GetPlan::Request::SharedPtr req,
                   nav_msgs::srv::GetPlan::Response::SharedPtr res) {
    RCLCPP_INFO(nh_->get_logger(), "Planning request received.");
    if (start_on_request_) {
      startPlanning();
    }
    if (!planSafe(req, res)) {
      RCLCPP_WARN(nh_->get_logger(),
                  "Planning failed; returning an empty plan.");
      res->plan = emptyPath();
    }
  }

  void clearMap(nav2_msgs::srv::ClearEntireCostmap::Request::SharedPtr,
                nav2_msgs::srv::ClearEntireCostmap::Response::SharedPtr) {
    grid_.clear();
    // Every CellId is invalidated, so the ad-hoc dirty list cannot be replayed.
    adhoc_dirty_.clear();
    RCLCPP_WARN(nh_->get_logger(), "Map cleared.");
  }

  /**
   * Restore the ad-hoc layer of every cell the last apply touched.
   *
   * Equivalent to fillLayer() over the whole grid only because nothing else
   * ever writes adhoc_layer_: cells created since the last apply already carry
   * default_costs_[adhoc_layer_] (Grid::createCell), and a cloud cost field
   * mapped onto the ad-hoc layer is rejected by checkInputParameters().  Every
   * operation that invalidates CellIds (clearMap(), later P6 eviction) must
   * drop the dirty list.
   */
  void clearAdHocLayer() {
    if (!isValidLayer(adhoc_layer_)) {
      adhoc_dirty_.clear();
      return;
    }
    const Cost def = default_costs_[adhoc_layer_];
    for (const CellId v : adhoc_dirty_) {
      grid_.costs(v)[static_cast<size_t>(adhoc_layer_)] = def;
    }
    adhoc_dirty_.clear();
  }

  void applySidelobesCosts(const Vec3 &robot_pos, float robot_yaw) {
    if (adhoc_layer_ < 0 ||
        static_cast<size_t>(adhoc_layer_) >= Costs::kSize) {
      return;
    }

    for (const auto &angle_offset : sidelobes_angle_offsets_) {
      const float angle_rad = radians(static_cast<float>(angle_offset));
      const Point2f center(robot_pos.x() + sidelobes_offset_distance_ *
                                               std::cos(robot_yaw + angle_rad),
                           robot_pos.y() + sidelobes_offset_distance_ *
                                               std::sin(robot_yaw + angle_rad));
      applyDiscCost(grid_, adhoc_layer_, center, sidelobes_radius_,
                    sidelobes_cost_, &adhoc_dirty_);
    }
  }

  void applyAdHocCosts(const Vec3 &robot_pos, float robot_yaw) {
    for (const auto &strategy : adhoc_costs_) {
      if (strategy == "sidelobes") {
        applySidelobesCosts(robot_pos, robot_yaw);
      }
    }
  }

  void planningTimer() {
    RCLCPP_INFO(nh_->get_logger(), "Planning timer callback.");
    Timer t;
    auto req = last_request_;
    auto res = std::make_shared<nav_msgs::srv::GetPlan::Response>();
    if (!planSafe(req, res)) {
      return;
    }
    // Move the path out instead of copying it into the publisher (P4); the
    // response is local to this callback and is not used afterwards.
    auto path = std::make_unique<nav_msgs::msg::Path>(std::move(res->plan));
    const size_t num_poses = path->poses.size();
    path_pub_->publish(std::move(path));
    RCLCPP_INFO(nh_->get_logger(),
                "Planning robot %s path (%lu poses) in map %s: %.3f s.",
                robot_frame_.c_str(), num_poses,
                map_frame_.c_str(), t.seconds_elapsed());
  }

  void receiveCloud(
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

    Timer t_loop;
    const size_t num_pts = num_points(*input);
    for (size_t pt = 0; pt < num_pts; ++pt, ++x_it) {
      Vec3 p(x_it[0], x_it[1], x_it[2]);
      p = transform * p;
      for (size_t j = 0; j < levels.size(); ++j) {
        if (std::isfinite(cost_iters[j][0])) {
          grid_.updatePointCost({p.x(), p.y()}, levels[j],
                                weights[j] * cost_iters[j][0]);
        }
        ++cost_iters[j];
      }
    }
    // One line per cloud; see logPlanSummary() for the planning-side line.
    RCLCPP_DEBUG(nh_->get_logger(),
                 "perf cloud[%d]: pts=%lu tf=%.4f points=%.4f cells=%lu",
                 cloud_index, static_cast<unsigned long>(num_pts), tf_seconds,
                 t_loop.seconds_elapsed(),
                 static_cast<unsigned long>(grid_.size()));
  }

  void receiveCloudSafe(
      const std::shared_ptr<const sensor_msgs::msg::PointCloud2> &input,
      int cloud_index) {
    try {
      receiveCloud(input, cloud_index);
    } catch (const tf2::TransformException &ex) {
      // Expected whenever TF is late: the frame is dropped rather than waited
      // for (P5).  Throttled so a persistent TF outage stays visible without
      // flooding the log at the cloud rate.
      RCLCPP_WARN_THROTTLE(nh_->get_logger(), *nh_->get_clock(), 2000,
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

protected:
  /**
   * Validate the per-cost-field parameter vectors.
   *
   * A vector left empty is padded with a sensible default and a warning is
   * logged; a non-empty vector of the wrong size is a configuration error and
   * throws std::runtime_error (B15).
   */
  void checkInputParameters(int num_input_clouds) {
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
      // P3: clearAdHocLayer() restores only the cells the last apply touched,
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

  rclcpp::Node::SharedPtr nh_;
  rclcpp::TimerBase::SharedPtr planning_timer_;

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

  // Subscribers
  std::vector<rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr>
      input_cloud_subs_;

  // Services
  rclcpp::Service<nav_msgs::srv::GetPlan>::SharedPtr get_plan_service_;
  nav_msgs::srv::GetPlan::Request::SharedPtr last_request_;
  rclcpp::Service<nav2_msgs::srv::ClearEntireCostmap>::SharedPtr
      clear_map_service_;

  // Input
  std::string position_field_{"x"};
  std::vector<std::string> cost_fields_;
  std::vector<long int> which_cloud_;
  std::vector<double> cloud_weights_;
  std::vector<long int> cloud_levels_;
  float max_cloud_age_{5.0};
  // TODO(B8/P6): declared and logged but not used to crop the input yet.
  float input_range_{10.0};
  bool sensor_data_qos_{false};

  // Grid
  Grid grid_{};

  // Graph
  int neighborhood_{8};
  Costs max_costs_;
  Costs default_costs_;

  // Planning
  // Re-planning frequency, repeating the last request if positive.
  float planning_freq_{1.0};
  bool start_on_request_{true};
  bool stop_on_goal_{true};
  float goal_reached_dist_{std::numeric_limits<float>::quiet_NaN()};
  int mode_{2};

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
