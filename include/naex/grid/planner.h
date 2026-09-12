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
#include <geometry_msgs/msg/point.hpp>
#include <nav2_msgs/srv/clear_entire_costmap.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/srv/get_plan.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <std_msgs/msg/float32.hpp>
#include <tf2/exceptions.hpp>
#include <tf2/time.hpp>
#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.hpp>
#include <tf2_ros/transform_listener.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <memory>
#include <numeric>
#include <sstream>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <utility>
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
    pose.header.frame_id = path.header.frame_id;
    pose.header.stamp = path.header.stamp;
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
    // P6a: crop of the input cloud around the sensor; <= 0 or NaN disables it.
    input_range_ = nh_->declare_parameter<float>("input_range", input_range_);
    // P6b: bound of the grid itself; 0 (the default) keeps the pre-P6
    // behaviour, i.e. an unbounded map that only ever grows.
    map_range_ = nh_->declare_parameter<float>("map_range", map_range_);
    evict_period_ =
        nh_->declare_parameter<float>("evict_period", evict_period_);

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
    updateMaxCostsAbsolute(true);

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
          updateMaxCostsAbsolute(true);
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
    occupancy_grid_resolution_ = nh_->declare_parameter<float>(
        "occupancy_grid_resolution", occupancy_grid_resolution_);
    // Not including the ad-hoc (sidelobes) layer.
    // TODO: this is vulnerable to changes of cloud_weights.
    max_total_cost_ = static_cast<float>(
        std::reduce(cloud_weights_.begin(), cloud_weights_.end(), 0.0));

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
    occ_grid_pub_ = nh_->create_publisher<nav_msgs::msg::OccupancyGrid>(
        "map_occupancy_grid", rclcpp::SystemDefaultsQoS());
    // Debug only.
    frontiers_pub_ = nh_->create_publisher<visualization_msgs::msg::MarkerArray>(
        "map_frontiers", rclcpp::SystemDefaultsQoS());

    if (publish_occupancy_grid_) {
      RCLCPP_INFO(nh_->get_logger(),
                  "Publishing the occupancy grid on 'map_occupancy_grid' "
                  "(%d x %d cells, resolution %.3f, max map cost %.3f); it "
                  "feeds the nav2 global costmap.",
                  occupancy_grid_w_, occupancy_grid_h_,
                  occupancy_grid_resolution_, max_total_cost_);
      RCLCPP_INFO(nh_->get_logger(),
                  "Sending one empty msg to 'map_occupancy_grid' to "
                  "initialize the nav2 global_costmap.");
      geometry_msgs::msg::Pose p{};
      p.orientation.w = 1.0;
      createAndPublishMapOccupancyGrid(p);
    }

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

    if (input_range_ > 0.f) {
      RCLCPP_INFO(nh_->get_logger(),
                  "Input clouds are cropped to %.1f m around the sensor.",
                  input_range_);
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
                  map_range_, cellRadius(grid_, map_range_), evict_period_,
                  kEvictMoveFraction * map_range_);
    } else {
      RCLCPP_INFO(nh_->get_logger(),
                  "Map is unbounded (map_range %.1f): it grows for the whole "
                  "mission.",
                  map_range_);
    }

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

  /// Visualize the connected frontier component the goal selection picked.
  void visualizeFrontiers(const std::vector<Point2f> &points) {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = map_frame_;
    marker.header.stamp = nh_->get_clock()->now();
    marker.ns = "points";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::POINTS;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.2;
    marker.scale.y = 0.2;
    marker.color.r = 64.0;
    marker.color.g = 224.0;
    marker.color.b = 208.0;
    marker.color.a = 1.0;
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
                               const Value min_dist, const int max_neighbors) {
    const std::vector<std::uint8_t> &visited = sp.visited();
    const uint8_t nb = static_cast<uint8_t>(neighborhood_);
    const VertexId n = static_cast<VertexId>(grid_.size());

    // 1. Collect the frontier cells into a grid of their own.
    //    "traversable" == expanded by the search AND at least min_dist away.
    Grid frontiers_grid(grid_.cellSize(), 1.f, default_costs_);
    std::vector<Cost> frontier_costs;   // indexed by frontiers_grid cell id
    std::vector<std::uint8_t> traversable; // same index

    for (VertexId v = 0; v < n; ++v) {
      const bool is_traversable =
          visited[v] && (toVec3(grid_.point(v)) - start).norm() >= min_dist;
      const int degree = neighborDegree(grid_, v, nb, [&](CellId t) {
        return withinRange(grid_, t, v_start, astar_max_range_);
      });
      if (degree <= max_neighbors) {
        frontiers_grid.createCell(
            frontiers_grid.pointToCell(grid_.point(v)));
        frontier_costs.push_back(sp.fValue(v));
        traversable.push_back(is_traversable ? 1 : 0);
      }
    }

    // 2. Frontier components; keep the one that gets nearest to the goal.
    const VertexId nf = static_cast<VertexId>(frontiers_grid.size());
    std::vector<std::uint8_t> explored(nf, 0);
    Value best_nearest_dist = std::numeric_limits<Value>::infinity();
    std::vector<VertexId> best_component;
    // One graph for all seeds (it used to be rebuilt per seed).
    BFS bfs(frontiers_grid, max_costs_absolute_);

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
        const Value d = (toVec3(frontiers_grid.point(fv)) - goal).norm();
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
        const CellId v = grid_.findCell(
            grid_.pointToCell(frontiers_grid.point(fv)));
        if (v == INVALID_CELL_ID) {
          continue;
        }
        cheapest_cost = frontier_costs[fv];
        cheapest_frontier = v;
      }
    }

    if (!connected_frontier_points.empty()) {
      visualizeFrontiers(connected_frontier_points);
    }

    if (cheapest_frontier != INVALID_VERTEX_ID) {
      RCLCPP_WARN(nh_->get_logger(), "Cheapest frontier: %s, cost: %f",
                  format(toVec3(grid_.point(cheapest_frontier))).c_str(),
                  cheapest_cost);
    } else {
      RCLCPP_WARN(nh_->get_logger(), "No admissible frontier found!");
    }
    return cheapest_frontier;
  }

  /**
   * Nearest cell to @p p0 whose costs are in bounds, and its distance.
   *
   * The O(N) scan only runs when the robot cell is blocked or unexplored (P9).
   */
  std::pair<float, VertexId> getNearestTraversableVertex(const Vec3 &p0) {
    Timer t_scan;
    float best_dist = std::numeric_limits<float>::infinity();
    VertexId best_v = INVALID_VERTEX_ID;
    const VertexId n = static_cast<VertexId>(grid_.size());
    for (VertexId v = 0; v < n; ++v) {
      if (!costsInBounds(grid_.costs(v), max_costs_absolute_)) {
        continue;
      }
      const Value dist = (toVec3(grid_.point(v)) - p0).norm();
      if (dist < best_dist) {
        best_v = v;
        best_dist = dist;
      }
    }
    plan_timings_.scan_traversable = t_scan.seconds_elapsed();
    if (best_v != INVALID_VERTEX_ID) {
      RCLCPP_INFO(nh_->get_logger(),
                  "Closest traversable point to start: %s (dist %.3f).",
                  format(toVec3(grid_.point(best_v))).c_str(), best_dist);
    } else {
      RCLCPP_ERROR(nh_->get_logger(), "No traversable points in graph!");
    }
    return std::make_pair(best_dist, best_v);
  }

  /// Fall-back plan when the start is nowhere near anything traversable.
  void returnStraightLinePlan(nav_msgs::srv::GetPlan::Response::SharedPtr res,
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

  /**
   * Start cell of the search.
   *
   * @param straight_line set to true when the caller should answer with a
   *        straight line to the goal instead of planning.
   * @return the start cell, or INVALID_VERTEX_ID when planning must fail.
   */
  VertexId selectStartVertex(const Vec3 &p0, bool &straight_line) {
    straight_line = false;
    const Cell start_cell = grid_.pointToCell({p0.x(), p0.y()});
    const CellId v_start = grid_.findCell(start_cell);

    if (v_start != INVALID_CELL_ID &&
        costsInBounds(grid_.costs(v_start), max_costs_absolute_)) {
      // Start cell exists and is traversable: plan from there.
      RCLCPP_INFO(nh_->get_logger(), "Planning from start position %s.",
                  format(toVec3(grid_.point(v_start))).c_str());
      return v_start;
    }
    const bool explored = v_start != INVALID_CELL_ID;
    if (explored) {
      RCLCPP_WARN(nh_->get_logger(), "Start position %s is not traversable.",
                  format(toVec3(grid_.point(v_start))).c_str());
    } else {
      RCLCPP_WARN(nh_->get_logger(), "Start position %s is unexplored.",
                  format(p0).c_str());
    }

    const auto nearest = getNearestTraversableVertex(p0);
    const float best_dist = nearest.first;
    const VertexId best_v = nearest.second;
    if (best_v == INVALID_VERTEX_ID) {
      return INVALID_VERTEX_ID;
    }
    if (best_dist <= max_start_to_traversable_dist_) {
      RCLCPP_INFO(nh_->get_logger(),
                  "Planning from nearest traversable point %s.",
                  format(toVec3(grid_.point(best_v))).c_str());
      return best_v;
    }
    if (explored) {
      RCLCPP_ERROR(nh_->get_logger(),
                   "Start point is further than max_start_to_traversable_dist "
                   "(%.3f > %.3f m) from the closest traversable point %s. "
                   "Failed to plan!",
                   best_dist, max_start_to_traversable_dist_,
                   format(toVec3(grid_.point(best_v))).c_str());
      return INVALID_VERTEX_ID;
    }
    RCLCPP_WARN(nh_->get_logger(),
                "Start point is further than max_start_to_traversable_dist "
                "(%.3f > %.3f m) from the closest traversable point %s. "
                "Planning straight line to goal!",
                best_dist, max_start_to_traversable_dist_,
                format(toVec3(grid_.point(best_v))).c_str());
    straight_line = true;
    return INVALID_VERTEX_ID;
  }

  /// Goal cell for an A* search, or INVALID_VERTEX_ID when there is none.
  VertexId selectAstarGoalVertex(const ShortestPaths &sp, VertexId v0,
                                 VertexId v_goal, bool is_goal_explored,
                                 const Vec3 &p0, const Vec3 &p1) {
    VertexId v1 = INVALID_VERTEX_ID;
    bool consider_frontier = false;
    const Value euclidean_dist_to_goal = (toVec3(grid_.point(v0)) - p1).norm();
    const bool is_goal_in_obstacle =
        is_goal_explored &&
        !costsInBounds(grid_.costs(v_goal), max_costs_absolute_);

    if (sp.foundGoal()) {
      // The goal is reachable.  If the cost-optimal route is much longer than
      // the straight line, a frontier may still be the better target: this is
      // the "drive the whole explored loop backwards" case.
      const Value start_to_goal_dist =
          sp.cheapestPathEuclideanDist(grid_, v0, v_goal);
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
      v1 = nearestCell(grid_, Point2f(p1.x(), p1.y()), [&sp](CellId v) {
        // Unreached cells carry FLT_MAX (or INF when cropped out); see
        // ShortestPaths::kUnreachableCost.
        return !(sp.fValue(v) > ShortestPaths::kUnreachableCost);
      });
      plan_timings_.scan_reachable = t_scan.seconds_elapsed();
      if (v1 != INVALID_CELL_ID) {
        RCLCPP_INFO(nh_->get_logger(),
                    "Plan target is nearest reachable point to goal: %s",
                    format(toVec3(grid_.point(v1))).c_str());
      }
    } else {
      // Goal unexplored, hence unreachable: head for the cheapest frontier.
      Timer t_frontier;
      v1 = getCheapestFrontier(sp, v0, p0, p1, frontier_min_dist_,
                               frontier_max_neighbors_);
      plan_timings_.frontier = t_frontier.seconds_elapsed();
    }

    if (consider_frontier) {
      Timer t_frontier;
      const VertexId v_frontier = getCheapestFrontier(
          sp, v0, p0, p1, frontier_min_dist_, frontier_max_neighbors_);
      plan_timings_.frontier = t_frontier.seconds_elapsed();
      if (v_frontier != INVALID_VERTEX_ID) {
        const Value frontier_to_goal_dist =
            (toVec3(grid_.point(v_frontier)) - toVec3(grid_.point(v_goal)))
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
    start.header.stamp = nh_->now();
    goal.header.stamp = nh_->now();

    if (!request_frame.empty() && request_frame != map_frame_) {
      RCLCPP_WARN_THROTTLE(nh_->get_logger(), *nh_->get_clock(), 1000,
                           "Start pose frame_id '%s' does not match map "
                           "frame_id '%s'. Attempting to transform.",
                           request_frame.c_str(), map_frame_.c_str());
      start =
          tf_->transform(start, map_frame_, tf2::durationFromSec(tf_timeout_));
    }
    if (!goal.header.frame_id.empty() && goal.header.frame_id != map_frame_) {
      RCLCPP_WARN_THROTTLE(nh_->get_logger(), *nh_->get_clock(), 1000,
                           "Goal pose frame_id '%s' does not match map "
                           "frame_id '%s'. Attempting to transform.",
                           goal.header.frame_id.c_str(), map_frame_.c_str());
      goal = tf_->transform(goal, map_frame_, tf2::durationFromSec(tf_timeout_));
    }

    // If the start is not valid, use the robot position instead.
    // TODO: this could be dangerous with navigate-through-poses in theory.
    if (!isValid(start.pose.position)) {
      // tf2::TimePointZero ("latest available") avoids the clock-type
      // mismatch of rclcpp::Time(0) (system time) against a ROS-time buffer.
      Timer t_tf;
      const auto tf = tf_->lookupTransform(map_frame_, robot_frame_,
                                           tf2::TimePointZero,
                                           tf2::durationFromSec(tf_timeout_));
      plan_timings_.start_tf = t_tf.seconds_elapsed();
      transform_to_pose(tf, start);
      start.header.frame_id = map_frame_;
    }

    if (mode_ == 2) {
      start.pose.position.z = 0.f;
      goal.pose.position.z = 0.f;
    }

    const Vec3 p0 = toVec3(start.pose.position);
    Vec3 p1 = toVec3(goal.pose.position);

    if (stop_on_goal_) {
      // Stop if close to the goal.
      const float dist_to_goal = (p1 - p0).norm();
      RCLCPP_INFO(nh_->get_logger(), "Distance to goal: %.3f m.", dist_to_goal);
      if (dist_to_goal <= goal_reached_dist_) {
        stopPlanning();
        return false;
      }
    }

    // Figure out the starting cell of the search.  No Graph here (P2): a Graph
    // caches the per-vertex total cost and would be stale by the time the
    // ad-hoc layer below is rewritten, so the bounds check is the free
    // function.
    bool straight_line = false;
    const VertexId v0 = selectStartVertex(p0, straight_line);
    if (straight_line) {
      returnStraightLinePlan(res, start, goal);
      return true;
    }
    if (v0 == INVALID_VERTEX_ID) {
      return false;
    }

    // Apply ad-hoc costs if enabled.
    if (!adhoc_costs_.empty()) {
      Timer t_adhoc;
      clearAdHocLayer();

      // Make sure the start pose is close enough to the robot pose (it is not
      // with navigate-through-poses).  Not ideal, but it keeps the ad-hoc
      // banana from appearing where it should not.  This blocks for up to
      // tf_timeout_, so it is reported as TF time and not as ad-hoc time.
      Timer t_tf_robot;
      const auto start_in_robot_frame =
          tf_->transform(start, robot_frame_, tf2::durationFromSec(tf_timeout_));
      const double tf_robot_seconds = t_tf_robot.seconds_elapsed();
      plan_timings_.start_tf += tf_robot_seconds;
      if (std::fabs(start_in_robot_frame.pose.position.x) > 3. ||
          std::fabs(start_in_robot_frame.pose.position.y) > 3. ||
          std::fabs(start_in_robot_frame.pose.orientation.w) < 0.9) {
        RCLCPP_WARN(nh_->get_logger(),
                    "Start pose in robot frame is not close to the origin: %s, "
                    "orientation w %f. Ad-hoc costs not applied!",
                    format(start_in_robot_frame.pose.position).c_str(),
                    start_in_robot_frame.pose.orientation.w);
      } else {
        // Extract robot yaw from start pose orientation.
        const auto &q = start.pose.orientation;
        const float robot_yaw =
            std::atan2(2.0f * (q.w * q.z + q.x * q.y),
                       1.0f - 2.0f * (q.y * q.y + q.z * q.z));
        applyAdHocCosts(p0, robot_yaw);
        RCLCPP_DEBUG(nh_->get_logger(),
                     "Applied ad-hoc costs at robot position %s, yaw %.3f rad.",
                     format(p0).c_str(), robot_yaw);
      }
      // Timing is reported by logPlanSummary().
      plan_timings_.adhoc = t_adhoc.seconds_elapsed() - tf_robot_seconds;
    }

    if (!isValid(goal.pose.position)) {
      RCLCPP_WARN(nh_->get_logger(), "Goal not valid.");
      // TODO: Return random path in exploration mode.
      return false;
    }

    // Is the goal inside the mapped area?  Only then can A* stop on it.
    const CellId v_goal = grid_.findCell(grid_.pointToCell({p1.x(), p1.y()}));
    const bool is_goal_explored = v_goal != INVALID_CELL_ID;

    // Run the search.  Reused across requests (P2): the predecessor, path-cost
    // and f-value buffers keep their capacity, so a steady-state cycle
    // allocates nothing here.
    Timer t_search;
    ShortestPaths &sp = shortest_paths_;
    if (use_astar_) {
      sp.computeAstar(nh_->get_logger(), grid_, v0, p1, is_goal_explored,
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
    createAndPublishMapCloud(sp);
    plan_timings_.map_cloud = t_map.seconds_elapsed();

    // Pick the cell the path ends in.
    t_part.reset();
    VertexId v1 = INVALID_VERTEX_ID;
    if (use_astar_) {
      v1 = selectAstarGoalVertex(sp, v0, v_goal, is_goal_explored, p0, p1);
    } else {
      // Path to the reachable cell closest to the goal.
      p1.z() = 0.f;
      Timer t_scan;
      // p1.z() is zeroed above, so the 2-D distance used by nearestCell() is
      // the same value the 3-D norm used to produce.
      v1 = nearestCell(grid_, Point2f(p1.x(), p1.y()), [&sp](CellId v) {
        return std::isfinite(sp.pathCost(v));
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
    RCLCPP_DEBUG(nh_->get_logger(), "v1 %u x %f y %f goal", v1,
                 grid_.point(v1).x, grid_.point(v1).y);

    const auto path_vertices = tracePathVertices(v0, v1, sp.predecessors());
    nav_msgs::msg::Path local_plan;
    local_plan.header.frame_id = map_frame_;
    local_plan.header.stamp = nh_->get_clock()->now();
    local_plan.poses.push_back(start);
    appendPath(path_vertices, grid_, local_plan);
    // helhest 01/2026: add the actual goal point to the end of the path for
    // the goal checker down the path.
    local_plan.poses.push_back(goal);
    res->plan = std::move(local_plan);

    RCLCPP_INFO(nh_->get_logger(),
                "Path with %lu poses toward goal %s planned (%.3f s).",
                res->plan.poses.size(), format(p1).c_str(),
                t.seconds_elapsed());
    return true;
  }

  void fillMapCloud(sensor_msgs::msg::PointCloud2 &cloud, const Grid &grid,
                    const std::vector<Cost> &path_costs,
                    const std::vector<Cost> &f_values) {
    // TODO: Allow sending local map.
    append_field<float>("x", 1, cloud);
    append_field<float>("y", 1, cloud);
    append_field<float>("z", 1, cloud);
    append_field<float>("cost", 1, cloud);
    append_field<float>("path_cost", 1, cloud);
    append_field<float>("f_value", 1, cloud);
    const VertexId num_cells = static_cast<VertexId>(grid.size());
    resize_cloud(cloud, 1, num_cells);

    sensor_msgs::PointCloud2Iterator<float> x_it(cloud, "x");
    sensor_msgs::PointCloud2Iterator<float> cost_it(cloud, "cost");
    sensor_msgs::PointCloud2Iterator<float> path_cost_it(cloud, "path_cost");
    sensor_msgs::PointCloud2Iterator<float> f_value_it(cloud, "f_value");
    for (VertexId v = 0; v < num_cells;
         ++v, ++x_it, ++cost_it, ++path_cost_it, ++f_value_it) {
      const auto p = grid.point(v);
      x_it[0] = p.x;
      x_it[1] = p.y;
      x_it[2] = 0.f;
      cost_it[0] = grid.costs(v).total();
      path_cost_it[0] = path_costs[v];
      f_value_it[0] = f_values[v];
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
    fillMapCloud(*cloud, grid_, sp.pathCosts(), sp.fValues());
    map_pub_->publish(std::move(cloud));
  }

  /**
   * Rasterize the cost grid into a nav_msgs/OccupancyGrid centred on @p start.
   *
   * Cells that are in bounds are free (0), cells that are not are 127; cells
   * the planner has never seen stay unknown (-1).  This is what the nav2
   * global costmap consumes.
   */
  void fillMapOccupancyGrid(nav_msgs::msg::OccupancyGrid &occ_grid) {
    occ_grid.data.assign(
        static_cast<size_t>(occ_grid.info.width) * occ_grid.info.height, -1);
    const VertexId n = static_cast<VertexId>(grid_.size());
    for (VertexId v = 0; v < n; ++v) {
      const int data_idx = pointToOccupancyGridCell(grid_.point(v), occ_grid);
      if (data_idx < 0) {
        continue;
      }
      occ_grid.data[static_cast<size_t>(data_idx)] =
          costsInBounds(grid_.costs(v), max_costs_absolute_) ? 0 : 127;
    }
  }

  /// Index of @p p in @p occ_grid, or -1 when it falls outside.
  int pointToOccupancyGridCell(const Point2f &p,
                               const nav_msgs::msg::OccupancyGrid &occ_grid) {
    const double dx = p.x - occ_grid.info.origin.position.x;
    const double dy = p.y - occ_grid.info.origin.position.y;
    const int64_t cell_x =
        static_cast<int64_t>(std::floor(dx / occ_grid.info.resolution));
    const int64_t cell_y =
        static_cast<int64_t>(std::floor(dy / occ_grid.info.resolution));
    if (cell_x < 0 || cell_x >= static_cast<int64_t>(occ_grid.info.width)) {
      return -1;
    }
    if (cell_y < 0 || cell_y >= static_cast<int64_t>(occ_grid.info.height)) {
      return -1;
    }
    return static_cast<int>(cell_x +
                            static_cast<int64_t>(occ_grid.info.width) * cell_y);
  }

  /// Bottom-left corner of an occupancy grid centred on @p robot_pose.
  geometry_msgs::msg::Point
  getOccupancyGridOrigin(const geometry_msgs::msg::Pose &robot_pose,
                         const nav_msgs::msg::OccupancyGrid &occ_grid) {
    geometry_msgs::msg::Point origin;
    origin.x = robot_pose.position.x -
               static_cast<double>(occ_grid.info.width) / 2. *
                   occ_grid.info.resolution;
    origin.y = robot_pose.position.y -
               static_cast<double>(occ_grid.info.height) / 2. *
                   occ_grid.info.resolution;
    return origin;
  }

  void createAndPublishMapOccupancyGrid(const geometry_msgs::msg::Pose &start) {
    auto occ_grid = std::make_unique<nav_msgs::msg::OccupancyGrid>();
    occ_grid->header.frame_id = map_frame_;
    const auto now = nh_->get_clock()->now();
    occ_grid->header.stamp = now;
    occ_grid->info.map_load_time = now;
    occ_grid->info.resolution = grid_.cellSize();
    occ_grid->info.width = static_cast<uint32_t>(std::max(0, occupancy_grid_w_));
    occ_grid->info.height =
        static_cast<uint32_t>(std::max(0, occupancy_grid_h_));
    // Assume the start pose of the request is the robot's current pose.
    occ_grid->info.origin.position = getOccupancyGridOrigin(start, *occ_grid);
    occ_grid->info.origin.orientation.w = 1.0;
    fillMapOccupancyGrid(*occ_grid);
    occ_grid_pub_->publish(std::move(occ_grid));
  }

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
  void logPlanSummary() const {
    RCLCPP_INFO_THROTTLE(
        nh_->get_logger(), *nh_->get_clock(), 1000,
        "perf plan: cells=%lu map_range=%.1f astar=%d tf=%.4f adhoc=%.4f "
        "dijkstra=%.4f map_cloud=%.4f scan_trav=%.4f scan_reach=%.4f "
        "frontier=%.4f total=%.4f",
        static_cast<unsigned long>(plan_timings_.grid_size), map_range_,
        use_astar_ ? 1 : 0, plan_timings_.start_tf, plan_timings_.adhoc,
        plan_timings_.dijkstra, plan_timings_.map_cloud,
        plan_timings_.scan_traversable, plan_timings_.scan_reachable,
        plan_timings_.frontier, plan_timings_.total);
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
   * operation that invalidates CellIds (clearMap(), the P6 eviction) must
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
    for (size_t pt = 0; pt < num_pts; ++pt, ++x_it) {
      // Non-finite input must be rejected before the cast in pointToCell()
      // (undefined behaviour, phantom cells), and the crop keeps the per-cloud
      // work bounded by input_range instead of by the size of the cloud (P6a).
      const Vec3 raw(x_it[0], x_it[1], x_it[2]);
      bool keep = isValid(raw);
      Point2f p(0.f, 0.f);
      if (keep) {
        const Vec3 q = transform * raw;
        p = Point2f(q.x(), q.y());
        keep = acceptInputPoint(grid_, p, origin, input_range_);
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
      for (size_t j = 0; j < levels.size(); ++j) {
        if (std::isfinite(cost_iters[j][0])) {
          grid_.updatePointCost(p, levels[j], weights[j] * cost_iters[j][0]);
        }
        ++cost_iters[j];
      }
    }
    const double loop_seconds = t_loop.seconds_elapsed();

    Timer t_evict;
    const bool evicted = maybeEvictCells(origin);
    // One line per cloud; see logPlanSummary() for the planning-side line.
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
      // executor thread for tf_timeout_.
      geometry_msgs::msg::PoseStamped robot_pose;
      const auto robot_to_map = tf_->lookupTransform(
          map_frame_, robot_frame_, tf2::TimePointZero,
          tf2::durationFromSec(cloud_tf_timeout_));
      transform_to_pose(robot_to_map, robot_pose);
      createAndPublishMapOccupancyGrid(robot_pose.pose);
    }
  }

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
   * input_range_ larger than map_range_ (or disabled) every cloud re-creates
   * the cells the last eviction dropped.  That is correct but wasteful, so in
   * production keep input_range_ <= map_range_.
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
  bool maybeEvictCells(const Point2f &robot) {
    if (!(map_range_ > 0.f) || grid_.empty()) {
      return false;
    }
    if (!inCellRange(grid_, robot)) {
      RCLCPP_WARN_THROTTLE(nh_->get_logger(), *nh_->get_clock(), 5000,
                           "Not evicting: robot position (%.1f, %.1f) is not a "
                           "valid grid cell.",
                           robot.x, robot.y);
      return false;
    }
    const double now = nh_->get_clock()->now().seconds();
    const float dx = robot.x - last_evict_at_.x;
    const float dy = robot.y - last_evict_at_.y;
    const bool moved = !std::isfinite(last_evict_at_.x) ||
                       !std::isfinite(last_evict_at_.y) ||
                       std::sqrt(dx * dx + dy * dy) >=
                           kEvictMoveFraction * map_range_;
    // Not (now - last < period), so that a clock jump backwards evicts rather
    // than blocks eviction forever.
    const bool due = !(now - last_evict_time_ < evict_period_);
    const bool grown =
        static_cast<double>(grid_.size()) >
        kEvictSizeFactor * boundedCellCount(cellRadius(grid_, map_range_));
    if (!moved && !due && !grown) {
      return false;
    }

    // The dirty list holds CellIds; replay it while they still mean something.
    clearAdHocLayer();
    const Eviction ev = evictOutsideRange(grid_, robot, map_range_);
    last_evict_at_ = robot;
    last_evict_time_ = now;
    if (ev.changed()) {
      RCLCPP_INFO_THROTTLE(nh_->get_logger(), *nh_->get_clock(), 1000,
                           "perf evict: map_range=%.1f center=(%.1f, %.1f) "
                           "cells_before=%lu cells_after=%lu removed=%lu",
                           map_range_, robot.x, robot.y,
                           static_cast<unsigned long>(ev.before),
                           static_cast<unsigned long>(ev.after),
                           static_cast<unsigned long>(ev.removed));
    }
    return true;
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
  void updateMaxCostsAbsolute(bool log) {
    max_costs_absolute_ = max_costs_;
    for (size_t i = 0; i < Costs::kSize; ++i) {
      if (!std::isfinite(max_costs_relative_[i]) ||
          i >= cloud_weights_.size()) {
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
  rclcpp::Service<nav2_msgs::srv::ClearEntireCostmap>::SharedPtr
      clear_map_service_;

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
