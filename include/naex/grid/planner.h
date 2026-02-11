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
// #include <memory>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav2_msgs/srv/clear_entire_costmap.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/srv/get_plan.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <std_msgs/msg/float32.hpp>
#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <unordered_map>
// #include <geometry_msgs/msg/point.hpp>
// #include <tf2/LinearMath/Vector3.h>
// #include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace naex {
namespace grid {

template <typename T> std::string format(T x, T y, T z) {
  std::stringstream s;
  s << "(" << x << ", " << y << ", " << z << ")";
  return s.str();
}
std::string format(const geometry_msgs::msg::Vector3 &v) {
  return format(v.x, v.y, v.z);
}
std::string format(const geometry_msgs::msg::Point &v) {
  return format(v.x, v.y, v.z);
}
std::string format(const Vec3 &v) { return format(v.x(), v.y(), v.z()); }

Vec3 toVec3(const geometry_msgs::msg::Point &p) { return Vec3(p.x, p.y, p.z); }
Vec3 toVec3(const geometry_msgs::msg::Vector3 &v) {
  return Vec3(v.x, v.y, v.z);
}
Vec3 toVec3(const Point2f &v) { return Vec3(v.x, v.y, 0.f); }

void tracePathVertices(VertexId v0, VertexId v1,
                       const std::vector<VertexId> &predecessor,
                       std::vector<VertexId> &path_vertices) {
  assert(predecessor[v0] == v0);
  Vertex v = v1;
  while (v != v0) {
    path_vertices.push_back(v);
    v = predecessor[v];
  }
  path_vertices.push_back(v);
  std::reverse(path_vertices.begin(), path_vertices.end());
}

std::vector<VertexId>
tracePathVertices(VertexId v0, VertexId v1,
                  const std::vector<VertexId> &predecessor) {
  std::vector<VertexId> path_vertices;
  tracePathVertices(v0, v1, predecessor, path_vertices);
  return path_vertices;
}

void appendPath(const std::vector<VertexId> &path_vertices, const Grid &grid,
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

template <typename T> bool isValid(T x, T y, T z) {
  return std::isfinite(x) && std::isfinite(y) && std::isfinite(z);
}
bool isValid(const Vec3 &p) { return isValid(p.x(), p.y(), p.z()); }
bool isValid(const geometry_msgs::msg::Point &p) {
  return isValid(p.x, p.y, p.z);
}
bool isValid(const geometry_msgs::msg::Vector3 &p) {
  return isValid(p.x, p.y, p.z);
}

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
    map_frame_ = nh_->declare_parameter<std::string>("map_frame", map_frame_);
    robot_frame_ =
        nh_->declare_parameter<std::string>("robot_frame", robot_frame_);
    tf_timeout_ = nh_->declare_parameter<float>("tf_timeout", tf_timeout_);

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

    std::vector<float> max_costs(cost_fields_.size());
    std::vector<float> default_costs(cost_fields_.size());
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

    std::vector<double> cost_thresholds(cost_fields_.size(), -1);
    cost_thresholds_ = nh_->declare_parameter<std::vector<double>>("cost_thresholds", cost_thresholds);

    // We want to change 'cost_thresholds' as a recovery.
    param_subscriber_ = std::make_shared<rclcpp::ParameterEventHandler>(nh_);
    auto cb = [this](const rclcpp::Parameter & p) {
      RCLCPP_INFO(
        nh_->get_logger(), "cb: Received an update to parameter \"%s\" of type %s:",
        p.get_name().c_str(),
        p.get_type_name().c_str());
      cost_thresholds_ = p.as_double_array();
      for (double c : cost_thresholds_) {
        RCLCPP_INFO(nh_->get_logger(), "%f", c);
      }
    };
    cb_handle_ = param_subscriber_->add_parameter_callback("cost_thresholds", cb);

    //Astar
    use_astar_ = nh_->declare_parameter<bool>("use_astar", use_astar_);
    astar_max_range_ = nh_->declare_parameter<float>("astar_max_range", astar_max_range_);
    frontier_min_dist_ = nh_->declare_parameter<float>("frontier_min_dist", frontier_min_dist_);
    frontier_max_neighbors_ = nh_->declare_parameter<int>("frontier_max_neighbors", frontier_max_neighbors_);

    // Ad-hoc cost parameters
    adhoc_costs_ = nh_->declare_parameter("adhoc_costs", adhoc_costs_);
    adhoc_layer_ = nh_->declare_parameter("adhoc_layer", adhoc_layer_);

    // Sidelobes strategy parameters
    sidelobes_offset_distance_ = nh_->declare_parameter(
        "sidelobes_offset_distance", sidelobes_offset_distance_);
    sidelobes_radius_ = nh_->declare_parameter(
        "sidelobes_radius", sidelobes_radius_);
    sidelobes_cost_ = nh_->declare_parameter(
        "sidelobes_cost", sidelobes_cost_);
    sidelobes_angle_offsets_ = nh_->declare_parameter(
        "sidelobes_angle_offsets", sidelobes_angle_offsets_);

    publish_occupancy_grid_ = nh_->declare_parameter<bool>("publish_occupancy_grid", publish_occupancy_grid_);
    occupancy_grid_w_ = nh_->declare_parameter<int>("occupancy_grid_w", occupancy_grid_w_);
    occupancy_grid_h_ = nh_->declare_parameter<int>("occupancy_grid_h", occupancy_grid_h_);
    occupancy_grid_resolution_ = nh_->declare_parameter<float>("occupancy_grid_resolution", occupancy_grid_resolution_);
    max_total_cost_ = std::reduce(cloud_weights_.begin(), cloud_weights_.end());; // not including sidelobes TODO: this is vulnerable to changes
    
    if (publish_occupancy_grid_) {
      RCLCPP_INFO(nh_->get_logger(),
        "Will be publishing occupancy grid at '/map_occupancy_grid'. This is used primarily to supply global costmap of nav2 with planner information.");
      RCLCPP_INFO(nh_->get_logger(),
        "Some of the settings for '/map_occupancy_grid' are:\nwidth: %d\nheight: %d\nresolution: %f\nmax cost of map: %f",
        occupancy_grid_w_,
        occupancy_grid_h_,
        occupancy_grid_resolution_,
        max_total_cost_
      );
    }
      
    tf_ = std::make_shared<tf2_ros::Buffer>(nh_->get_clock());
    tf_sub_ = std::make_shared<tf2_ros::TransformListener>(*tf_);

    map_pub_ = nh_->create_publisher<sensor_msgs::msg::PointCloud2>("map", 2);
    local_map_pub_ =
        nh_->create_publisher<sensor_msgs::msg::PointCloud2>("local_map", 2);
    path_pub_ = nh_->create_publisher<nav_msgs::msg::Path>("path", 2);
    planning_freq_pub_ =
        nh_->create_publisher<std_msgs::msg::Float32>("planning_freq", 2);
    occ_grid_pub_ = nh_->create_publisher<nav_msgs::msg::OccupancyGrid>("map_occupancy_grid", rclcpp::SystemDefaultsQoS());

    if(publish_occupancy_grid_) {
      RCLCPP_INFO(nh_->get_logger(),
      "Sending one empty msg to '/map_occupancy_grid' to initialize nav2 global_costmap.");
      geometry_msgs::msg::Pose p{};
      p.position.x = 0.0;
      p.position.y = 0.0;
      p.position.z = 0.0;
      p.orientation.x = 0.0;
      p.orientation.y = 0.0;
      p.orientation.z = 0.0;
      p.orientation.w = 1.0;
      createAndPublishMapOccupancyGrid(p);
    }
    
    for (int i = 0; i < num_input_clouds; ++i) {
      std::stringstream ss;
      ss << "input_cloud_" << i;
      input_cloud_subs_.push_back(
          nh_->create_subscription<sensor_msgs::msg::PointCloud2>(
              ss.str(), queue_size,
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
        "get_plan", std::bind(&Planner::requestPlan, this,
                              std::placeholders::_1, std::placeholders::_2));
    clear_map_service_ =
        nh_->create_service<nav2_msgs::srv::ClearEntireCostmap>(
            "clear_plan_map",
            std::bind(&Planner::clearMap, this, std::placeholders::_1,
                      std::placeholders::_2));

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
        std::bind(&Planner::planningTimer, this));
    auto msg = std::make_shared<std_msgs::msg::Float32>();
    msg->data = planning_freq_;
    planning_freq_pub_->publish(*msg);
    RCLCPP_WARN(nh_->get_logger(), "Planning started.");
  }

  nav_msgs::msg::Path emptyPath() {
    nav_msgs::msg::Path msg;
    msg.header.frame_id = map_frame_;
    msg.header.stamp = nh_->get_clock()->now();
    return msg;
  }

  void stopPlanning() {
    planning_timer_->cancel();
    path_pub_->publish(emptyPath());
    auto msg = std::make_shared<std_msgs::msg::Float32>();
    msg->data = 0;
    planning_freq_pub_->publish(*msg);
    RCLCPP_WARN(nh_->get_logger(), "Planning stopped.");
  }

  VertexId getCheapestFrontier(const Graph &graph, const Vec3 &start, const Value min_dist, const int max_neighbors, const ShortestPaths &sp) {
    VertexId cheapest_frontier{INVALID_VERTEX};
    Cost cheapest_cost = std::numeric_limits<float>::infinity();
    for (VertexId v = 0; v < graph.grid().size(); ++v) {

      auto cost = sp.fValue(v);
      // RCLCPP_WARN(nh_->get_logger(), "fval x: %f, y: %f, c: %f", graph.grid().point(v).x, graph.grid().point(v).y, cost);

      if (!std::isfinite(cost) || cost > 10e9) {
        // BUG: for some reason sometimes instead of inf the cost is 340282346638528859811704183484516925440.000000
        // unclear why
        // RCLCPP_WARN(nh_->get_logger(), "inft x: %f, y: %f, c: %f", graph.grid().point(v).x, graph.grid().point(v).y, cost);
        continue;
      }

      // Check distance
      Value dist = (toVec3(graph.grid().point(v)) - start).norm();
      if (dist < min_dist) {
        continue;
      }

      // Check degree
      // std::pair<EdgeIter, EdgeIter> out_edges = graph.out_edges(v);
      int deg = 0;
      // for (auto edge:out_edges) {
      typename boost::graph_traits<Graph>::out_edge_iterator ei, ei_end;
      for(boost::tie(ei, ei_end) = boost::out_edges(v, graph); ei != ei_end; ++ei) {
        if (boost::source(*ei, graph) != boost::target(*ei, graph)) {
          deg += 1;
        }
      }
      if (deg <= max_neighbors) {
        // RCLCPP_WARN(nh_->get_logger(), "x: %f, y: %f, c: %f", graph.grid().point(v).x, graph.grid().point(v).y, cost);
        // Check if cheapest
        if (cost < cheapest_cost) {
          cheapest_cost = cost;
          cheapest_frontier = v;
        }
      } else {
        continue;
      }
    }
    if (cheapest_frontier != INVALID_VERTEX) {
      RCLCPP_WARN(nh_->get_logger(), "CHEAPEST FRONTIER: x: %f, y: %f, c: %f", graph.grid().point(cheapest_frontier).x, graph.grid().point(cheapest_frontier).y, cheapest_cost);
    } else {
      RCLCPP_WARN(nh_->get_logger(), "No admissible frontier found!");
    }
    return cheapest_frontier;
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
      const auto tf =
          tf_->lookupTransform(map_frame_, robot_frame_, rclcpp::Time(0),
                               rclcpp::Duration::from_seconds(tf_timeout_));
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
    if (!graph.costsInBounds(v0)) {
      RCLCPP_WARN(nh_->get_logger(), "Robot position %s is not traversable.",
                  format(toVec3(grid_.point(v0))).c_str());
    }

    // Use the nearest traversable point to robot as the starting point.
    float best_dist = std::numeric_limits<float>::infinity();
    for (VertexId v = 0; v < grid_.size(); ++v) {
      if (!graph.costsInBounds(grid_.costs(v))) {
        continue;
      }

      Value dist = (toVec3(grid_.point(v)) - p0).norm();
      if (dist < best_dist) {
        v0 = v;
        best_dist = dist;
      }
    }
    RCLCPP_INFO(nh_->get_logger(),
                "Closest traversable point to start: %s (%.3f).",
                format(toVec3(grid_.point(v0))).c_str(), best_dist);

    // Apply ad-hoc costs if enabled
    if (!adhoc_costs_.empty()) {
      Timer t_adhoc;
      clearAdHocLayer();
      
      // Extract robot yaw from start pose orientation
      auto &q = start.pose.orientation;
      float robot_yaw = atan2(2.0f * (q.w * q.z + q.x * q.y),
                              1.0f - 2.0f * (q.y * q.y + q.z * q.z));
      
      applyAdHocCosts(p0, robot_yaw);
      RCLCPP_DEBUG(nh_->get_logger(),
                   "Applied ad-hoc costs at robot position %s, yaw %.3f rad: %.3f s.",
                   format(p0).c_str(), robot_yaw, t_adhoc.seconds_elapsed());
    }

    // Point2f goal_point = Point2f(p1.x(), p1.y());
    VertexId v_goal = grid_.cellId(grid_.pointToCell({p1.x(), p1.y()}));

    if (!isValid(req->goal.pose.position)) {
      RCLCPP_WARN(nh_->get_logger(), "Goal not valid.");
      // TODO: Return random path in exploration mode.
      return false; 
    }

    ShortestPaths sp(nh_, grid_, v0, v_goal, use_astar_, astar_max_range_, cost_thresholds_, cloud_weights_, neighborhood_, max_costs_);
    if (use_astar_) {
      RCLCPP_INFO(nh_->get_logger(), "AStar (%lu pts before filtering): %.3f s.", grid_.size(),
                  t_part.seconds_elapsed());
    } else {
      RCLCPP_INFO(nh_->get_logger(), "Dijkstra (%lu pts before filtering): %.3f s.", grid_.size(),
                  t_part.seconds_elapsed());
    }
    createAndPublishMapCloud(sp);

    if(publish_occupancy_grid_) {
      createAndPublishMapOccupancyGrid(start.pose);
    }

    // Choose which point we select as the goal
    VertexId v1 = INVALID_VERTEX;

    if (use_astar_) {
      bool consider_frontier = false;
      Value euclidean_dist_to_goal = (toVec3(grid_.point(v0)) - toVec3(grid_.point(v_goal))).norm();
      if (sp.astar_found_goal()) {
        // If using astar and the goal point is part of the graph.
        // In this case calculate the distance of the cheapest path.
        // If it is not too long compared to the euclidean distance between start and goal
        // then consider it admissible.
        // If it is too long, consider the cheapest frontier as a substitute.
        Value start_to_goal_dist = sp.cheapest_path_euclidean_dist(v0, v_goal);
        if (start_to_goal_dist > max_relative_dist_to_goal_ * euclidean_dist_to_goal) {
          consider_frontier = true;
        } else {
          v1 = v_goal;
        }
      } else if (consider_frontier) {
        // If found start but the distance is long, so we consider the frontier.
        // That is if the frontier gets us closer to the goal.
        auto v_frontier = getCheapestFrontier(sp.graph(), p0, frontier_min_dist_, frontier_max_neighbors_, sp); 
        Value frontier_to_goal_dist = sp.cheapest_path_euclidean_dist(v0, v_frontier);
        if (frontier_to_goal_dist < euclidean_dist_to_goal) {
          v1 = v_frontier;
        } else {
          v1 = v_goal;
        }
      } else {
        // If goal unreachable, select the cheapest frontier point as a temporary goal.
        v1 = getCheapestFrontier(sp.graph(), p0, frontier_min_dist_, frontier_max_neighbors_, sp); 
      }
    } else {
      // If planning for a given goal, return path to the closest reachable
      // point from the goal.
      t_part.reset();
      Vec3 p1 = toVec3(req->goal.pose.position);
      p1.z() = 0.f;
      
      Value best_dist = std::numeric_limits<Cost>::infinity();
      // TODO: Use graph vertex iterator.
      for (VertexId v = 0; v < grid_.size(); ++v) {
        if (!std::isfinite(sp.pathCost(v))) {
          continue;
        }
        
        Value dist = (toVec3(grid_.point(v)) - p1).norm();
        if (dist < best_dist) {
          v1 = v;
          best_dist = dist;
        }
      }  
    }

    if (v1 == INVALID_VERTEX) {
      RCLCPP_ERROR(nh_->get_logger(),
      "No feasible path towards %s was found (%.6f, %.3f s).",
      format(p1).c_str(), t_part.seconds_elapsed(),
      t.seconds_elapsed());
      return false;
    }

    RCLCPP_WARN(nh_->get_logger(), "GOAL POINT: x: %f, y: %f", graph.grid().point(v1).x, graph.grid().point(v1).y);

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

  void fillMapCloud(sensor_msgs::msg::PointCloud2 &cloud, const Grid &grid,
                    const std::vector<Cost> &path_costs, const std::vector<Cost> &f_values) {
    // TODO: Allow sending local map.
    append_field<float>("x", 1, cloud);
    append_field<float>("y", 1, cloud);
    append_field<float>("z", 1, cloud);
    append_field<float>("cost", 1, cloud);
    append_field<float>("path_cost", 1, cloud);
    append_field<float>("f_value", 1, cloud);
    resize_cloud(cloud, 1, grid_.size());

    sensor_msgs::PointCloud2Iterator<float> x_it(cloud, "x");
    sensor_msgs::PointCloud2Iterator<float> cost_it(cloud, "cost");
    sensor_msgs::PointCloud2Iterator<float> path_cost_it(cloud, "path_cost");
    sensor_msgs::PointCloud2Iterator<float> f_values_it(cloud, "f_value");
    for (VertexId v = 0; v < grid_.size();
         ++v, ++x_it, ++cost_it, ++path_cost_it, ++f_values_it) {
      const auto p = grid_.point(v);
      x_it[0] = p.x;
      x_it[1] = p.y;
      x_it[2] = 0.f;
      cost_it[0] = grid_.costs(v).total();
      path_cost_it[0] = path_costs[v];
      f_values_it[0] = f_values[v];
    }
  }

  void createAndPublishMapCloud(const ShortestPaths &sp) {
    sensor_msgs::msg::PointCloud2 cloud;
    cloud.header.frame_id = map_frame_;
    cloud.header.stamp = nh_->get_clock()->now();
    fillMapCloud(cloud, grid_, sp.pathCosts(), sp.fValues());
    map_pub_->publish(cloud);
  }
  
  void fillMapOccupancyGrid(nav_msgs::msg::OccupancyGrid &occ_grid) {
    occ_grid.data.assign(
        occ_grid.info.width * occ_grid.info.height,
        -1
    );
    
    for (VertexId v = 0; v < grid_.size(); ++v) {
      const auto p = grid_.point(v);
      int data_idx = pointToOccupancyGridCell(p, occ_grid);
      if (data_idx==-1) {
        continue;
      }
      // RCLCPP_WARN(nh_->get_logger(), "cell costs %f %f %f %f", grid_.costs(v)[0], grid_.costs(v)[1], grid_.costs(v)[2], grid_.costs(v)[3]);
      if (naex::grid::is_obstacle(v, grid_, cost_thresholds_, cloud_weights_)) {
        occ_grid.data[data_idx] = 127;
      } else {
        occ_grid.data[data_idx] = 0;
      }
      // occ_grid.data[data_idx] = std::clamp((int)(grid_.costs(v).total()*127/max_total_cost_), 0, 10);
    }
  }

  int pointToOccupancyGridCell(const Point2f &p, nav_msgs::msg::OccupancyGrid &occ_grid) {
    int cell = -1;
    geometry_msgs::msg::Point cell_p;
    cell_p.x = p.x - occ_grid.info.origin.position.x;
    cell_p.y = p.y - occ_grid.info.origin.position.y;

    int cell_x = std::floor(cell_p.x/occ_grid.info.resolution);
    int cell_y = std::floor(cell_p.y/occ_grid.info.resolution);

    if (cell_x >= 0 && cell_x < occ_grid.info.width) {
      if (cell_y >= 0 && cell_y < occ_grid.info.height) {
        cell = cell_x + occ_grid.info.width * cell_y;
      }
    }
    return cell;
  }

  geometry_msgs::msg::Point getOccupancyGridOrigin(const geometry_msgs::msg::Pose &robot_pose,
                                                    const nav_msgs::msg::OccupancyGrid &occ_grid) {
    geometry_msgs::msg::Point origin_vec;

    origin_vec.x = -(float)(occ_grid.info.width)/2.*occ_grid.info.resolution;
    origin_vec.y = -(float)(occ_grid.info.height)/2.*occ_grid.info.resolution;
      
    geometry_msgs::msg::Point origin;
    origin.x = robot_pose.position.x + origin_vec.x;
    origin.y = robot_pose.position.y + origin_vec.y;

    return origin;
  }

  void createAndPublishMapOccupancyGrid(const geometry_msgs::msg::Pose &start) {
    nav_msgs::msg::OccupancyGrid occ_grid;
    occ_grid.header.frame_id = map_frame_;
    auto now = nh_->get_clock()->now();
    occ_grid.header.stamp = now;
    occ_grid.info.map_load_time = now;
    occ_grid.info.resolution = grid_.cellSize();
    occ_grid.info.width = occupancy_grid_w_;
    occ_grid.info.height = occupancy_grid_h_;
    
    geometry_msgs::msg::Point origin = getOccupancyGridOrigin(start, occ_grid);
    occ_grid.info.origin.position = origin;         // assume the starting pose from the request is the robot's current pose

    fillMapOccupancyGrid(occ_grid);
    occ_grid_pub_->publish(occ_grid);
  }

  bool planSafe(nav_msgs::srv::GetPlan::Request::SharedPtr req,
                nav_msgs::srv::GetPlan::Response::SharedPtr res) {
    try {
      return plan(req, res);
    } catch (const tf2::TransformException &ex) {
      RCLCPP_ERROR(nh_->get_logger(), "Transform failed: %s.", ex.what());
      return false;
    }
  }

  bool requestPlan(nav_msgs::srv::GetPlan::Request::SharedPtr req,
                   nav_msgs::srv::GetPlan::Response::SharedPtr res) {
    RCLCPP_INFO(nh_->get_logger(), "Planning request received.");
    if (start_on_request_) {
      startPlanning();
    }
    return planSafe(req, res);
  }

  void clearMap(nav2_msgs::srv::ClearEntireCostmap::Request::SharedPtr req,
                nav2_msgs::srv::ClearEntireCostmap::Response::SharedPtr res) {
    grid_.clear();
    RCLCPP_WARN(nh_->get_logger(), "Map cleared.");
  }

  void clearAdHocLayer() {
    if (adhoc_layer_ < 0 || adhoc_layer_ >= 4) {
      return;
    }
    Cost default_cost = default_costs_[adhoc_layer_];
    for (VertexId v = 0; v < grid_.size(); ++v) {
      grid_.costs(v)[adhoc_layer_] = default_cost;
    }
  }

  void applySidelobesCosts(const Vec3 &robot_pos, float robot_yaw) {
    if (adhoc_layer_ < 0 || adhoc_layer_ >= 4) {
      return;
    }

    for (const auto &angle_offset : sidelobes_angle_offsets_) {
      float angle_rad = angle_offset * M_PI / 180.0f;
      Vec3 center(
          robot_pos.x() + sidelobes_offset_distance_ * cos(robot_yaw + angle_rad),
          robot_pos.y() + sidelobes_offset_distance_ * sin(robot_yaw + angle_rad),
          0.0f);
      
      for (VertexId v = 0; v < grid_.size(); ++v) {
        Vec3 cell_pos = toVec3(grid_.point(v));
        float dist = (cell_pos - center).norm();
        
        if (dist <= sidelobes_radius_) {
          grid_.costs(v)[adhoc_layer_] = sidelobes_cost_;
        }
      }
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
    path_pub_->publish(res->plan);
    RCLCPP_INFO(nh_->get_logger(),
                "Planning robot %s path (%lu poses) in map %s: %.3f s.",
                robot_frame_.c_str(), res->plan.poses.size(),
                map_frame_.c_str(), t.seconds_elapsed());
  }

  void receiveCloud(
      const std::shared_ptr<const sensor_msgs::msg::PointCloud2> &input,
      int i) {
    const auto age = (nh_->get_clock()->now() - input->header.stamp).seconds();
    if (age > max_cloud_age_) {
      RCLCPP_INFO(nh_->get_logger(),
                  "Skipping old input cloud from %s, age %.1f s > %.1f s.",
                  input->header.frame_id.c_str(), age, max_cloud_age_);
      return;
    }

    geometry_msgs::msg::TransformStamped cloud_to_map;
    cloud_to_map = tf_->lookupTransform(
        map_frame_, input->header.frame_id, input->header.stamp,
        rclcpp::Duration::from_seconds(tf_timeout_));

    Eigen::Isometry3f transform(tf2::transformToEigen(cloud_to_map.transform));
    sensor_msgs::PointCloud2ConstIterator<float> x_it(*input, position_field_);

    std::vector<uint8_t> levels;
    std::vector<uint8_t> weights;
    std::vector<sensor_msgs::PointCloud2ConstIterator<float>> cost_iters;

    for (int j = 0; j < cost_fields_.size(); ++j) {
      if (which_cloud_[j] == i) {
        levels.push_back(j < cloud_levels_.size() ? cloud_levels_[j] : j);
        weights.push_back(j < cloud_weights_.size() ? cloud_weights_[j] : 1.0);
        const std::string cost_field =
            j < cost_fields_.size() ? cost_fields_[j] : "cost";
        cost_iters.push_back(
            sensor_msgs::PointCloud2ConstIterator<float>(*input, cost_field));
      }
    }

    for (int i = 0; i < input->height * input->width; ++i, ++x_it) {
      Vec3 p(x_it[0], x_it[1], x_it[2]);
      p = transform * p;
      for (int j = 0; j < levels.size(); ++j) {
        if (std::isfinite(cost_iters[j][0])) {
          grid_.updatePointCost({p.x(), p.y()}, levels[j],
                                weights[j] * cost_iters[j][0]);
        }
        ++cost_iters[j];
      }
    }
  }

  void receiveCloudSafe(
      const std::shared_ptr<const sensor_msgs::msg::PointCloud2> &input,
      uint8_t level) {
    try {
      receiveCloud(input, level);
    } catch (const tf2::TransformException &ex) {
      RCLCPP_ERROR(nh_->get_logger(),
                   "Could not transform input cloud from %s to %s: %s.",
                   input->header.frame_id.c_str(), map_frame_.c_str(),
                   ex.what());
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
  rclcpp::Node::SharedPtr nh_;
  rclcpp::TimerBase::SharedPtr planning_timer_;

  std::shared_ptr<rclcpp::ParameterEventHandler> param_subscriber_;
  std::shared_ptr<rclcpp::ParameterCallbackHandle> cb_handle_;

  // Astar
  bool use_astar_{false};
  float astar_max_range_{50.};       // m; nodes further away than this are ignored
  float frontier_min_dist_{3.};    // m; frontiers closer than this are ignored
  int frontier_max_neighbors_{5};  // max num of neighbors to be considered a frontier 
  float max_relative_dist_to_goal_{2.0};

  // Transforms and frames
  std::shared_ptr<tf2_ros::Buffer> tf_{};
  std::shared_ptr<tf2_ros::TransformListener> tf_sub_;
  float tf_timeout_{3.0};
  std::string map_frame_{"map"};
  std::string robot_frame_{"base_footprint"};

  // Publishers
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr map_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr local_map_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr planning_freq_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr occ_grid_pub_;
  bool publish_occupancy_grid_{true};
  int occupancy_grid_w_{500};
  int occupancy_grid_h_{500};
  float occupancy_grid_resolution_{0.4};
  float max_total_cost_{2.};  // currently not used
  std::vector<double> cost_thresholds_;

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
  std::vector<int> cloud_levels_;
  float max_cloud_age_{5.0};
  float input_range_{10.0};

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

  // Sidelobes strategy
  float sidelobes_offset_distance_{1.0f};
  float sidelobes_radius_{0.5f};
  float sidelobes_cost_{10.0f};
  std::vector<double> sidelobes_angle_offsets_{-90.0f, -90.0f};
};

} // namespace grid
} // namespace naex
