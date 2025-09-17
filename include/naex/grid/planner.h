#pragma once

#include "naex/clouds.h"
#include "naex/grid/graph.h"
#include "naex/grid/grid.h"
#include "naex/grid/search.h"
#include "naex/iterators.h"
#include "naex/timer.h"
#include "naex/transforms.h"
#include "naex/types.h"
#include <functional>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/srv/get_plan.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <std_msgs/msg/float32.hpp>
#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <unordered_map>

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

    std::vector<float> max_costs;
    std::vector<float> default_costs;
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

    tf_ = std::make_shared<tf2_ros::Buffer>(nh_->get_clock());
    tf_sub_ = std::make_shared<tf2_ros::TransformListener>(*tf_);

    map_pub_ = nh_->create_publisher<sensor_msgs::msg::PointCloud2>("map", 2);
    local_map_pub_ =
        nh_->create_publisher<sensor_msgs::msg::PointCloud2>("local_map", 2);
    path_pub_ = nh_->create_publisher<nav_msgs::msg::Path>("path", 2);
    planning_freq_pub_ =
        nh_->create_publisher<std_msgs::msg::Float32>("planning_freq", 2);

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

    ShortestPaths sp(grid_, v0, neighborhood_, max_costs_);
    RCLCPP_INFO(nh_->get_logger(), "Dijkstra (%lu pts): %.3f s.", grid_.size(),
                t_part.seconds_elapsed());
    createAndPublishMapCloud(sp);

    // If planning for a given goal, return path to the closest reachable
    // point from the goal.
    t_part.reset();
    if (isValid(req->goal.pose.position)) {
      Vec3 p1 = toVec3(req->goal.pose.position);
      p1.z() = 0.f;

      VertexId v1 = INVALID_VERTEX;
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
      if (v1 == INVALID_VERTEX) {
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
    resize_cloud(cloud, 1, grid_.size());

    sensor_msgs::PointCloud2Iterator<float> x_it(cloud, "x");
    sensor_msgs::PointCloud2Iterator<float> cost_it(cloud, "cost");
    sensor_msgs::PointCloud2Iterator<float> path_cost_it(cloud, "path_cost");
    for (VertexId v = 0; v < grid_.size();
         ++v, ++x_it, ++cost_it, ++path_cost_it) {
      const auto p = grid_.point(v);
      x_it[0] = p.x;
      x_it[1] = p.y;
      x_it[2] = 0.f;
      cost_it[0] = grid_.costs(v).total();
      path_cost_it[0] = path_costs[v];
    }
  }

  void createAndPublishMapCloud(const ShortestPaths &sp) {
    sensor_msgs::msg::PointCloud2 cloud;
    cloud.header.frame_id = map_frame_;
    cloud.header.stamp = nh_->get_clock()->now();
    fillMapCloud(cloud, grid_, sp.pathCosts());
    map_pub_->publish(cloud);
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

    const uint8_t level = i < cloud_levels_.size() ? cloud_levels_[i] : i;
    const float weight = i < cloud_weights_.size() ? cloud_weights_[i] : 1.0;
    const std::string cost_field =
        i < cost_fields_.size() ? cost_fields_[i] : "cost";
    sensor_msgs::PointCloud2ConstIterator<float> x_it(*input, position_field_);
    sensor_msgs::PointCloud2ConstIterator<float> cost_it(*input, cost_field);

    for (int i = 0; i < input->height * input->width; ++i, ++x_it, ++cost_it) {
      Vec3 p(x_it[0], x_it[1], x_it[2]);
      p = transform * p;
      grid_.updatePointCost({p.x(), p.y()}, level, weight * cost_it[0]);
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

  // Subscribers
  std::vector<rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr>
      input_cloud_subs_;

  // Services
  rclcpp::Service<nav_msgs::srv::GetPlan>::SharedPtr get_plan_service_;
  nav_msgs::srv::GetPlan::Request::SharedPtr last_request_;

  // Input
  std::string position_field_{"x"};
  std::vector<std::string> cost_fields_;
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
};

} // namespace grid
} // namespace naex
