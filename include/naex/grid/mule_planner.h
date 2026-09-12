#pragma once

#include "naex/grid/conversions.h"
#include "naex/grid/grid.h"
#include "naex/grid/path.h"
#include "naex/grid/search.h"
#include "naex/timer.h"
#include "naex/types.h"
#include <geometry_msgs/msg/point.hpp>
#include <mutex>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

namespace naex {
namespace grid {

/**
 * Distance below which the last resampled pose is taken to be the original
 * last pose already, so that it is not duplicated.
 */
inline constexpr float kResampleEndEpsilon = 1e-6f;

/// 2-D position of a ROS point, narrowed to the float the grid works in.
inline Point2f to_point2f(const geometry_msgs::msg::Point &p) {
  return Point2f(static_cast<float>(p.x), static_cast<float>(p.y));
}

class MulePlanner {
public:
  /// Depth of this node's publisher queues (replanned path, debug grid).
  /// Class-scope so that it cannot collide with the grid planner's own.
  static constexpr size_t kPublisherQueueDepth = 2;

  MulePlanner(rclcpp::Node::SharedPtr nh) : nh_(nh) {
    max_cloud_age_ =
        nh_->declare_parameter<float>("max_cloud_age", max_cloud_age_);
    max_ts_diff_ = nh_->declare_parameter<float>("max_ts_diff", max_ts_diff_);
    min_traversable_path_length_ = nh_->declare_parameter<float>(
        "min_traversable_path_length", min_traversable_path_length_);
    position_field_ =
        nh_->declare_parameter<std::string>("position_field", position_field_);
    cost_field_ =
        nh_->declare_parameter<std::string>("cost_field", cost_field_);
    astar_max_range_ =
        nh_->declare_parameter<float>("astar_max_range", astar_max_range_);
    obstacle_cost_threshold_ = nh_->declare_parameter<float>(
        "obstacle_cost_threshold", obstacle_cost_threshold_);
    max_start_to_traversable_dist_ = nh_->declare_parameter<float>(
        "max_start_to_traversable_dist", max_start_to_traversable_dist_);
    // Mirrors obstacle_cost_threshold_ in the struct the search takes.
    // Costs' ctor is explicit (B1), hence no braced initializer here.
    max_costs_ = Costs(obstacle_cost_threshold_);

    neighborhood_ = nh_->declare_parameter<int>("neighborhood", neighborhood_);

    path_sampling_dist_ = nh_->declare_parameter<float>("path_sampling_dist",
                                                        path_sampling_dist_);
    cell_size_ = nh_->declare_parameter<float>("cell_size", 1.0f);
    float forget_factor = nh_->declare_parameter<float>("forget_factor", 1.0f);
    std::vector<float> default_costs = {0.5f};
    default_costs_ = nh_->declare_parameter<std::vector<float>>("default_costs",
                                                                default_costs);
    grid_ = Grid(cell_size_, forget_factor, default_costs_);

    pcl_sub_ = nh_->create_subscription<sensor_msgs::msg::PointCloud2>(
        "points", rclcpp::SensorDataQoS(),
        std::bind(&MulePlanner::cloud_cb, this, std::placeholders::_1));
    path_sub_ = nh_->create_subscription<nav_msgs::msg::Path>(
        "path", rclcpp::SystemDefaultsQoS(),
        std::bind(&MulePlanner::path_cb, this, std::placeholders::_1));

    path_pub_ = nh_->create_publisher<nav_msgs::msg::Path>(
        "~/path", kPublisherQueueDepth);
    map_pub_ = nh_->create_publisher<sensor_msgs::msg::PointCloud2>(
        "planner_grid", kPublisherQueueDepth);
  }
  // --------------------------------------------------------
  // --------------------------------------------------------
  // --------------------------------------------------------

  void
  cloud_cb(const std::shared_ptr<const sensor_msgs::msg::PointCloud2> &input) {
    Timer t;

    // Process cloud into single scan grid.
    grid_.clear();
    const auto age = (nh_->get_clock()->now() - input->header.stamp).seconds();
    if (age > max_cloud_age_) {
      RCLCPP_WARN(nh_->get_logger(),
                  "Skipping old input cloud from %s, age %.1f s > %.1f s.",
                  input->header.frame_id.c_str(), age, max_cloud_age_);
      return;
    }

    sensor_msgs::PointCloud2ConstIterator<float> x_it(*input, position_field_);
    sensor_msgs::PointCloud2ConstIterator<float> cost_iter(*input, cost_field_);

    const size_t num_pts = static_cast<size_t>(input->height) * input->width;
    for (size_t i = 0; i < num_pts; ++i, ++x_it, ++cost_iter) {
      const Point2f p(x_it[0], x_it[1]);
      // A non-finite or out-of-int16 position is undefined behaviour in the
      // cast inside point_to_cell() and creates phantom cells (P6).
      if (std::isfinite(cost_iter[0]) && in_cell_range(grid_, p)) {
        grid_.update_point_cost(p, 0, cost_iter[0]);
      }
    }

    nav_msgs::msg::Path current_path;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (path_.poses.empty()) {
        RCLCPP_WARN(nh_->get_logger(), "No path yet.");
        return;
      }

      const auto ts_diff =
          std::fabs(rclcpp::Time(path_.header.stamp).seconds() -
                    rclcpp::Time(input->header.stamp).seconds());
      if (ts_diff > max_ts_diff_) {
        RCLCPP_WARN(nh_->get_logger(),
                    "The difference between path and cloud time stamps it too "
                    "large: path %s, cloud %s ... %.1f s > %.1f s.",
                    path_.header.frame_id.c_str(),
                    input->header.frame_id.c_str(), ts_diff, max_ts_diff_);
        return;
      }
      current_path = path_;
    }

    if (is_path_obstacle_free(current_path)) {
      auto republished_path = resample_path(current_path);
      republished_path.header.stamp = nh_->get_clock()->now();
      path_pub_->publish(republished_path);
      RCLCPP_INFO(
          nh_->get_logger(),
          "Original path is obstacle-free, republishing %lu poses (%.3f s).",
          republished_path.poses.size(), t.seconds_elapsed());
      return;
    }

    // Get start vertex.
    // Figure out the starting point for the search.
    geometry_msgs::msg::PoseStamped start;
    start.header.frame_id = input->header.frame_id;
    start.header.stamp = nh_->get_clock()->now();
    start.pose.position.x = 0.;
    start.pose.position.y = 0.;
    start.pose.position.z = 0.;
    start.pose.orientation.w = 1.;
    VertexId v0 = grid_.find_cell(grid_.point_to_cell({0., 0.}));

    if (v0 != INVALID_VERTEX_ID) {
      // Grid contains start cell.
      if (naex::grid::costs_in_bounds(grid_.costs(v0), max_costs_)) {
        // Start cell is traversable -> perfect, plan from there, do nothing.
        RCLCPP_INFO(nh_->get_logger(), "Planning from start position %s.",
                    format(to_vec3(grid_.point(v0))).c_str());
      } else {
        // Start cell is not traversable .
        RCLCPP_WARN(nh_->get_logger(), "Start position %s is not traversable.",
                    format(to_vec3(grid_.point(v0))).c_str());
        const auto [best_dist, best_v] = get_nearest_traversable_vertex(
            nh_->get_logger(), grid_, max_costs_, Vec3::Zero());
        if (best_v == INVALID_VERTEX_ID) {
          publish_empty_path(input->header.frame_id);
          return;
        }
        if (best_dist <= max_start_to_traversable_dist_) {
          // Nearest traversable point is near -> plan from this point instead.
          v0 = best_v;
          RCLCPP_INFO(nh_->get_logger(),
                      "Planning from nearest traversable point %s.",
                      format(to_vec3(grid_.point(v0))).c_str());
        } else {
          // Nearest traversable point is far -> fail to plan.
          RCLCPP_ERROR(nh_->get_logger(),
                       "Start point is further than "
                       "max_start_to_traversable_dist_ (%.3f > %.3f m) from "
                       "the closest traversable point %s.\nFailed to plan!",
                       best_dist, max_start_to_traversable_dist_,
                       format(to_vec3(grid_.point(best_v))).c_str());
          publish_empty_path(input->header.frame_id);
          return;
        }
      }
    } else {
      // Grid does not contain start cell.
      RCLCPP_WARN(nh_->get_logger(), "Start position (0., 0.) is unexplored.");
      const auto [best_dist, best_v] = get_nearest_traversable_vertex(
          nh_->get_logger(), grid_, max_costs_, Vec3::Zero());
      if (best_v == INVALID_VERTEX_ID) {
        publish_empty_path(input->header.frame_id);
        return;
      }
      if (best_dist <= max_start_to_traversable_dist_) {
        // Nearest traversable point is near -> plan from this point instead.
        v0 = best_v;
        RCLCPP_INFO(nh_->get_logger(),
                    "Planning from nearest traversable point %s.",
                    format(to_vec3(grid_.point(v0))).c_str());
      } else {
        // Nearest traversable point is far -> fail replanning.
        RCLCPP_WARN(
            nh_->get_logger(),
            "Start point is further than max_start_to_traversable_dist_ (%.3f "
            "> %.3f m) from the closest traversable point %s.\nFailed to plan!",
            best_dist, max_start_to_traversable_dist_,
            format(to_vec3(grid_.point(best_v))).c_str());
        publish_empty_path(input->header.frame_id);
        return;
      }
    }
    geometry_msgs::msg::Point last_path_point =
        current_path.poses.back().pose.position;
    Vec3 p1 = to_vec3(last_path_point);

    // Reused across clouds (P2): the search buffers keep their capacity.
    ShortestPaths &astar = shortest_paths_;
    astar.compute_astar(nh_->get_logger(), grid_, v0, p1, false,
                        astar_max_range_, static_cast<uint8_t>(neighborhood_),
                        max_costs_);

    create_and_publish_map_cloud(astar, input->header.frame_id);

    const auto [goal, goal_path_index] = choose_goal(current_path, astar);
    if (goal == INVALID_VERTEX_ID) {
      RCLCPP_WARN(nh_->get_logger(),
                  "No reachable goal found on path. Publishing empty path.");
      publish_empty_path(input->header.frame_id);
      return;
    }

    const auto path_vertices =
        trace_path_vertices(v0, goal, astar.predecessors());

    nav_msgs::msg::Path local_plan;
    local_plan.header.frame_id = input->header.frame_id;
    local_plan.header.stamp = nh_->get_clock()->now();
    local_plan.poses.push_back(start);
    append_path(path_vertices, grid_, local_plan);

    if (is_path_obstacle_free(current_path, goal_path_index + 1)) {
      append_path_suffix(current_path, goal_path_index + 1, local_plan);
      path_pub_->publish(resample_path(local_plan));
      RCLCPP_INFO(nh_->get_logger(),
                  "Published replanned prefix with original suffix, %lu poses "
                  "total (%.3f s).",
                  local_plan.poses.size(), t.seconds_elapsed());
      return;
    }

    const auto traversable_path_length = path_length(local_plan);
    if (traversable_path_length >= min_traversable_path_length_) {
      path_pub_->publish(resample_path(local_plan));
      RCLCPP_INFO(nh_->get_logger(),
                  "Published traversable prefix only, length %.3f m with %lu "
                  "poses (%.3f s).",
                  traversable_path_length, local_plan.poses.size(),
                  t.seconds_elapsed());
      return;
    }

    RCLCPP_WARN(nh_->get_logger(),
                "Traversable prefix is too short (%.3f m < %.3f m). Publishing "
                "empty path.",
                traversable_path_length, min_traversable_path_length_);
    publish_empty_path(input->header.frame_id);
  }

  // --------------------------------------------------------
  // --------------------------------------------------------
  // --------------------------------------------------------

  void path_cb(const std::shared_ptr<const nav_msgs::msg::Path> &input) {
    std::lock_guard<std::mutex> lock(mutex_);
    path_ = *input;
  }

  void create_and_publish_map_cloud(const ShortestPaths &sp,
                                    const std::string &frame_id) {
    // rviz-only topic; skip building 24 B per cell when nobody listens (P4).
    if (map_pub_->get_subscription_count() == 0) {
      return;
    }
    auto cloud = std::make_unique<sensor_msgs::msg::PointCloud2>();
    // The grid is built in the input cloud's own frame (see cloud_cb), not
    // the robot frame, so the debug cloud must carry that frame too.
    cloud->header.frame_id = frame_id;
    cloud->header.stamp = nh_->get_clock()->now();
    fill_map_cloud(*cloud, grid_, sp.path_costs(), sp.f_values());
    map_pub_->publish(std::move(cloud));
  }

  // --------------------------------------------------------
  // --------------------------------------------------------
  // --------------------------------------------------------
  // For now just return the furthest reachable point on the path.

  std::pair<VertexId, size_t> choose_goal(const nav_msgs::msg::Path &path,
                                          const ShortestPaths &astar) {
    VertexId goal{INVALID_VERTEX_ID};
    size_t goal_path_index = 0;

    for (size_t i = 0; i < path.poses.size(); ++i) {
      const auto &pose = path.poses[i];
      const auto c = grid_.point_to_cell(to_point2f(pose.pose.position));
      const VertexId v = grid_.find_cell(c);
      if (v != INVALID_VERTEX_ID && std::isfinite(astar.path_cost(v)) &&
          naex::grid::costs_in_bounds(grid_.costs(v), max_costs_) &&
          astar.visited()[v]) {
        // reachable
        goal = v;
        goal_path_index = i;
      }
    }

    RCLCPP_INFO(nh_->get_logger(), "Goal is %u at path index %zu.", goal,
                goal_path_index);
    return {goal, goal_path_index};
  }

  bool is_path_obstacle_free(const nav_msgs::msg::Path &path,
                             size_t start_index = 0) const {
    for (size_t i = start_index; i < path.poses.size(); ++i) {
      const auto &pose = path.poses[i];
      const auto cell = grid_.point_to_cell(to_point2f(pose.pose.position));
      const VertexId v = grid_.find_cell(cell);
      if (v == INVALID_VERTEX_ID) {
        continue;
      }
      if (!naex::grid::costs_in_bounds(grid_.costs(v), max_costs_)) {
        return false;
      }
    }
    return true;
  }

  void append_path_suffix(const nav_msgs::msg::Path &source, size_t start_index,
                          nav_msgs::msg::Path &target) const {
    for (size_t i = start_index; i < source.poses.size(); ++i) {
      auto pose = source.poses[i];
      pose.header.frame_id = target.header.frame_id;
      pose.header.stamp = target.header.stamp;
      target.poses.push_back(pose);
    }
  }

  float path_length(const nav_msgs::msg::Path &path) const {
    float length = 0.f;
    for (size_t i = 1; i < path.poses.size(); ++i) {
      const auto &p0 = path.poses[i - 1].pose.position;
      const auto &p1 = path.poses[i].pose.position;
      const auto dx = p1.x - p0.x;
      const auto dy = p1.y - p0.y;
      const auto dz = p1.z - p0.z;
      length += std::sqrt(dx * dx + dy * dy + dz * dz);
    }
    return length;
  }

  nav_msgs::msg::Path resample_path(const nav_msgs::msg::Path &path) const {
    if (path_sampling_dist_ <= 0.f || path.poses.size() < 2) {
      return path;
    }
    nav_msgs::msg::Path out;
    out.header = path.header;
    out.poses.push_back(path.poses.front());
    float carry = 0.f;
    for (size_t i = 1; i < path.poses.size(); ++i) {
      const auto &p0 = path.poses[i - 1].pose.position;
      const auto &p1 = path.poses[i].pose.position;
      const float dx = p1.x - p0.x;
      const float dy = p1.y - p0.y;
      const float dz = p1.z - p0.z;
      const float seg_len = std::sqrt(dx * dx + dy * dy + dz * dz);
      float d = path_sampling_dist_ - carry;
      while (d <= seg_len) {
        const float t = d / seg_len;
        geometry_msgs::msg::PoseStamped pose = path.poses[i];
        pose.pose.position.x = p0.x + t * dx;
        pose.pose.position.y = p0.y + t * dy;
        pose.pose.position.z = p0.z + t * dz;
        out.poses.push_back(pose);
        d += path_sampling_dist_;
      }
      carry = seg_len - (d - path_sampling_dist_);
    }
    const auto &last = path.poses.back().pose.position;
    const auto &prev = out.poses.back().pose.position;
    const float dist_to_last = std::sqrt((last.x - prev.x) * (last.x - prev.x) +
                                         (last.y - prev.y) * (last.y - prev.y) +
                                         (last.z - prev.z) * (last.z - prev.z));
    if (dist_to_last > kResampleEndEpsilon) {
      out.poses.push_back(path.poses.back());
    }
    return out;
  }

  void publish_empty_path(const std::string &frame_id) {
    nav_msgs::msg::Path empty_path;
    empty_path.header.frame_id = frame_id;
    empty_path.header.stamp = nh_->get_clock()->now();
    path_pub_->publish(empty_path);
  }

private:
  rclcpp::Node::SharedPtr nh_;

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pcl_sub_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;

  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr map_pub_;

  // Params
  float max_cloud_age_{0.5};
  std::string position_field_{"x"};
  std::string cost_field_{"traversability"};
  float astar_max_range_{50.}; // m; nodes further away than this are ignored
  int neighborhood_{8};
  float obstacle_cost_threshold_{0.7};
  Costs max_costs_;
  float max_ts_diff_{0.5};
  float min_traversable_path_length_{0.0};
  float max_start_to_traversable_dist_{5.};
  float path_sampling_dist_{
      0.f}; // 0 = disabled; >0 = resample path at this spacing (m)
  float cell_size_{1.0};
  Costs default_costs_;

  nav_msgs::msg::Path path_;
  Grid grid_{};
  /// Reused A* buffers (P2); see cloud_cb().
  ShortestPaths shortest_paths_;

  std::mutex mutex_;
};
} // namespace grid
} // namespace naex
