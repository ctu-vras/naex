#pragma once
#include <geometry_msgs/msg/point.hpp>
#include <nav_msgs/msg/path.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <rclcpp/rclcpp.hpp>
#include "naex/types.h"
#include "naex/grid/grid.h"
#include "naex/grid/search.h"
#include "naex/grid/planner.h"

namespace naex {
namespace grid {
class MulePlanner {
  public:
    MulePlanner(rclcpp::Node::SharedPtr nh) : nh_(nh) {
      max_cloud_age_ = nh_->declare_parameter<float>("max_cloud_age", max_cloud_age_);
      max_ts_diff_ = nh_->declare_parameter<float>("max_ts_diff", max_ts_diff_);
      min_traversable_path_length_ = nh_->declare_parameter<float>("min_traversable_path_length", min_traversable_path_length_);
      position_field_ = nh_->declare_parameter<std::string>("position_field", position_field_);
      cost_field_ = nh_->declare_parameter<std::string>("cost_field", cost_field_);
      robot_frame_ = nh_->declare_parameter<std::string>("robot_frame", robot_frame_);
      astar_max_range_ = nh_->declare_parameter<float>("astar_max_range", astar_max_range_);
      obstacle_cost_threshold_ = nh_->declare_parameter<float>("obstacle_cost_threshold", obstacle_cost_threshold_);
      max_start_to_traversable_dist_ = nh_->declare_parameter<float>("max_start_to_traversable_dist", max_start_to_traversable_dist_);
      max_costs_ = {obstacle_cost_threshold_}; // mirrors obstacle_cost_threshold_ but in different struct, just a formality

      neighborhood_ = nh_->declare_parameter<int>("neighborhood", neighborhood_);

      path_sampling_dist_ = nh_->declare_parameter<float>("path_sampling_dist", path_sampling_dist_);
      cell_size_ = nh_->declare_parameter<float>("cell_size", 1.0f);
      float forget_factor = nh_->declare_parameter<float>("forget_factor", 1.0f);
      std::vector<float> default_costs = {0.5f};
      default_costs_ = nh_->declare_parameter<std::vector<float>>("default_costs", default_costs);
      grid_ = Grid(cell_size_, forget_factor, default_costs_);

      pcl_sub_ = nh_->create_subscription<sensor_msgs::msg::PointCloud2>("points", rclcpp::SensorDataQoS(),
                  std::bind(&MulePlanner::cloudCb, this, std::placeholders::_1));
      path_sub_ = nh_->create_subscription<nav_msgs::msg::Path>("path", rclcpp::SystemDefaultsQoS(),
                  std::bind(&MulePlanner::pathCb, this, std::placeholders::_1));

      path_pub_ = nh_->create_publisher<nav_msgs::msg::Path>("~/path", 2);
      map_pub_ = nh_->create_publisher<sensor_msgs::msg::PointCloud2>("planner_grid", 2);
    }
    // --------------------------------------------------------
    // --------------------------------------------------------
    // --------------------------------------------------------

    void cloudCb(
        const std::shared_ptr<const sensor_msgs::msg::PointCloud2> &input
      ) {
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

      for (int i = 0; i < input->height * input->width; ++i, ++x_it, ++cost_iter) {
        Vec3 p(x_it[0], x_it[1], x_it[2]);
        if (std::isfinite(cost_iter[0])) {
          grid_.updatePointCost({p.x(), p.y()}, 0, cost_iter[0]);
        }
      }

      nav_msgs::msg::Path current_path;
      {
        std::lock_guard<std::mutex> lock(mutex_);
        if (path_.poses.empty()) {
          RCLCPP_WARN(nh_->get_logger(), "No path yet.");
          return;
        }

        const auto ts_diff = std::fabs(rclcpp::Time(path_.header.stamp).seconds() -
                                       rclcpp::Time(input->header.stamp).seconds());
        if (ts_diff > max_ts_diff_) {
          RCLCPP_WARN(
              nh_->get_logger(),
              "The difference between path and cloud time stamps it too large: path %s, cloud %s ... %.1f s > %.1f s.",
              path_.header.frame_id.c_str(), input->header.frame_id.c_str(), ts_diff,
              max_ts_diff_);
          return;
        }
        current_path = path_;
      }

      if (isPathObstacleFree(current_path)) {
        auto republished_path = resamplePath(current_path);
        republished_path.header.stamp = nh_->get_clock()->now();
        path_pub_->publish(republished_path);
        RCLCPP_INFO(nh_->get_logger(),
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
      VertexId v0 = INVALID_VERTEX;

      if (grid_.hasCell(grid_.pointToCell({0., 0.}))) {
        // Grid contains start cell.
        v0 = grid_.cellId(grid_.pointToCell({0., 0.}));
        if (naex::grid::costsInBounds(grid_.costs(v0), max_costs_)) {
          // Start cell is traversable -> perfect, plan from there, do nothing.
          RCLCPP_INFO(nh_->get_logger(), "Planning from start position %s.",
                      format(toVec3(grid_.point(v0))).c_str());
        } else {
          // Start cell is not traversable .
          RCLCPP_WARN(nh_->get_logger(), "Start position %s is not traversable.",
                      format(toVec3(grid_.point(v0))).c_str());
            std::pair<float, VertexId> nearest_traversable = getNearestTraversableVertex();
            float best_dist = nearest_traversable.first;
            VertexId best_v = nearest_traversable.second;
          if (best_v == INVALID_VERTEX) {
            publishEmptyPath(input->header.frame_id);
            return;
          }
          if (best_dist <= max_start_to_traversable_dist_) {
            // Nearest traversable point is near -> plan from this point instead.
            v0 = best_v;
            RCLCPP_INFO(nh_->get_logger(), "Planning from nearest traversable point %s.",
                      format(toVec3(grid_.point(v0))).c_str());
          } else {
            // Nearest traversable point is far -> fail to plan.
            RCLCPP_ERROR(nh_->get_logger(),
              "Start point is further than max_start_to_traversable_dist_ (%.3f > %.3f m) from the closest traversable point %s.\nFailed to plan!",
              best_dist, max_start_to_traversable_dist_, format(toVec3(grid_.point(best_v))).c_str());
              publishEmptyPath(input->header.frame_id);
              return;
          }
        }
      } else {
        // Grid does not contain start cell.
        RCLCPP_WARN(nh_->get_logger(), "Start position (0., 0.) is unexplored.");
        std::pair<float, VertexId> nearest_traversable = getNearestTraversableVertex();
        float best_dist = nearest_traversable.first;
        VertexId best_v = nearest_traversable.second;
        if (best_v == INVALID_VERTEX) {
            publishEmptyPath(input->header.frame_id);
            return;
        }
        if (best_dist <= max_start_to_traversable_dist_) {
          // Nearest traversable point is near -> plan from this point instead.
          v0 = best_v;
          RCLCPP_INFO(nh_->get_logger(), "Planning from nearest traversable point %s.",
                      format(toVec3(grid_.point(v0))).c_str());
        } else {
          // Nearest traversable point is far -> fail replanning.
          RCLCPP_WARN(nh_->get_logger(),
            "Start point is further than max_start_to_traversable_dist_ (%.3f > %.3f m) from the closest traversable point %s.\nFailed to plan!",
            best_dist, max_start_to_traversable_dist_, format(toVec3(grid_.point(best_v))).c_str());
          publishEmptyPath(input->header.frame_id);
          return;
        }
      }
      geometry_msgs::msg::Point last_path_point = current_path.poses.back().pose.position;
      Vec3 p1 = toVec3(last_path_point);

      auto astar = std::make_shared<AstarShortestPaths>(nh_, grid_, v0, p1, false, astar_max_range_, neighborhood_, obstacle_cost_threshold_);

      createAndPublishMapCloud(astar);

      const auto [goal, goal_path_index] = chooseGoal(current_path, astar);
      if (goal == INVALID_VERTEX) {
        RCLCPP_WARN(nh_->get_logger(), "No reachable goal found on path. Publishing empty path.");
        publishEmptyPath(input->header.frame_id);
        return;
      }

      std::vector<VertexId> path_vertices;
      assert(astar->predecessors()[v0] == v0);
      Vertex v = goal;
      while (v != v0) {
        path_vertices.push_back(v);
        if (v == astar->predecessors()[v]) {
          RCLCPP_ERROR(nh_->get_logger(), "Encountered cyclic predecessor while tracing path.");
          break;
        }
        v = astar->predecessors()[v];
      }
      path_vertices.push_back(v);
      std::reverse(path_vertices.begin(), path_vertices.end());

      if (path_vertices.empty()) {
        RCLCPP_WARN(nh_->get_logger(), "Planned path is empty. Publishing empty path.");
        publishEmptyPath(input->header.frame_id);
        return;
      }

      nav_msgs::msg::Path local_plan;
      local_plan.header.frame_id = input->header.frame_id;
      local_plan.header.stamp = nh_->get_clock()->now();
      local_plan.poses.push_back(start);
      appendPath(path_vertices, grid_, local_plan);

      if (local_plan.poses.empty()) {
        RCLCPP_WARN(nh_->get_logger(), "Planned path is empty. Publishing empty path.");
        publishEmptyPath(input->header.frame_id);
        return;
      }

      if (isPathObstacleFree(current_path, goal_path_index + 1)) {
        appendPathSuffix(current_path, goal_path_index + 1, local_plan);
        path_pub_->publish(resamplePath(local_plan));
        RCLCPP_INFO(nh_->get_logger(),
                    "Published replanned prefix with original suffix, %lu poses total (%.3f s).",
                    local_plan.poses.size(), t.seconds_elapsed());
        return;
      }

      const auto traversable_path_length = pathLength(local_plan);
      if (traversable_path_length >= min_traversable_path_length_) {
        path_pub_->publish(resamplePath(local_plan));
        RCLCPP_INFO(nh_->get_logger(),
                    "Published traversable prefix only, length %.3f m with %lu poses (%.3f s).",
                    traversable_path_length, local_plan.poses.size(), t.seconds_elapsed());
        return;
      }

      RCLCPP_WARN(
          nh_->get_logger(),
          "Traversable prefix is too short (%.3f m < %.3f m). Publishing empty path.",
          traversable_path_length, min_traversable_path_length_);
      publishEmptyPath(input->header.frame_id);
    }

    // --------------------------------------------------------
    // --------------------------------------------------------
    // --------------------------------------------------------

    void pathCb(
        const std::shared_ptr<const nav_msgs::msg::Path> &input
      ) {
      std::lock_guard<std::mutex> lock(mutex_);
      path_ = *input;
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

    void createAndPublishMapCloud(const std::shared_ptr<ShortestPaths> sp) {
      sensor_msgs::msg::PointCloud2 cloud;
      cloud.header.frame_id = robot_frame_;
      cloud.header.stamp = nh_->get_clock()->now();
      fillMapCloud(cloud, grid_, sp->pathCosts(), sp->fValues());
      map_pub_->publish(cloud);
    }

    // --------------------------------------------------------
    // --------------------------------------------------------
    // --------------------------------------------------------
    // For now just return the furthest reachable point on the path.

    std::pair<VertexId, size_t>
    chooseGoal(const nav_msgs::msg::Path &path,
               const std::shared_ptr<AstarShortestPaths> &astar) {
      VertexId goal{INVALID_VERTEX};
      size_t goal_path_index = 0;

      for (size_t i = 0; i < path.poses.size(); ++i) {
        const auto &pose = path.poses[i];
        auto c = grid_.pointToCell({pose.pose.position.x, pose.pose.position.y});
        if (grid_.hasCell(c)) {
          VertexId v = grid_.cellId(c);
          if (std::isfinite(astar->pathCost(v)) &&
              naex::grid::costsInBounds(grid_.costs(v), max_costs_) &&
              astar->visited()[v]) {
            // reachable
            goal = v;
            goal_path_index = i;
          }
        }
      }

      RCLCPP_INFO(nh_->get_logger(), "Goal is %u at path index %zu.", goal,
                  goal_path_index);
      return {goal, goal_path_index};
    }

    bool isPathObstacleFree(const nav_msgs::msg::Path &path,
                            size_t start_index = 0) const {
      for (size_t i = start_index; i < path.poses.size(); ++i) {
        const auto &pose = path.poses[i];
        auto cell = grid_.pointToCell(
            {pose.pose.position.x, pose.pose.position.y});
        if (!grid_.hasCell(cell)) {
          continue;
        }
        if (!naex::grid::costsInBounds(grid_.costs(grid_.cellId(cell)),
                                       max_costs_)) {
          return false;
        }
      }
      return true;
    }

    void appendPathSuffix(const nav_msgs::msg::Path &source, size_t start_index,
                          nav_msgs::msg::Path &target) const {
      for (size_t i = start_index; i < source.poses.size(); ++i) {
        auto pose = source.poses[i];
        pose.header.frame_id = target.header.frame_id;
        pose.header.stamp = target.header.stamp;
        target.poses.push_back(pose);
      }
    }

    float pathLength(const nav_msgs::msg::Path &path) const {
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

    nav_msgs::msg::Path resamplePath(const nav_msgs::msg::Path &path) const {
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
      const float dist_to_last = std::sqrt(
          (last.x - prev.x) * (last.x - prev.x) +
          (last.y - prev.y) * (last.y - prev.y) +
          (last.z - prev.z) * (last.z - prev.z));
      if (dist_to_last > 1e-6f) {
        out.poses.push_back(path.poses.back());
      }
      return out;
    }

    void publishEmptyPath(const std::string &frame_id) {
      nav_msgs::msg::Path empty_path;
      empty_path.header.frame_id = frame_id;
      empty_path.header.stamp = nh_->get_clock()->now();
      path_pub_->publish(empty_path);
    }

    std::pair<float,VertexId> getNearestTraversableVertex() {
      // Use the nearest traversable point to robot as the starting point.
      float best_dist = std::numeric_limits<float>::infinity();
      VertexId best_v = INVALID_VERTEX;
      for (VertexId v = 0; v < grid_.size(); ++v) {
        if (!naex::grid::costsInBounds(grid_.costs(v), max_costs_)) {
          continue;
        }

        Value dist = toVec3(grid_.point(v)).norm();
        if (dist < best_dist) {
          best_v = v;
          best_dist = dist;
        }
      }
      if (best_v != INVALID_VERTEX) {
        RCLCPP_INFO(nh_->get_logger(),
        "Closest traversable point to start: %s (dist %.3f).",
        format(toVec3(grid_.point(best_v))).c_str(), best_dist);
      } else {
        RCLCPP_ERROR(nh_->get_logger(), "No traversable points in graph!");
      }
      return std::make_pair(best_dist, best_v);
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
    std::string robot_frame_{"os_sensor"};
    float astar_max_range_{50.};       // m; nodes further away than this are ignored
    int neighborhood_{8};
    float obstacle_cost_threshold_{0.7};
    Costs max_costs_;
    float max_ts_diff_{0.5};
    float min_traversable_path_length_{0.0};
    float max_start_to_traversable_dist_{5.};
    float path_sampling_dist_{0.f};  // 0 = disabled; >0 = resample path at this spacing (m)
    float cell_size_{1.0};
    Costs default_costs_;

    nav_msgs::msg::Path path_;
    Grid grid_{};

    std::mutex mutex_;
};
} // namespace grid
} // namespace naex
