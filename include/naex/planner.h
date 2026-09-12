#pragma once

#include <flann/flann.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <limits>
#include <memory>
#include <mutex>
#include <naex/buffer.h>
#include <naex/graph.h>
#include <naex/map.h>
#include <naex/types.h>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/srv/get_plan.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <string>
#include <tf2_ros/buffer.hpp>
#include <tf2_ros/transform_listener.hpp>
#include <vector>

namespace naex {

/**
 * @brief Global planner on an incrementally built point map.
 *
 * The map (@ref Map) is merged from input clouds, labelled for traversability
 * and searched with Dijkstra over the k-NN graph. With a valid goal in the
 * request a path towards it is returned; otherwise the planner picks the goal
 * with the lowest relative cost (path cost over reward), i.e. it explores.
 */
class Planner {
public:
  typedef std::recursive_mutex Mutex;
  typedef std::lock_guard<Mutex> Lock;
  typedef nav_msgs::srv::GetPlan GetPlan;

  explicit Planner(rclcpp::Node::SharedPtr nh);

  /// Declare parameters and create the ROS interfaces.
  void configure();

  /**
   * Wait for the other robots and bootstrap the map.
   *
   * In ROS 1 this ran in the constructor. Here it is deferred to a one-shot
   * timer so that the executor is already spinning and time (notably
   * @c /clock under @c use_sim_time) is available to the TF buffer.
   */
  void initialize();

  /// Re-read the parameters which may be changed at runtime.
  void update_params();

  void bootstrap_map();

  Value time_from_init(const double time) const;
  Value time_from_init(const rclcpp::Time &time) const;

  void gather_viewpoints();

  void trace_path_indices(Vertex start, Vertex goal, const Vertex *predecessor,
                          std::vector<Vertex> &path_indices);

  void append_path(const std::vector<Vertex> &path_indices,
                   const std::vector<Point> &points, nav_msgs::msg::Path &path);

  Buffer<Elem> viewpoint_dist(const flann::Matrix<Elem> &points);
  Buffer<Elem> other_viewpoint_dist(const flann::Matrix<Elem> &points);

  void input_map_received(const sensor_msgs::msg::PointCloud2 &cloud);

  template <typename T>
  static bool valid_point(const T x, const T y, const T z) {
    return std::isfinite(x) && std::isfinite(y) && std::isfinite(z);
  }

  Value distance_reward(Value distance) const;

  bool plan(GetPlan::Request::SharedPtr req, GetPlan::Response::SharedPtr res);

  void cloud_received(
      const std::shared_ptr<const sensor_msgs::msg::PointCloud2> &cloud);

  void planning_timer_cb();

  std::vector<Value> find_robots(const std::string &frame,
                                 const rclcpp::Time &stamp, float timeout);

  void check_initialized();

  void send_cloud(
      const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr &pub,
      const rclcpp::Time &stamp, bool force = false);

  void send_map(const rclcpp::Time &stamp, bool force = false);

  template <typename C>
  void send_cloud(
      const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr &pub,
      const C &indices, const rclcpp::Time &stamp, bool force = false) {
    if (indices.empty())
      return;
    if (force || pub->get_subscription_count() > 0) {
      Timer t;
      sensor_msgs::msg::PointCloud2 cloud;
      cloud.header.frame_id = map_frame_;
      cloud.header.stamp =
          stamp.nanoseconds() == 0 ? nh_->get_clock()->now() : stamp;
      map_.create_cloud_msg(indices, cloud);
      pub->publish(cloud);
      RCLCPP_DEBUG(nh_->get_logger(), "Sending cloud %s: %.3f s.",
                   pub->get_topic_name(), t.seconds_elapsed());
    }
  }

  void send_dirty_cloud(const rclcpp::Time &stamp, bool force = false);
  void send_updated_cloud(const rclcpp::Time &stamp, bool force = false);
  void send_local_map(Value *origin, const rclcpp::Time &stamp,
                      bool force = false);

  void input_cloud_received(
      const std::shared_ptr<const sensor_msgs::msg::PointCloud2> &input);
  void input_cloud_received_safe(
      const std::shared_ptr<const sensor_msgs::msg::PointCloud2> &input);

protected:
  rclcpp::Node::SharedPtr nh_;

  std::shared_ptr<tf2_ros::Buffer> tf_{};
  std::shared_ptr<tf2_ros::TransformListener> tf_sub_;

  // All callbacks share one reentrant group, reproducing the ROS 1
  // MultiThreadedSpinner. The data are guarded by the mutexes below and by
  // Map's own mutexes.
  rclcpp::CallbackGroup::SharedPtr callback_group_;

  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr viewpoints_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr
      other_viewpoints_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr map_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr updated_map_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr dirty_map_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr map_diff_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr local_map_pub_;

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  std::vector<rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr>
      input_cloud_subs_;

  rclcpp::TimerBase::SharedPtr init_timer_;
  rclcpp::TimerBase::SharedPtr planning_timer_;
  rclcpp::TimerBase::SharedPtr viewpoints_update_timer_;
  rclcpp::TimerBase::SharedPtr update_params_timer_;

  rclcpp::Service<GetPlan>::SharedPtr get_plan_service_;
  Mutex last_request_mutex_;
  GetPlan::Request last_request_;

  std::string position_name_{"x"};
  std::string normal_name_{"normal_x"};

  std::string map_frame_{"map"};
  std::string robot_frame_{"base_footprint"};
  std::vector<std::string> robot_frames_{};
  float max_cloud_age_{5.0};
  float input_range_{10.0};

  int neighborhood_knn_{12};
  float neighborhood_radius_{0.5};
  float normal_radius_{neighborhood_radius_};

  Mutex viewpoints_mutex_;
  float viewpoints_update_freq_{1.0};
  std::vector<Vec3> viewpoints_{};
  std::vector<Vec3> other_viewpoints_{};
  float min_vp_distance_{1.5};
  float max_vp_distance_{6.0};
  bool collect_rewards_{true};
  float full_coverage_dist_{3.0};
  float coverage_dist_spread_{1.5};
  float self_factor_{0.25};
  bool suppress_base_reward_{true};
  float path_cost_pow_{1.0};
  float min_path_cost_{0.0};
  // Re-planning frequency, repeating the last request if positive.
  float planning_freq_{0.5};
  // Randomize starting vertex within tolerance radius.
  bool random_start_{false};
  double plan_from_goal_dist_{0.0};
  geometry_msgs::msg::PoseStamped last_start_{};
  geometry_msgs::msg::PoseStamped last_goal_{};
  // Z offset for bootstrap map height
  float bootstrap_z_{0.0};
  Mutex initialized_mutex_;
  bool initialized_{false};
  double time_initialized_{std::numeric_limits<double>::quiet_NaN()};

  int queue_size_{5};
  Mutex map_mutex_;
  Map map_{};
};

} // namespace naex
