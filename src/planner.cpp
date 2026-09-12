#include <naex/planner.h>

#include <naex/clouds.h>
#include <naex/exceptions.h>
#include <naex/exclude_frames_filter.h>
#include <naex/filter.h>
#include <naex/flann.h>
#include <naex/graph.h>
#include <naex/iterators.h>
#include <naex/nearest_neighbors.h>
#include <naex/range_filter.h>
#include <naex/reward.h>
#include <naex/step_filter.h>
#include <naex/timer.h>
#include <naex/transform_filter.h>
#include <naex/transforms.h>
#include <naex/types.h>
#include <naex/voxel_filter.h>

#include <algorithm>
#include <boost/property_map/property_map.hpp>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <functional>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <limits>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <sstream>
#include <tf2/exceptions.hpp>
#include <tf2/time.hpp>
#include <tf2_eigen/tf2_eigen.hpp>

namespace naex {
namespace {

/// Create a dense xyz cloud from a row-major N-by-3 matrix.
void create_xyz_cloud(const flann::Matrix<Elem> &points,
                      sensor_msgs::msg::PointCloud2 &cloud) {
  cloud.is_bigendian = bigendian();
  cloud.is_dense = true;
  cloud.point_step = 0;
  cloud.fields.clear();
  append_field<Elem>("x", 1, cloud);
  append_field<Elem>("y", 1, cloud);
  append_field<Elem>("z", 1, cloud);
  resize_cloud(cloud, 1, uint32_t(points.rows));
  if (points.rows == 0) {
    return;
  }
  sensor_msgs::PointCloud2Iterator<Elem> it(cloud, "x");
  for (size_t i = 0; i < points.rows; ++i, ++it) {
    it[0] = points[i][0];
    it[1] = points[i][1];
    it[2] = points[i][2];
  }
}

/// Duration of one period of the given frequency, for create_wall_timer.
std::chrono::nanoseconds period_from_freq(double freq) {
  return std::chrono::nanoseconds(int64_t(1e9 / freq));
}

} // namespace

Planner::Planner(rclcpp::Node::SharedPtr nh) : nh_(nh) {
  // Invalid position invokes exploration mode.
  last_request_.start.pose.position.x =
      std::numeric_limits<double>::quiet_NaN();
  last_request_.start.pose.position.y =
      std::numeric_limits<double>::quiet_NaN();
  last_request_.start.pose.position.z =
      std::numeric_limits<double>::quiet_NaN();
  last_request_.goal.pose.position.x = std::numeric_limits<double>::quiet_NaN();
  last_request_.goal.pose.position.y = std::numeric_limits<double>::quiet_NaN();
  last_request_.goal.pose.position.z = std::numeric_limits<double>::quiet_NaN();
  last_request_.tolerance = 2.0f;
  configure();
}

void Planner::configure() {
  callback_group_ =
      nh_->create_callback_group(rclcpp::CallbackGroupType::Reentrant);

  position_name_ =
      nh_->declare_parameter<std::string>("position_name", position_name_);
  normal_name_ =
      nh_->declare_parameter<std::string>("normal_name", normal_name_);
  map_frame_ = nh_->declare_parameter<std::string>("map_frame", map_frame_);
  robot_frame_ =
      nh_->declare_parameter<std::string>("robot_frame", robot_frame_);
  // NB: ROS 1 took a {key: frame} dictionary here, which ROS 2 parameters
  // cannot express. Only the frames are used, so this is a plain list now.
  robot_frames_ = nh_->declare_parameter<std::vector<std::string>>(
      "robot_frames", robot_frames_);

  max_cloud_age_ =
      nh_->declare_parameter<float>("max_cloud_age", max_cloud_age_);
  input_range_ = nh_->declare_parameter<float>("input_range", input_range_);
  map_.max_pitch_ = nh_->declare_parameter<float>("max_pitch", map_.max_pitch_);
  map_.max_roll_ = nh_->declare_parameter<float>("max_roll", map_.max_roll_);
  map_.inclination_penalty_ = nh_->declare_parameter<float>(
      "inclination_penalty", map_.inclination_penalty_);

  map_.neighborhood_radius_ = nh_->declare_parameter<float>(
      "neighborhood_radius", map_.neighborhood_radius_);
  normal_radius_ =
      nh_->declare_parameter<float>("normal_radius", normal_radius_);

  // Parameters which may be updated at runtime, see update_params().
  nh_->declare_parameter<float>("clearance_radius", map_.clearance_radius_);
  nh_->declare_parameter<float>("clearance_low", map_.clearance_low_);
  nh_->declare_parameter<float>("clearance_high", map_.clearance_high_);
  nh_->declare_parameter<float>("min_points_obstacle",
                                map_.min_points_obstacle_);
  nh_->declare_parameter<float>("max_ground_diff_std", map_.max_ground_diff_std_);
  nh_->declare_parameter<float>("max_mean_abs_ground_diff",
                                map_.max_mean_abs_ground_diff_);
  nh_->declare_parameter<float>("edge_min_centroid_offset",
                                map_.edge_min_centroid_offset_);
  nh_->declare_parameter<float>("min_dist_to_obstacle",
                                map_.min_dist_to_obstacle_);
  update_params();

  viewpoints_update_freq_ = nh_->declare_parameter<float>(
      "viewpoints_update_freq", viewpoints_update_freq_);
  min_vp_distance_ =
      nh_->declare_parameter<float>("min_vp_distance", min_vp_distance_);
  max_vp_distance_ =
      nh_->declare_parameter<float>("max_vp_distance", max_vp_distance_);
  collect_rewards_ =
      nh_->declare_parameter<bool>("collect_rewards", collect_rewards_);
  full_coverage_dist_ =
      nh_->declare_parameter<float>("full_coverage_dist", full_coverage_dist_);
  coverage_dist_spread_ = nh_->declare_parameter<float>("coverage_dist_spread",
                                                        coverage_dist_spread_);

  self_factor_ = nh_->declare_parameter<float>("self_factor", self_factor_);
  suppress_base_reward_ = nh_->declare_parameter<bool>("suppress_base_reward",
                                                       suppress_base_reward_);
  path_cost_pow_ =
      nh_->declare_parameter<float>("path_cost_pow", path_cost_pow_);
  min_path_cost_ =
      nh_->declare_parameter<float>("min_path_cost", min_path_cost_);
  planning_freq_ =
      nh_->declare_parameter<float>("planning_freq", planning_freq_);
  random_start_ = nh_->declare_parameter<bool>("random_start", random_start_);
  plan_from_goal_dist_ = nh_->declare_parameter<double>("plan_from_goal_dist",
                                                        plan_from_goal_dist_);
  bootstrap_z_ = nh_->declare_parameter<float>("bootstrap_z", bootstrap_z_);

  int num_input_clouds = nh_->declare_parameter<int>("num_input_clouds", 1);
  num_input_clouds = std::max(1, num_input_clouds);
  queue_size_ = nh_->declare_parameter<int>("input_queue_size", queue_size_);
  queue_size_ = std::max(1, queue_size_);
  map_.points_min_dist_ =
      nh_->declare_parameter<float>("points_min_dist", map_.points_min_dist_);

  map_.min_empty_cos_ =
      nh_->declare_parameter<float>("min_empty_cos", map_.min_empty_cos_);
  map_.min_num_empty_ =
      nh_->declare_parameter<int>("min_num_empty", map_.min_num_empty_);
  map_.min_empty_ratio_ =
      nh_->declare_parameter<float>("min_empty_ratio", map_.min_empty_ratio_);
  map_.max_occ_counter_ =
      nh_->declare_parameter<int>("max_occ_counter", map_.max_occ_counter_);

  filter_robots_ = nh_->declare_parameter<bool>("filter_robots", filter_robots_);

  const bool among_robots =
      std::find(robot_frames_.begin(), robot_frames_.end(), robot_frame_) !=
      robot_frames_.end();
  if (!among_robots) {
    RCLCPP_INFO(nh_->get_logger(), "Adding robot frame %s to robot frames.",
                robot_frame_.c_str());
    robot_frames_.push_back(robot_frame_);
  }
  for (const auto &f : robot_frames_) {
    RCLCPP_INFO(nh_->get_logger(), "Robot frame: %s", f.c_str());
  }

  viewpoints_.reserve(size_t(7200. * viewpoints_update_freq_) * 3);
  other_viewpoints_.reserve(size_t(7200. * viewpoints_update_freq_) * 3 *
                            robot_frames_.size());

  tf_ = std::make_shared<tf2_ros::Buffer>(nh_->get_clock(),
                                          tf2::durationFromSec(30.0));
  tf_sub_ = std::make_shared<tf2_ros::TransformListener>(*tf_);

  viewpoints_pub_ =
      nh_->create_publisher<sensor_msgs::msg::PointCloud2>("viewpoints", 5);
  other_viewpoints_pub_ = nh_->create_publisher<sensor_msgs::msg::PointCloud2>(
      "other_viewpoints", 5);
  map_pub_ = nh_->create_publisher<sensor_msgs::msg::PointCloud2>("map", 5);
  updated_map_pub_ =
      nh_->create_publisher<sensor_msgs::msg::PointCloud2>("updated_map", 5);
  dirty_map_pub_ =
      nh_->create_publisher<sensor_msgs::msg::PointCloud2>("dirty_map", 5);
  map_diff_pub_ =
      nh_->create_publisher<sensor_msgs::msg::PointCloud2>("map_diff", 5);
  local_map_pub_ =
      nh_->create_publisher<sensor_msgs::msg::PointCloud2>("local_map", 5);
  path_pub_ = nh_->create_publisher<nav_msgs::msg::Path>("path", 5);

  rclcpp::SubscriptionOptions sub_opts;
  sub_opts.callback_group = callback_group_;

  cloud_sub_ = nh_->create_subscription<sensor_msgs::msg::PointCloud2>(
      "input_map", queue_size_,
      [this](const std::shared_ptr<const sensor_msgs::msg::PointCloud2> &msg) {
        this->cloud_received(msg);
      },
      sub_opts);

  for (int i = 0; i < num_input_clouds; ++i) {
    std::stringstream ss;
    ss << "input_cloud_" << i;
    input_cloud_subs_.push_back(
        nh_->create_subscription<sensor_msgs::msg::PointCloud2>(
            ss.str(), queue_size_,
            [this](
                const std::shared_ptr<const sensor_msgs::msg::PointCloud2> &msg) {
              this->input_cloud_received_safe(msg);
            },
            sub_opts));
  }

  if (viewpoints_update_freq_ > 0.f) {
    viewpoints_update_timer_ = nh_->create_wall_timer(
        period_from_freq(viewpoints_update_freq_),
        [this]() { this->gather_viewpoints(); }, callback_group_);
  } else {
    RCLCPP_WARN(nh_->get_logger(),
                "Don't gather viewpoints (invalid frequency %.3f).",
                viewpoints_update_freq_);
  }

  if (planning_freq_ > 0.f) {
    planning_timer_ =
        nh_->create_wall_timer(period_from_freq(planning_freq_),
                               [this]() { this->planning_timer_cb(); },
                               callback_group_);
    RCLCPP_INFO(nh_->get_logger(),
                "Re-plan automatically at %.1f Hz using the last request.",
                planning_freq_);
  } else {
    RCLCPP_WARN(nh_->get_logger(),
                "Don't re-plan automatically using the last request.");
  }

  update_params_timer_ = nh_->create_wall_timer(
      std::chrono::seconds(2), [this]() { this->update_params(); },
      callback_group_);

  get_plan_service_ = nh_->create_service<GetPlan>(
      "get_plan",
      [this](const GetPlan::Request::SharedPtr req,
             GetPlan::Response::SharedPtr res) { this->plan(req, res); },
      rclcpp::ServicesQoS(), callback_group_);

  // Waiting for other robots and bootstrapping the map needs the clock and
  // the TF buffer, i.e. a spinning executor. Do it from a one-shot timer.
  init_timer_ = nh_->create_wall_timer(std::chrono::milliseconds(1),
                                       [this]() {
                                         init_timer_->cancel();
                                         this->initialize();
                                       },
                                       callback_group_);
}

void Planner::initialize() {
  Timer t;
  RCLCPP_INFO(nh_->get_logger(), "Initializing. Waiting for other robots...");
  find_robots(map_frame_, rclcpp::Time(0, 0, nh_->get_clock()->get_clock_type()),
              15.f);
  Lock lock(initialized_mutex_);
  initialized_ = true;
  time_initialized_ = nh_->get_clock()->now().seconds();
  bootstrap_map();
  RCLCPP_INFO(nh_->get_logger(), "Initialized at %.1f s (%.3f s).",
              time_initialized_, t.seconds_elapsed());
}

void Planner::update_params() {
  Timer t;
  map_.clearance_radius_ =
      float(nh_->get_parameter("clearance_radius").as_double());
  map_.clearance_low_ = float(nh_->get_parameter("clearance_low").as_double());
  map_.clearance_high_ = float(nh_->get_parameter("clearance_high").as_double());
  map_.min_points_obstacle_ =
      float(nh_->get_parameter("min_points_obstacle").as_double());
  map_.max_ground_diff_std_ =
      float(nh_->get_parameter("max_ground_diff_std").as_double());
  map_.max_mean_abs_ground_diff_ =
      float(nh_->get_parameter("max_mean_abs_ground_diff").as_double());
  map_.edge_min_centroid_offset_ =
      float(nh_->get_parameter("edge_min_centroid_offset").as_double());
  map_.min_dist_to_obstacle_ =
      float(nh_->get_parameter("min_dist_to_obstacle").as_double());
  RCLCPP_DEBUG(nh_->get_logger(), "Parameters updated (%.6f s).",
               t.seconds_elapsed());
}

void Planner::bootstrap_map() {
  if (std::isnan(bootstrap_z_)) {
    RCLCPP_WARN(nh_->get_logger(), "Map not bootstrapped (invalid z).");
    return;
  }
  RCLCPP_INFO(nh_->get_logger(),
              "Bootstrapping map with traversable robot neighborhood.");

  const int n =
      int(4 * map_.clearance_radius_ / map_.points_min_dist_ + 1);
  const int n_pts = n * n;
  const auto now = nh_->get_clock()->now();

  Eigen::Isometry3f robot_to_map;
  try {
    const auto cloud_to_map_tf = tf_->lookupTransform(
        map_frame_, robot_frame_, tf2::TimePointZero, tf2::durationFromSec(15.));
    robot_to_map = tf2::transformToEigen(cloud_to_map_tf.transform).cast<float>();
  } catch (const tf2::TransformException &ex) {
    RCLCPP_ERROR(nh_->get_logger(),
                 "Could not bootstrap map due to missing transform from %s "
                 "into map %s: %s.",
                 robot_frame_.c_str(), map_frame_.c_str(), ex.what());
    return;
  }
  RCLCPP_INFO(nh_->get_logger(), "Position of %s in map %s: [%.1f %.1f %.1f].",
              robot_frame_.c_str(), map_frame_.c_str(), robot_to_map(0, 3),
              robot_to_map(1, 3), robot_to_map(2, 3));

  sensor_msgs::msg::PointCloud2 cloud;
  cloud.header.frame_id = map_frame_;
  cloud.header.stamp = now;
  cloud.is_bigendian = bigendian();
  cloud.is_dense = true;
  append_field<float>("x", 1, cloud);
  append_field<float>("y", 1, cloud);
  append_field<float>("z", 1, cloud);
  resize_cloud(cloud, uint32_t(1), uint32_t(n_pts));
  sensor_msgs::PointCloud2Iterator<float> pt(cloud, "x");

  for (int i = 0; i < n; ++i) {
    for (int j = 0; j < n; ++j, ++pt) {
      Vec3 x_cloud((-n / 2 + i) * map_.points_min_dist_,
                   (-n / 2 + j) * map_.points_min_dist_, bootstrap_z_);
      Vec3 x_map = robot_to_map * x_cloud;
      pt[0] = x_map(0);
      pt[1] = x_map(1);
      pt[2] = x_map(2);
    }
  }
  input_map_received(cloud);
  RCLCPP_WARN(nh_->get_logger(),
              "Map bootstrapped with %i input points (%lu in map).", n_pts,
              map_.size());
}

Value Planner::time_from_init(const double time) const {
  return Value(time - time_initialized_);
}

Value Planner::time_from_init(const rclcpp::Time &time) const {
  return time_from_init(time.seconds());
}

void Planner::gather_viewpoints() {
  RCLCPP_DEBUG(nh_->get_logger(), "Gathering viewpoints for %lu actors.",
               robot_frames_.size());
  Timer t;
  if (map_frame_.empty()) {
    RCLCPP_ERROR(nh_->get_logger(),
                 "Could not gather robot positions due to missing map frame.");
    return;
  }
  const auto current_expected = nh_->get_clock()->now();
  Lock lock(viewpoints_mutex_);
  for (const auto &frame : robot_frames_) {
    try {
      // Try to get most recent viewpoints.
      const auto timeout = std::max(
          3.0 - (nh_->get_clock()->now() - current_expected).seconds(), 0.0);
      const auto tf =
          tf_->lookupTransform(map_frame_, frame, current_expected,
                               rclcpp::Duration::from_seconds(timeout));

      Vec3 pos(Value(tf.transform.translation.x),
               Value(tf.transform.translation.y),
               Value(tf.transform.translation.z));

      const bool self = (frame == robot_frame_);
      if (self) {
        viewpoints_.push_back(pos);
      } else {
        other_viewpoints_.push_back(pos);
      }

      if (map_.empty()) {
        RCLCPP_WARN(nh_->get_logger(),
                    "Empty map, no points updated from gathered viewpoints.");
        continue;
      }
      Lock cloud_lock(map_.cloud_mutex_);
      Lock index_lock(map_.index_mutex_);

      if (collect_rewards_) {
        RadiusQuery<Value> q1(*map_.index_, FlannMat(pos.data(), 1, 3),
                              2 * max_vp_distance_);

        std::vector<Vec3> vps = {pos};
        update_coverage(map_.cloud_, q1.nn_[0], vps, full_coverage_dist_,
                        coverage_dist_spread_, max_vp_distance_, true, self);

        collect_rewards(map_.cloud_, q1.nn_[0], full_coverage_dist_,
                        coverage_dist_spread_, max_vp_distance_, self_factor_,
                        suppress_base_reward_);
      } else {
        RadiusQuery<Value> q(*map_.index_, FlannMat(pos.data(), 1, 3),
                             max_vp_distance_);
        assert(q.nn_.size() == 1);
        assert(q.dist_.size() == 1);

        RCLCPP_DEBUG(nh_->get_logger(),
                     "%lu / %lu points within %.1f m from %s origin.",
                     q.nn_[0].size(), map_.index_->size(), max_vp_distance_,
                     frame.c_str());
        for (size_t i = 0; i < q.nn_[0].size(); ++i) {
          const Vertex v = q.nn_[0][i];
          const Value d = std::sqrt(q.dist_[0][i]);
          const Value ts = time_from_init(current_expected);
          // TODO: Account for time to enable patrolling.
          if (self) {
            map_.cloud_[v].dist_to_actor_ =
                (std::isfinite(map_.cloud_[v].actor_last_visit_)
                     ? std::min(map_.cloud_[v].dist_to_actor_, d)
                     : d);
            map_.cloud_[v].actor_last_visit_ = ts;
          } else {
            map_.cloud_[v].dist_to_other_actors_ =
                (std::isfinite(map_.cloud_[v].other_actors_last_visit_)
                     ? std::min(map_.cloud_[v].dist_to_other_actors_, d)
                     : d);
            map_.cloud_[v].other_actors_last_visit_ = ts;
          }
        }
      }
    } catch (const tf2::TransformException &ex) {
      RCLCPP_WARN_THROTTLE(nh_->get_logger(), *nh_->get_clock(), 5000,
                           "Viewpoint of %s not updated: %s.", frame.c_str(),
                           ex.what());
      continue;
    }
  }
  const auto now = nh_->get_clock()->now();
  if (viewpoints_pub_->get_subscription_count() > 0 && !viewpoints_.empty()) {
    sensor_msgs::msg::PointCloud2 vp_cloud;
    flann::Matrix<Elem> vp(viewpoints_.data()->data(), viewpoints_.size(), 3);
    create_xyz_cloud(vp, vp_cloud);
    vp_cloud.header.frame_id = map_frame_;
    vp_cloud.header.stamp = now;
    viewpoints_pub_->publish(vp_cloud);
  }
  if (other_viewpoints_pub_->get_subscription_count() > 0 &&
      !other_viewpoints_.empty()) {
    sensor_msgs::msg::PointCloud2 other_vp_cloud;
    flann::Matrix<Elem> other_vp(other_viewpoints_.data()->data(),
                                 other_viewpoints_.size(), 3);
    create_xyz_cloud(other_vp, other_vp_cloud);
    other_vp_cloud.header.frame_id = map_frame_;
    other_vp_cloud.header.stamp = now;
    other_viewpoints_pub_->publish(other_vp_cloud);
  }
  RCLCPP_INFO(nh_->get_logger(),
              "Gathering viewpoints for %lu actors done (%.3f s).",
              robot_frames_.size(), t.seconds_elapsed());
}

void Planner::trace_path_indices(Vertex start, Vertex goal,
                                 const Vertex *predecessor,
                                 std::vector<Vertex> &path_indices) {
  assert(predecessor[start] == start);
  Vertex v = goal;
  while (v != start) {
    path_indices.push_back(v);
    v = predecessor[v];
  }
  path_indices.push_back(v);
  std::reverse(path_indices.begin(), path_indices.end());
}

void Planner::append_path(const std::vector<Vertex> &path_indices,
                          const std::vector<Point> &points,
                          nav_msgs::msg::Path &path) {
  if (path_indices.empty()) {
    return;
  }
  path.poses.reserve(path.poses.size() + path_indices.size());
  for (const auto &v : path_indices) {
    geometry_msgs::msg::PoseStamped pose;
    pose.pose.position.x = points[v].position_[0];
    pose.pose.position.y = points[v].position_[1];
    pose.pose.position.z = points[v].position_[2];
    pose.pose.orientation.w = 1.;
    if (!path.poses.empty()) {
      Vec3 x(pose.pose.position.x - path.poses.back().pose.position.x,
             pose.pose.position.y - path.poses.back().pose.position.y,
             pose.pose.position.z - path.poses.back().pose.position.z);
      x.normalize();
      Vec3 z = ConstVec3Map(points[v].normal_);
      // Fix z direction to be consistent with the previous pose.
      // Assume map z points upward.
      if (z.dot(Vec3(0.f, 0.f, 1.f)) < 0.) {
        z = -z;
      }

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

Buffer<Elem> Planner::viewpoint_dist(const flann::Matrix<Elem> &points) {
  Timer t;
  Buffer<Elem> dist(points.rows);
  std::vector<Vec3> vp_copy;
  {
    Lock lock(viewpoints_mutex_);
    if (viewpoints_.empty()) {
      RCLCPP_WARN(nh_->get_logger(),
                  "No viewpoints gathered. Return infinity.");
      std::fill(dist.begin(), dist.end(),
                std::numeric_limits<Elem>::infinity());
      return dist;
    }
    vp_copy = viewpoints_;
  }
  const size_t n_vp = vp_copy.size();
  RCLCPP_INFO(nh_->get_logger(), "Number of viewpoints: %lu.", n_vp);
  flann::Matrix<Elem> vp(vp_copy.data()->data(), n_vp, 3);
  flann::Index<flann::L2_3D<Elem>> vp_index(vp,
                                            flann::KDTreeSingleIndexParams());
  vp_index.buildIndex();
  Query<Elem> vp_query(vp_index, points, 1);
  return vp_query.dist_buf_;
}

Buffer<Elem> Planner::other_viewpoint_dist(const flann::Matrix<Elem> &points) {
  Timer t;
  Buffer<Elem> dist(points.rows);
  std::vector<Vec3> vp_copy;
  {
    Lock lock(viewpoints_mutex_);
    if (other_viewpoints_.empty()) {
      RCLCPP_WARN(nh_->get_logger(),
                  "No viewpoints gathered from other robots. Return infinity.");
      std::fill(dist.begin(), dist.end(),
                std::numeric_limits<Elem>::infinity());
      return dist;
    }
    vp_copy = other_viewpoints_;
  }
  const size_t n_vp = vp_copy.size();
  RCLCPP_INFO(nh_->get_logger(),
              "Number of viewpoints from other robots: %lu.", n_vp);
  flann::Matrix<Elem> vp(vp_copy.data()->data(), n_vp, 3);
  flann::Index<flann::L2_3D<Elem>> vp_index(vp,
                                            flann::KDTreeSingleIndexParams());
  vp_index.buildIndex();
  Query<Elem> vp_query(vp_index, points, 1);
  return vp_query.dist_buf_;
}

void Planner::input_map_received(
    const sensor_msgs::msg::PointCloud2 &cloud) {
  Lock cloud_lock(map_.cloud_mutex_);
  Lock index_lock(map_.index_mutex_);
  Lock dirty_lock(map_.dirty_mutex_);

  map_.cloud_.clear();
  map_.graph_.clear();
  map_.clear_dirty();

  auto points = flann_matrix_view<Value>(
      const_cast<sensor_msgs::msg::PointCloud2 &>(cloud), position_name_,
      uint32_t(3));
  Vec3 zero(0, 0, 0);
  Value *origin_ptr =
      !viewpoints_.empty() ? viewpoints_.data()->data() : zero.data();
  flann::Matrix<Value> origin(origin_ptr, 1, 3);
  map_.initialize(points, origin);
  map_.update_dirty();
  send_local_map(origin_ptr, cloud.header.stamp);
}

Value Planner::distance_reward(Value distance) const {
  Value r = std::isfinite(distance) ? distance : max_vp_distance_;
  r = r >= min_vp_distance_ ? r : 0.f;
  r /= max_vp_distance_;
  return r;
}

bool Planner::plan(GetPlan::Request::SharedPtr req,
                   GetPlan::Response::SharedPtr res) {
  Timer t;
  Timer t_part;
  {
    Lock lock(initialized_mutex_);
    if (!initialized_) {
      RCLCPP_WARN(nh_->get_logger(), "Won't plan. Waiting for initialization.");
      return false;
    }
  }
  RCLCPP_INFO(nh_->get_logger(),
              "Planning request from [%.1f, %.1f, %.1f] to [%.1f, %.1f, %.1f] "
              "with tolerance %.1f m.",
              req->start.pose.position.x, req->start.pose.position.y,
              req->start.pose.position.z, req->goal.pose.position.x,
              req->goal.pose.position.y, req->goal.pose.position.z,
              req->tolerance);
  {
    Lock lock(last_request_mutex_);
    last_request_ = *req;
  }

  geometry_msgs::msg::PoseStamped start = req->start;
  if (!valid_point(start.pose.position.x, start.pose.position.y,
                   start.pose.position.z)) {
    try {
      const auto tf =
          tf_->lookupTransform(map_frame_, robot_frame_, tf2::TimePointZero,
                               tf2::durationFromSec(5.));
      transform_to_pose(tf, start);
      // If the robot is near the previous goal, try to plan from this goal.
      const auto last_goal_valid = valid_point(last_goal_.pose.position.x,
                                               last_goal_.pose.position.y,
                                               last_goal_.pose.position.z);
      if (last_goal_valid) {
        Eigen::Vector3d pos, last_goal;
        tf2::fromMsg(tf.transform.translation, pos);
        tf2::fromMsg(last_goal_.pose.position, last_goal);
        const double dist = (pos - last_goal).norm();
        if (dist < plan_from_goal_dist_) {
          start = last_goal_;
          RCLCPP_INFO(nh_->get_logger(),
                      "Planning from previous goal [%.1f, %.1f, %.1f].",
                      start.pose.position.x, start.pose.position.y,
                      start.pose.position.z);
        }
      }
    } catch (const tf2::TransformException &ex) {
      RCLCPP_ERROR(nh_->get_logger(),
                   "Could not get robot %s position in map %s: %s.",
                   robot_frame_.c_str(), map_frame_.c_str(), ex.what());
      return false;
    }
  }

  Lock cloud_lock(map_.cloud_mutex_);
  Lock index_lock(map_.index_mutex_);

  t.reset();

  const size_t min_map_points = size_t(Neighborhood::K_NEIGHBORS);
  if (map_.size() < min_map_points) {
    RCLCPP_ERROR(nh_->get_logger(), "Cannot plan in map with %lu < %lu points.",
                 map_.size(), min_map_points);
    return false;
  }

  // TODO: Deal with occupancy on merging.
  // TODO: Index rebuild incrementally with new points.

  // Use the nearest traversable point to robot as the starting point.
  Vec3 start_position(Value(start.pose.position.x),
                      Value(start.pose.position.y),
                      Value(start.pose.position.z));
  const Value start_tol =
      req->tolerance > 0. ? req->tolerance : neighborhood_radius_;
  std::vector<Vertex> traversable;
  for (const auto v : map_.nearby_indices(start_position.data(), start_tol)) {
    if (!(map_.cloud_[v].flags_ & TRAVERSABLE) ||
        (map_.cloud_[v].flags_ & EDGE)) {
      continue;
    }
    traversable.push_back(v);
  }
  if (traversable.empty()) {
    RCLCPP_ERROR(
        nh_->get_logger(),
        "No traversable vertex found within %.1f m from [%.1f, %.1f, %.1f].",
        start_tol, start_position.x(), start_position.y(), start_position.z());
    return false;
  }
  const Vertex v_start = random_start_
                             ? traversable[std::rand() % traversable.size()]
                             : traversable[0];
  RCLCPP_DEBUG(nh_->get_logger(),
               "%s point %lu at [%.1f, %.1f, %.1f] chosen "
               "from %lu traversable ones within %.1f m "
               "from start position [%.1f, %.1f, %.1f].",
               (random_start_ ? "Random" : "Closest"), size_t(v_start),
               map_.cloud_[v_start].position_[0],
               map_.cloud_[v_start].position_[1],
               map_.cloud_[v_start].position_[2], traversable.size(), start_tol,
               req->start.pose.position.x, req->start.pose.position.y,
               req->start.pose.position.z);

  // TODO: Append starting pose as a special vertex with orientation dependent
  // edges. Note, that for some worlds and robots, the neighborhood must be
  // quite large to get traversable points.
  Graph g(map_);
  // Plan in NN graph with approx. travel time costs.
  std::vector<Vertex> predecessor(size_t(g.num_vertices()), INVALID_VERTEX);
  std::vector<Value> path_costs(size_t(g.num_vertices()),
                                std::numeric_limits<Value>::infinity());
  EdgeCosts edge_costs(map_);
  boost::typed_identity_property_map<Vertex> index_map;

  t_part.reset();
  // TODO: Stop via exception if needed.
  boost::dijkstra_shortest_paths_no_color_map(
      g, v_start, predecessor.data(), path_costs.data(), edge_costs, index_map,
      std::less<Value>(), boost::closed_plus<Value>(),
      std::numeric_limits<Value>::infinity(), Value(0.),
      boost::dijkstra_visitor<boost::null_visitor>());
  RCLCPP_INFO(nh_->get_logger(), "Dijkstra (%i pts): %.3f s.", g.num_vertices(),
              t_part.seconds_elapsed());

  // If planning for a given goal, return path to the closest reachable
  // point from the goal.
  t_part.reset();
  if (valid_point(req->goal.pose.position.x, req->goal.pose.position.y,
                  req->goal.pose.position.z)) {
    Vec3 goal_position(Value(req->goal.pose.position.x),
                       Value(req->goal.pose.position.y),
                       Value(req->goal.pose.position.z));
    Vertex v_goal = INVALID_VERTEX;
    Value best_dist = std::numeric_limits<Value>::infinity();
    for (size_t v = 0; v < path_costs.size(); ++v) {
      if (!std::isfinite(path_costs[v])) {
        continue;
      }
      const Value dist =
          (ConstVec3Map(map_.cloud_[v].position_) - goal_position).norm();
      if (dist < best_dist) {
        v_goal = Vertex(v);
        best_dist = dist;
      }
    }
    if (v_goal == INVALID_VERTEX) {
      RCLCPP_ERROR(nh_->get_logger(),
                   "No feasible path towards [%.1f, %.1f, %.1f] was found "
                   "(%.6f, %.3f s).",
                   goal_position.x(), goal_position.y(), goal_position.z(),
                   t_part.seconds_elapsed(), t.seconds_elapsed());
      return false;
    }
    std::vector<Vertex> path_indices;
    trace_path_indices(v_start, v_goal, predecessor.data(), path_indices);
    res->plan.header.frame_id = map_frame_;
    res->plan.header.stamp = nh_->get_clock()->now();
    res->plan.poses.push_back(start);
    append_path(path_indices, map_.cloud_, res->plan);
    RCLCPP_INFO(nh_->get_logger(),
                "Path with %lu poses toward fixed goal [%.1f, %.1f, %.1f] "
                "planned (%.3f s).",
                res->plan.poses.size(), goal_position.x(), goal_position.y(),
                goal_position.z(), t.seconds_elapsed());
    return true;
  }

  // TODO: Account for time to enable patrolling (coverage half-life).
  Vertex v_goal = INVALID_VERTEX;
  for (size_t v = 0; v < path_costs.size(); ++v) {
    if (!collect_rewards_) {
      map_.cloud_[v].reward_ = std::max(
          std::min(distance_reward(map_.cloud_[v].dist_to_actor_),
                   distance_reward(map_.cloud_[v].other_actors_last_visit_)),
          self_factor_ * distance_reward(map_.cloud_[v].dist_to_actor_));
      map_.cloud_[v].reward_ *= (1 + map_.cloud_[v].num_edge_neighbors_);
      // Decrease rewards in specific areas (staging area).
      // TODO: Ensure correct frame (subt) is used here.
      // TODO: Parametrize the areas.
      suppress_reward(map_.cloud_[v]);
    }

    // Keep original path cost, but discount for relative cost.
    map_.cloud_[v].path_cost_ = path_costs[v];
    map_.cloud_[v].relative_cost_ =
        std::pow(map_.cloud_[v].path_cost_, path_cost_pow_) /
        map_.cloud_[v].reward_;
    // Prefer longer feasible paths, with lowest relative costs.
    if (std::isfinite(map_.cloud_[v].path_cost_) &&
        map_.cloud_[v].path_cost_ >= min_path_cost_ &&
        (v_goal == INVALID_VERTEX ||
         map_.cloud_[v].relative_cost_ < map_.cloud_[v_goal].relative_cost_)) {
      v_goal = Vertex(v);
    }
  }

  if (map_pub_->get_subscription_count() > 0) {
    Timer t_send;
    sensor_msgs::msg::PointCloud2 map_cloud;
    map_cloud.header.frame_id = map_frame_;
    map_cloud.header.stamp = nh_->get_clock()->now();
    map_.create_cloud_msg(map_cloud);
    map_pub_->publish(map_cloud);
    RCLCPP_DEBUG(nh_->get_logger(), "Sending map: %.3f s.",
                 t_send.seconds_elapsed());
  }

  if (v_goal == INVALID_VERTEX) {
    RCLCPP_ERROR(nh_->get_logger(), "No valid path (with cost >= %.1f s)/goal "
                                    "found.",
                 min_path_cost_);
    return false;
  }

  std::vector<Vertex> path_indices;
  trace_path_indices(v_start, v_goal, predecessor.data(), path_indices);
  res->plan.header.frame_id = map_frame_;
  res->plan.header.stamp = nh_->get_clock()->now();
  res->plan.poses.push_back(start);
  append_path(path_indices, map_.cloud_, res->plan);
  if (!res->plan.poses.empty()) {
    last_start_ = res->plan.poses.front();
    last_goal_ = res->plan.poses.back();
  }
  RCLCPP_INFO(nh_->get_logger(),
              "Path with %lu poses to goal [%.1f, %.1f, %.1f] "
              "has cost %.3f, reward %.3f, relative cost %.3f (%.3f s).",
              res->plan.poses.size(), map_.cloud_[v_goal].position_[0],
              map_.cloud_[v_goal].position_[1],
              map_.cloud_[v_goal].position_[2], map_.cloud_[v_goal].path_cost_,
              map_.cloud_[v_goal].reward_, map_.cloud_[v_goal].relative_cost_,
              t.seconds_elapsed());
  return true;
}

void Planner::cloud_received(
    const std::shared_ptr<const sensor_msgs::msg::PointCloud2> &cloud) {
  RCLCPP_INFO(nh_->get_logger(), "Cloud received (%lu points).",
              num_points(*cloud));
  {
    Lock lock(initialized_mutex_);
    if (!initialized_) {
      RCLCPP_INFO(nh_->get_logger(),
                  "Skipping input cloud. Waiting for initialization.");
      return;
    }
  }

  // TODO: Build map from all aligned input clouds (interp tf).
  // TODO: Recompute normals.
  if (cloud->row_step != cloud->point_step * cloud->width) {
    RCLCPP_ERROR(nh_->get_logger(),
                 "Skipping cloud with unsupported row step.");
    return;
  }
  const auto age =
      (nh_->get_clock()->now() - rclcpp::Time(cloud->header.stamp)).seconds();
  if (age > max_cloud_age_) {
    RCLCPP_INFO(nh_->get_logger(), "Skipping cloud %.1f s > %.1f s old.", age,
                max_cloud_age_);
    return;
  }
  if (!map_frame_.empty() && map_frame_ != cloud->header.frame_id) {
    RCLCPP_ERROR(nh_->get_logger(),
                 "Cloud frame %s does not match specified map frame %s.",
                 cloud->header.frame_id.c_str(), map_frame_.c_str());
    return;
  }

  // TODO: Allow x[3] or x,y,z and normal[3] or normal_x,y,z.
  const auto field_x = find_field(*cloud, position_name_);
  if (!field_x) {
    RCLCPP_ERROR(nh_->get_logger(), "Skipping cloud without positions.");
    return;
  }
  if (field_x->datatype != sensor_msgs::msg::PointField::FLOAT32) {
    RCLCPP_ERROR(nh_->get_logger(), "Skipping cloud with unsupported type %u.",
                 field_x->datatype);
    return;
  }

  const auto field_nx = find_field(*cloud, normal_name_);
  if (!field_nx) {
    RCLCPP_ERROR(nh_->get_logger(), "Skipping cloud without normals.");
    return;
  }
  if (field_nx->datatype != sensor_msgs::msg::PointField::FLOAT32) {
    RCLCPP_ERROR(nh_->get_logger(),
                 "Skipping cloud with unsupported normal type %u.",
                 field_nx->datatype);
    return;
  }

  geometry_msgs::msg::PoseStamped start;
  try {
    const auto tf =
        tf_->lookupTransform(cloud->header.frame_id, robot_frame_,
                             tf2::TimePointZero, tf2::durationFromSec(5.));
    transform_to_pose(tf, start);
  } catch (const tf2::TransformException &ex) {
    RCLCPP_ERROR(nh_->get_logger(), "Could not get robot position: %s.",
                 ex.what());
    return;
  }
  // TODO: Update whole map with the input map cloud.
}

void Planner::planning_timer_cb() {
  RCLCPP_DEBUG(nh_->get_logger(), "Planning timer callback.");
  Timer t;
  auto req = std::make_shared<GetPlan::Request>();
  {
    Lock lock(last_request_mutex_);
    *req = last_request_;
  }
  auto res = std::make_shared<GetPlan::Response>();
  if (!plan(req, res)) {
    return;
  }
  path_pub_->publish(res->plan);
  RCLCPP_INFO(nh_->get_logger(),
              "Planning robot %s path (%lu poses) in map %s: %.3f s.",
              robot_frame_.c_str(), res->plan.poses.size(), map_frame_.c_str(),
              t.seconds_elapsed());
}

std::vector<Value> Planner::find_robots(const std::string &frame,
                                        const rclcpp::Time &stamp,
                                        float timeout) {
  (void)frame;
  Timer t;
  std::vector<Value> robots;
  robots.reserve(3 * robot_frames_.size());
  for (const auto &f : robot_frames_) {
    if (f == robot_frame_) {
      continue;
    }
    const auto timeout_duration = rclcpp::Duration::from_seconds(std::max(
        timeout - (nh_->get_clock()->now() - stamp).seconds(), 0.));
    geometry_msgs::msg::TransformStamped tf;
    try {
      tf = tf_->lookupTransform(map_frame_, f, stamp, timeout_duration);
    } catch (const tf2::TransformException &ex) {
      RCLCPP_WARN(nh_->get_logger(), "Could not get %s pose in %s: %s.",
                  f.c_str(), map_frame_.c_str(), ex.what());
      continue;
    }
    robots.push_back(static_cast<Value>(tf.transform.translation.x));
    robots.push_back(static_cast<Value>(tf.transform.translation.y));
    robots.push_back(static_cast<Value>(tf.transform.translation.z));
    RCLCPP_INFO(nh_->get_logger(), "Robot %s found in %s at [%.1f, %.1f, %.1f].",
                f.c_str(), map_frame_.c_str(), tf.transform.translation.x,
                tf.transform.translation.y, tf.transform.translation.z);
  }
  RCLCPP_INFO(nh_->get_logger(),
              "%lu / %lu robots found in %.3f s (timeout %.3f s).",
              robots.size() / 3, robot_frames_.size(), t.seconds_elapsed(),
              double(timeout));
  return robots;
}

void Planner::check_initialized() {
  Lock lock(initialized_mutex_);
  if (!initialized_) {
    throw NotInitialized("Not initialized. Waiting for other robots.");
  }
}

void Planner::send_cloud(
    const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr &pub,
    const rclcpp::Time &stamp, bool force) {
  if (force || pub->get_subscription_count() > 0) {
    Timer t;
    sensor_msgs::msg::PointCloud2 cloud;
    cloud.header.frame_id = map_frame_;
    cloud.header.stamp =
        stamp.nanoseconds() == 0 ? nh_->get_clock()->now() : stamp;
    map_.create_cloud_msg(cloud);
    if (num_points(cloud) > 0) {
      pub->publish(cloud);
      RCLCPP_DEBUG(nh_->get_logger(), "Sending cloud %s: %.3f s.",
                   pub->get_topic_name(), t.seconds_elapsed());
    }
  }
}

void Planner::send_map(const rclcpp::Time &stamp, bool force) {
  send_cloud(map_pub_, stamp, force);
}

void Planner::send_dirty_cloud(const rclcpp::Time &stamp, bool force) {
  if (force || dirty_map_pub_->get_subscription_count() > 0) {
    Lock cloud_lock(map_.cloud_mutex_);
    Lock index_lock(map_.index_mutex_);
    Lock updated_lock(map_.updated_mutex_);
    Lock dirty_lock(map_.dirty_mutex_);
    send_cloud(dirty_map_pub_, map_.dirty_indices_, stamp, force);
  }
}

void Planner::send_updated_cloud(const rclcpp::Time &stamp, bool force) {
  if (force || updated_map_pub_->get_subscription_count() > 0) {
    Lock cloud_lock(map_.cloud_mutex_);
    Lock index_lock(map_.index_mutex_);
    Lock updated_lock(map_.updated_mutex_);
    send_cloud(updated_map_pub_, map_.updated_indices_, stamp, force);
  }
}

void Planner::send_local_map(Value *origin, const rclcpp::Time &stamp,
                             bool force) {
  if (force || local_map_pub_->get_subscription_count() > 0) {
    const auto indices = map_.nearby_indices(origin, input_range_);
    send_cloud(local_map_pub_, indices, stamp, force);
  }
}

void Planner::input_cloud_received(
    const std::shared_ptr<const sensor_msgs::msg::PointCloud2> &input) {
  const rclcpp::Time stamp(input->header.stamp);
  const auto age = (nh_->get_clock()->now() - stamp).seconds();
  if (age > max_cloud_age_) {
    RCLCPP_INFO(nh_->get_logger(),
                "Skipping old input cloud from %s, age %.1f s > %.1f s.",
                input->header.frame_id.c_str(), age, max_cloud_age_);
    return;
  }

  check_initialized();
  sensor_msgs::msg::PointCloud2 step_filtered;
  StepFilter step_filter(1024, 1024);
  step_filter.filter(*input, step_filtered);

  Timer t_tf;
  const double wait =
      std::max(5.0 - (nh_->get_clock()->now() - stamp).seconds(), 0.0);
  geometry_msgs::msg::TransformStamped cloud_to_map;
  cloud_to_map =
      tf_->lookupTransform(map_frame_, input->header.frame_id, stamp,
                           rclcpp::Duration::from_seconds(wait));
  RCLCPP_DEBUG(nh_->get_logger(),
               "Had to wait %.3f s for input cloud transform.",
               t_tf.seconds_elapsed());

  Eigen::Isometry3f transform(tf2::transformToEigen(cloud_to_map.transform));

  // TODO: Update map occupancy based on reconstructed surface of 2D cloud.
  if (step_filtered.height > 1 && step_filtered.width > 1) {
    map_.update_occupancy_projection(step_filtered, cloud_to_map.transform);
  } else {
    RCLCPP_WARN(nh_->get_logger(),
                "Cannot update occupancy using unstructured point cloud.");
  }

  Timer t_filter;
  FilterChain<sensor_msgs::msg::PointCloud2>::Filters filters{
      std::make_shared<VoxelFilter<float, int>>("x", map_.points_min_dist_),
      std::make_shared<RangeFilter<float>>("x", 1.f, input_range_),
      std::make_shared<ExcludeFramesFilter<float>>(
          "x", robot_frames_, 1.f, tf_, nh_->get_clock(),
          rclcpp::Duration::from_seconds(3.0)),
      std::make_shared<FilterFromProcessor<sensor_msgs::msg::PointCloud2>>(
          std::make_shared<TransformProcessor<float>>(
              "x", map_frame_, tf_, nh_->get_clock(),
              rclcpp::Duration::from_seconds(3.0)))};
  FilterChain<sensor_msgs::msg::PointCloud2> chain(filters);

  auto cloud = std::make_shared<sensor_msgs::msg::PointCloud2>();
  chain.filter(step_filtered, *cloud);
  RCLCPP_INFO(nh_->get_logger(), "%lu filters applied (%.3f s).",
              filters.size(), t_filter.seconds_elapsed());

  Vec3 origin = transform.translation();
  flann::Matrix<Elem> origin_mat(origin.data(), 1, 3);

  const auto points = flann_matrix_view<float>(*cloud, "x", 3);

  Lock cloud_lock(map_.cloud_mutex_);
  Lock index_lock(map_.index_mutex_);
  {
    Lock added_lock(map_.updated_mutex_);
    Lock lock_dirty(map_.dirty_mutex_);
    map_.merge(points, origin_mat);
    map_.update_dirty();
    // TODO: Mark affected map points for update?
    send_dirty_cloud(cloud->header.stamp);
    map_.clear_dirty();
    send_updated_cloud(cloud->header.stamp);
    map_.clear_updated();
  }
  send_local_map(origin.data(), cloud->header.stamp);
  send_map(cloud->header.stamp);
}

void Planner::input_cloud_received_safe(
    const std::shared_ptr<const sensor_msgs::msg::PointCloud2> &input) {
  try {
    input_cloud_received(input);
  } catch (const tf2::TransformException &ex) {
    RCLCPP_ERROR(nh_->get_logger(),
                 "Could not transform input cloud from %s to %s: %s.",
                 input->header.frame_id.c_str(), map_frame_.c_str(), ex.what());
    return;
  } catch (const Exception &ex) {
    RCLCPP_ERROR(nh_->get_logger(), "Input cloud processing failed: %s\n%s",
                 ex.what(), ex.stacktrace().c_str());
  } catch (const std::runtime_error &ex) {
    RCLCPP_ERROR(nh_->get_logger(), "Input cloud processing failed: %s",
                 ex.what());
  } catch (...) {
    RCLCPP_ERROR(nh_->get_logger(),
                 "Input cloud processing failed with an unknown exception.");
  }
}

} // namespace naex
