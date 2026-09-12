#include "naex/clouds.h"
#include "naex/timer.h"
#include "naex/traversability.h"
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <string>
#include <tf2/exceptions.hpp>
#include <tf2_ros/buffer.hpp>
#include <tf2_ros/transform_listener.hpp>
#include <utility>

namespace naex {

/**
 * @brief Traversability estimation node.
 *
 * Subscribes to input point clouds, estimates traversability, and publishes
 * the annotated clouds.
 */
class TraversabilityNode : public rclcpp::Node {
public:
  explicit TraversabilityNode(
      const rclcpp::NodeOptions &options = rclcpp::NodeOptions())
      : rclcpp::Node("traversability", options) {
    update_params();
    advertise();
    subscribe();
    RCLCPP_INFO(get_logger(), "Node initialized.");
  }

  void update_params() {
    proc_.min_z_ = declare_parameter<float>("min_z", proc_.min_z_);
    proc_.max_z_ = declare_parameter<float>("max_z", proc_.max_z_);
    proc_.support_radius_ =
        declare_parameter<float>("support_radius", proc_.support_radius_);
    proc_.min_support_ =
        int(declare_parameter<int>("min_support", proc_.min_support_));
    proc_.inclination_radius_ = declare_parameter<float>(
        "inclination_radius", proc_.inclination_radius_);
    proc_.inclination_weight_ = declare_parameter<float>(
        "inclination_weight", proc_.inclination_weight_);
    proc_.normal_std_weight_ =
        declare_parameter<float>("normal_std_weight", proc_.normal_std_weight_);
    proc_.clearance_radius_ =
        declare_parameter<float>("clearance_radius", proc_.clearance_radius_);
    proc_.clearance_low_ =
        declare_parameter<float>("clearance_low", proc_.clearance_low_);
    proc_.clearance_high_ =
        declare_parameter<float>("clearance_high", proc_.clearance_high_);
    proc_.obstacle_weight_ =
        declare_parameter<float>("obstacle_weight", proc_.obstacle_weight_);
    proc_.remove_low_support_ = declare_parameter<bool>(
        "remove_low_support", proc_.remove_low_support_);
    fixed_frame_ = declare_parameter<std::string>("fixed_frame", fixed_frame_);
    timeout_ = declare_parameter<double>("timeout", timeout_);

    RCLCPP_INFO(get_logger(), "Min z: %.3g m", double(proc_.min_z_));
    RCLCPP_INFO(get_logger(), "Max z: %.3g m", double(proc_.max_z_));
    RCLCPP_INFO(get_logger(), "Support radius: %.3g m",
                double(proc_.support_radius_));
    RCLCPP_INFO(get_logger(), "Min support: %i", proc_.min_support_);
    RCLCPP_INFO(get_logger(), "Inclination radius: %.3g m",
                double(proc_.inclination_radius_));
    RCLCPP_INFO(get_logger(), "Inclination weight: %.3g",
                double(proc_.inclination_weight_));
    RCLCPP_INFO(get_logger(), "Normal std weight: %.3g",
                double(proc_.normal_std_weight_));
    RCLCPP_INFO(get_logger(), "Clearance radius: %.3g m",
                double(proc_.clearance_radius_));
    RCLCPP_INFO(get_logger(), "Clearance low: %.3g m",
                double(proc_.clearance_low_));
    RCLCPP_INFO(get_logger(), "Clearance high: %.3g m",
                double(proc_.clearance_high_));
    RCLCPP_INFO(get_logger(), "Obstacle weight: %.3g",
                double(proc_.obstacle_weight_));
    RCLCPP_INFO(get_logger(), "Remove points with low support: %i",
                int(proc_.remove_low_support_));
    RCLCPP_INFO(get_logger(), "Fixed frame: %s", fixed_frame_.c_str());
    RCLCPP_INFO(get_logger(), "Timeout: %.3g s", timeout_);
  }

  void advertise() {
    cloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("output", 2);
  }

  void subscribe() {
    if (!fixed_frame_.empty()) {
      tf_ = std::make_shared<tf2_ros::Buffer>(get_clock());
      tf_sub_ = std::make_shared<tf2_ros::TransformListener>(*tf_);
    }
    cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
        "input", 2,
        [this](
            const std::shared_ptr<const sensor_msgs::msg::PointCloud2> &msg) {
          this->on_cloud(msg);
        });
  }

  void
  on_cloud(const std::shared_ptr<const sensor_msgs::msg::PointCloud2> &msg) {
    Timer t;
    geometry_msgs::msg::TransformStamped tf;
    tf.transform.rotation.w = 1.0;
    if (!fixed_frame_.empty()) {
      try {
        tf = tf_->lookupTransform(fixed_frame_, msg->header.frame_id,
                                  msg->header.stamp,
                                  rclcpp::Duration::from_seconds(timeout_));
      } catch (const tf2::TransformException &ex) {
        RCLCPP_ERROR(get_logger(), "Could not transform %s to %s: %s.",
                     msg->header.frame_id.c_str(), fixed_frame_.c_str(),
                     ex.what());
        return;
      }
      RCLCPP_INFO(get_logger(), "Waited for transform: %f s.",
                  t.seconds_elapsed());
    }
    t.reset();
    auto output = std::make_unique<sensor_msgs::msg::PointCloud2>();
    proc_.process(*msg, tf.transform, *output);
    const size_t n = num_points(*output);
    cloud_pub_->publish(std::move(output));
    RCLCPP_INFO(get_logger(), "Traversability estimated at %lu points: %f s.",
                n, t.seconds_elapsed());
  }

protected:
  Traversability proc_;
  std::string fixed_frame_;
  // Transform lookup timeout, kept short not to block the executor.
  double timeout_{0.1};
  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::shared_ptr<tf2_ros::TransformListener> tf_sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
};

} // namespace naex

RCLCPP_COMPONENTS_REGISTER_NODE(naex::TraversabilityNode)
