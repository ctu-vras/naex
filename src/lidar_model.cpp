#include "naex/clouds.h"
#include "naex/timer.h"
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

namespace naex {

/**
 * @brief Fits a spherical projection model to incoming point clouds.
 */
class LidarModel : public rclcpp::Node {
public:
  explicit LidarModel(
      const rclcpp::NodeOptions &options = rclcpp::NodeOptions())
      : rclcpp::Node("lidar_model", options) {
    check_model_ = declare_parameter<bool>("check_model", check_model_);
    cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
        "cloud", 2,
        [this](const std::shared_ptr<const sensor_msgs::msg::PointCloud2>
                   &msg) { this->cloudReceived(msg); });
    RCLCPP_INFO(get_logger(), "Node initialized.");
  }

  void cloudReceived(
      const std::shared_ptr<const sensor_msgs::msg::PointCloud2> &msg) {
    Timer t;
    RCLCPP_DEBUG(get_logger(), "Cloud %u-by-%u received.", msg->height,
                 msg->width);
    print_cloud_summary(*msg);
    if (!model_.fit(*msg)) {
      RCLCPP_WARN(get_logger(), "Fitting sensor model failed.");
      return;
    }
    RCLCPP_INFO(get_logger(),
                "Cloud %u-by-%u: "
                "elevation [%.3g, %.3g], step %.3g, "
                "azimuth [%.3g, %.3g], step %.3g [deg] "
                "(%.3f s).",
                model_.height_, model_.width_,
                degrees(model_.elevation_start_),
                degrees(model_.elevation_start_ +
                        (model_.height_ - 1) * model_.elevation_step_),
                degrees(model_.elevation_step_), degrees(model_.azimuth_start_),
                degrees(model_.azimuth_start_ +
                        (model_.width_ - 1) * model_.azimuth_step_),
                degrees(model_.azimuth_step_), t.seconds_elapsed());
    model_.print_model_summary();
    if (check_model_) {
      model_.check(*msg);
    }
  }

private:
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  SphericalProjection model_;
  bool check_model_{false};
};

} // namespace naex

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<naex::LidarModel>());
  rclcpp::shutdown();
  return 0;
}
