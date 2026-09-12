#pragma once

#include <Eigen/Dense>
#include <algorithm>
#include <cassert>
#include <cstddef>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <memory>
#include <naex/filter.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <string>
#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_ros/buffer.hpp>

namespace naex {

/// Transform cloud positions in place into the target frame.
template <typename T>
class TransformProcessor : public Processor<sensor_msgs::msg::PointCloud2> {
public:
  typedef Eigen::Transform<T, 3, Eigen::Isometry> Transform;
  typedef Eigen::Matrix<T, 3, 1, Eigen::DontAlign> Vec3T;
  typedef Eigen::Map<Vec3T> Vec3TMap;

  TransformProcessor(const std::string &field, const std::string &target,
                     const std::shared_ptr<tf2_ros::Buffer> buffer,
                     const rclcpp::Clock::SharedPtr clock,
                     const rclcpp::Duration &wait)
      : Processor<sensor_msgs::msg::PointCloud2>(), field_(field),
        target_(target), buffer_(buffer), clock_(clock), wait_(wait) {
    assert(!field_.empty());
    assert(!target_.empty());
    assert(buffer_.get() != nullptr);
    assert(clock_.get() != nullptr);
    assert(wait_.seconds() >= 0.0);
  }
  virtual ~TransformProcessor() = default;

  void process(sensor_msgs::msg::PointCloud2 &cloud) override {
    const rclcpp::Time stamp(cloud.header.stamp);
    const rclcpp::Duration wait = rclcpp::Duration::from_seconds(
        std::max(wait_.seconds() - (clock_->now() - stamp).seconds(), 0.0));
    // May throw a tf2::TransformException.
    const auto to_target =
        buffer_->lookupTransform(target_, cloud.header.frame_id, stamp, wait);
    Transform transform(tf2::transformToEigen(to_target.transform));

    const size_t n = static_cast<size_t>(cloud.height) * cloud.width;
    sensor_msgs::PointCloud2Iterator<T> it(cloud, field_);
    for (size_t i = 0; i < n; ++i, ++it) {
      Vec3TMap x(&it[0]);
      x = transform * x;
    }
    // NB: header.frame_id is deliberately left untouched, as in ROS 1.
  }

protected:
  std::string field_{};
  std::string target_{};
  std::shared_ptr<tf2_ros::Buffer> buffer_{};
  rclcpp::Clock::SharedPtr clock_{};
  rclcpp::Duration wait_{0, 0};
};

} // namespace naex
