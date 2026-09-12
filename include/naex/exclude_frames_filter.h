#pragma once

#include <naex/cloud_filter.h>
#include <Eigen/Dense>
#include <algorithm>
#include <cassert>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <tf2_ros/buffer.hpp>
#include <vector>

namespace naex {

/// Drop points closer than min_range to the origin of any of the given frames.
template <typename T>
class ExcludeFramesFilter : public PointCloud2FilterFieldBase<T> {
public:
  typedef Eigen::Matrix<T, 3, 1, Eigen::DontAlign> Vec3T;
  typedef Eigen::Map<const Vec3T> ConstVec3TMap;

  ExcludeFramesFilter(const std::string &field,
                      const std::vector<std::string> &frames, T min_range,
                      const std::shared_ptr<tf2_ros::Buffer> buffer,
                      const rclcpp::Clock::SharedPtr clock,
                      const rclcpp::Duration &wait)
      : PointCloud2FilterFieldBase<T>(field), frames_(frames),
        min_range_(min_range), buffer_(buffer), clock_(clock), wait_(wait) {
    assert(!frames.empty());
    assert(min_range_ >= 0.0);
    assert(buffer_.get() != nullptr);
    assert(clock_.get() != nullptr);
    assert(wait_.seconds() >= 0.0);
  }

  void prepare(const sensor_msgs::msg::PointCloud2 &input) override {
    positions_.clear();
    positions_.reserve(3 * frames_.size());
    const rclcpp::Time stamp(input.header.stamp);
    for (const auto &f : frames_) {
      const rclcpp::Duration wait = rclcpp::Duration::from_seconds(
          std::max(wait_.seconds() - (clock_->now() - stamp).seconds(), 0.0));
      const auto tf =
          buffer_->lookupTransform(input.header.frame_id, f, stamp, wait);
      positions_.push_back(T(tf.transform.translation.x));
      positions_.push_back(T(tf.transform.translation.y));
      positions_.push_back(T(tf.transform.translation.z));
    }
  }

  bool filter(const T *x) override {
    ConstVec3TMap vec(x);
    for (size_t i = 0; i + 2 < positions_.size(); i += 3) {
      ConstVec3TMap pos(&positions_[i]);
      if ((vec - pos).norm() < min_range_) {
        return false;
      }
    }
    return true;
  }

protected:
  std::vector<std::string> frames_{};
  T min_range_{0.0};
  std::shared_ptr<tf2_ros::Buffer> buffer_{};
  rclcpp::Clock::SharedPtr clock_{};
  rclcpp::Duration wait_{0, 0};
  std::vector<T> positions_{};
};

} // namespace naex
