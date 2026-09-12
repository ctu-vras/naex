#pragma once

#include <cmath>
#include <cstddef>
#include <naex/clouds.h>
#include <naex/filter.h>
#include <naex/geom.h>
#include <naex/timer.h>
#include <naex/types.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <string>
#include <vector>

namespace naex {

// Templated point field filters.

template <typename T>
class PointCloud2FilterFieldBase
    : public Filter<sensor_msgs::msg::PointCloud2> {
public:
  typedef PointCloud2FilterFieldBase<T> Same;
  typedef std::shared_ptr<Same> Ptr;
  typedef std::shared_ptr<const Same> ConstPtr;

  explicit PointCloud2FilterFieldBase(const std::string &field)
      : field_(field) {}
  virtual ~PointCloud2FilterFieldBase() = default;

  virtual void prepare(const sensor_msgs::msg::PointCloud2 &input) {
    (void)input;
  }

  void filter(const sensor_msgs::msg::PointCloud2 &input,
              sensor_msgs::msg::PointCloud2 &output) override {
    prepare(input);
    Timer t;
    const size_t n = num_points(input);
    std::vector<size_t> indices;
    indices.reserve(n);
    sensor_msgs::PointCloud2ConstIterator<T> it(input, field_);
    for (size_t i = 0; i < n; ++i, ++it) {
      if (filter(&it[0])) {
        indices.push_back(i);
      }
    }
    copy_points(input, indices, output);
    RCLCPP_DEBUG(rclcpp::get_logger("naex.filter"),
                 "Filter %s kept %lu / %lu points (%.6f s).", this->type_name(),
                 indices.size(), n, t.seconds_elapsed());
  }
  virtual bool filter(const T *x) = 0;

protected:
  std::string field_;
};

} // namespace naex
