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

template <typename T>
class PointCloud2FilterFieldChain
    : public PointCloud2FilterFieldBase<T>,
      public FilterChain<sensor_msgs::msg::PointCloud2> {
public:
  typedef std::vector<typename PointCloud2FilterFieldBase<T>::Ptr> Filters;

  PointCloud2FilterFieldChain(const std::string &field, const Filters &filters)
      : PointCloud2FilterFieldBase<T>(field),
        FilterChain<sensor_msgs::msg::PointCloud2>(
            FilterChain<sensor_msgs::msg::PointCloud2>::Filters(
                filters.begin(), filters.end())) {}
  virtual ~PointCloud2FilterFieldChain() = default;

  void filter(const sensor_msgs::msg::PointCloud2 &input,
              sensor_msgs::msg::PointCloud2 &output) override {
    // First iterate over points, chain in field filter, and copy points to
    // keep.
    FilterChain<sensor_msgs::msg::PointCloud2>::filter(input, output);
  }
  bool filter(const T *x) override {
    bool keep = true;
    for (auto &f : this->filters_) {
      // We know the type thanks to constructor.
      auto *g = static_cast<PointCloud2FilterFieldBase<T> *>(f.get());
      if (!g)
        continue;
      keep &= g->filter(x);
    }
    return keep;
  }
};

template <typename T>
class PointCloud2InPlaceFilterBase
    : public Processor<sensor_msgs::msg::PointCloud2> {
public:
  explicit PointCloud2InPlaceFilterBase(const std::string &field)
      : field_(field) {}
  virtual ~PointCloud2InPlaceFilterBase() = default;

  void process(sensor_msgs::msg::PointCloud2 &cloud) override {
    const size_t n = num_points(cloud);
    sensor_msgs::PointCloud2Iterator<T> it(cloud, field_);
    for (size_t i = 0; i < n; ++i, ++it) {
      filter(&it[0]);
    }
  }
  virtual bool filter(T *x) = 0;

protected:
  std::string field_;
};

} // namespace naex
