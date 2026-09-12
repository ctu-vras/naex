#pragma once

#include <Eigen/Dense>
#include <limits>
#include <naex/cloud_filter.h>
#include <string>

namespace naex {

/// Keep points whose distance from the field origin is within bounds.
template <typename T> class RangeFilter : public PointCloud2FilterFieldBase<T> {
public:
  typedef Eigen::Matrix<T, 3, 1, Eigen::DontAlign> Vec3T;
  typedef Eigen::Map<const Vec3T> ConstVec3TMap;

  explicit RangeFilter(const std::string &field, T min_range = 0.0,
                       T max_range = std::numeric_limits<T>::infinity())
      : PointCloud2FilterFieldBase<T>(field), min_range_(min_range),
        max_range_(max_range) {}

  bool filter(const T *x) override {
    ConstVec3TMap vec(x);
    const T range = vec.norm();
    return range >= min_range_ && range <= max_range_;
  }

protected:
  T min_range_{0.0};
  T max_range_{std::numeric_limits<T>::infinity()};
};

} // namespace naex
