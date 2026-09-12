#pragma once

#include <algorithm>
#include <cassert>
#include <cmath>
#include <cstddef>
#include <limits>
#include <naex/clouds.h>
#include <naex/filter.h>
#include <naex/hash.h>
#include <naex/timer.h>
#include <naex/types.h>
#include <random>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace naex {

template <typename C> void index_range(size_t n, C &indices) {
  indices.reserve(n);
  for (size_t i = 0; i < n; ++i) {
    indices.push_back(i);
  }
  indices.resize(n);
}

template <typename It> void shuffle(It begin, It end) {
  std::random_device rd;
  std::mt19937 g(rd());
  std::shuffle(begin, end, g);
}

template <typename C> void random_permutation(size_t n, C &indices) {
  Timer t;
  index_range(n, indices);
  shuffle(indices.begin(), indices.end());
  RCLCPP_DEBUG(rclcpp::get_logger("naex.filter"),
               "Random permutation of %lu points created (%.6f s).", n,
               t.seconds_elapsed());
}

/// Integer voxel key.
template <typename I> class Voxel {
public:
  typedef Voxel<I> Same;

  class Hash {
  public:
    size_t operator()(const Same &voxel) const noexcept { return voxel.hash(); }
  };

  Voxel() {}
  Voxel(I x, I y, I z) : x_(x), y_(y), z_(z) {}
  Voxel(const Same &other) : x_(other.x_), y_(other.y_), z_(other.z_) {}
  Same &operator=(const Same &other) = default;

  size_t hash() const noexcept {
    size_t h = 0;
    hash_combine(h, x_);
    hash_combine(h, y_);
    hash_combine(h, z_);
    return h;
  }
  friend bool operator==(const Same &a, const Same &b) {
    return (a.x_ == b.x_) && (a.y_ == b.y_) && (a.z_ == b.z_);
  }

  static I lb() { return std::numeric_limits<I>::min(); }
  static I ub() { return std::numeric_limits<I>::max(); }

  template <typename T> bool from(const T *values, T bin_size) {
    if (!std::isfinite(values[0]) || !std::isfinite(values[1]) ||
        !std::isfinite(values[2])) {
      return false;
    }
    Value x = std::floor(values[0] / bin_size);
    Value y = std::floor(values[1] / bin_size);
    Value z = std::floor(values[2] / bin_size);
    if (x < Value(lb()) || x > Value(ub()) || y < Value(lb()) ||
        y > Value(ub()) || z < Value(lb()) || z > Value(ub())) {
      return false;
    }
    x_ = static_cast<I>(x);
    y_ = static_cast<I>(y);
    z_ = static_cast<I>(z);
    return true;
  }
  template <typename T> void center(T *values, T bin_size) const {
    values[0] = (T(x_) + T(0.5)) * bin_size;
    values[1] = (T(y_) + T(0.5)) * bin_size;
    values[2] = (T(z_) + T(0.5)) * bin_size;
  }

  I x_{0};
  I y_{0};
  I z_{0};
};

template <typename I> using VoxelVec = std::vector<Voxel<I>>;

template <typename I>
using VoxelSet = std::unordered_set<Voxel<I>, typename Voxel<I>::Hash>;

template <typename I, typename T>
using VoxelMap = std::unordered_map<Voxel<I>, T, typename Voxel<I>::Hash>;

template <typename T, typename I>
void voxel_filter(const sensor_msgs::msg::PointCloud2 &input,
                  const std::string &field, const T bin_size,
                  sensor_msgs::msg::PointCloud2 &output, VoxelSet<I> &voxels) {
  Timer t, t_part;
  assert(input.row_step % input.point_step == 0);
  const size_t n_pts = num_points(input);

  std::vector<size_t> indices;
  random_permutation(n_pts, indices);

  t_part.reset();
  std::vector<size_t> keep;
  keep.reserve(indices.size());
  sensor_msgs::PointCloud2ConstIterator<T> pt_it(input, field);
  voxels.reserve(indices.size());
  for (size_t i = 0; i < n_pts; ++i, ++pt_it) {
    Voxel<I> voxel;
    if (!voxel.template from<T>(&pt_it[0], bin_size))
      continue;

    if (voxels.find(voxel) == voxels.end()) {
      voxels.insert(voxel);
      keep.push_back(i);
    }
  }
  RCLCPP_DEBUG(rclcpp::get_logger("naex.filter"),
               "Getting %lu indices to keep (%.6f s).", keep.size(),
               t_part.seconds_elapsed());

  // Copy selected indices.
  t_part.reset();
  copy_points(input, keep, output);
  RCLCPP_DEBUG(rclcpp::get_logger("naex.filter"),
               "%lu / %lu points kept by voxel filter (%.6f s).", keep.size(),
               n_pts, t_part.seconds_elapsed());
}

/// Keep one point per occupied voxel of the given size.
template <typename T, typename I>
class VoxelFilter : public Filter<sensor_msgs::msg::PointCloud2> {
public:
  VoxelFilter(const std::string &field, T bin_size)
      : Filter<sensor_msgs::msg::PointCloud2>(), field_(field),
        bin_size_(bin_size) {
    assert(std::isfinite(bin_size) && bin_size > 0.0);
  }
  virtual ~VoxelFilter() = default;

  void filter(const sensor_msgs::msg::PointCloud2 &input,
              sensor_msgs::msg::PointCloud2 &output) override {
    VoxelSet<I> voxels;
    voxel_filter<T, I>(input, field_, bin_size_, output, voxels);
  }

protected:
  std::string field_;
  T bin_size_;
};

} // namespace naex
