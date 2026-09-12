#pragma once

#include <naex/buffer.h>
#include <Eigen/Dense>
#include <cstdarg>
#include <cstddef>
#include <cstdint>
#include <flann/flann.hpp>

namespace naex {

/// Dense D-dimensional array over a shared Buffer, with byte strides.
template <size_t D, typename T, typename B = T> class Array {
public:
  typedef Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic, Eigen::DontAlign>
      EigenMatrix;
  typedef Eigen::Map<EigenMatrix> EigenMatrixView;

  Array() {}

  explicit Array(size_t dim_0)
      : size_{dim_0}, stride_{sizeof(T)}, data_(numel()) {
    static_assert(D == 1, "D must be 1.");
  }
  Array(size_t dim_0, size_t dim_1)
      : size_{dim_0, dim_1}, stride_{sizeof(T) * dim_1, sizeof(T)},
        data_(numel()) {
    static_assert(D == 2, "D must be 2.");
  }
  Array(size_t dim_0, size_t dim_1, size_t dim_2)
      : size_{dim_0, dim_1, dim_2},
        stride_{sizeof(T) * dim_2 * dim_1, sizeof(T) * dim_2, sizeof(T)},
        data_(numel()) {
    static_assert(D == 3, "D must be 3.");
  }
  size_t numel() const {
    size_t n = 1;
    for (size_t dim = 0; dim < D; ++dim) {
      n *= size_[dim];
    }
    return n;
  }
  flann::Matrix<T> flann_matrix_view() {
    static_assert(D == 2, "D must be 2.");
    return flann::Matrix<T>(data_.data(), size_[0], size_[1]);
  }
  EigenMatrixView eigen_matrix_view() {
    static_assert(D == 2, "D must be 2.");
    return EigenMatrixView(data_.data(), size_[0], size_[1]);
  }
  size_t linear(size_t i_0) const { return i_0 * stride_[0]; }
  size_t linear(size_t i_0, size_t i_1) const {
    return i_0 * stride_[0] + i_1 * stride_[1];
  }
  size_t linear(size_t i_0, size_t i_1, size_t i_2) const {
    return i_0 * stride_[0] + i_1 * stride_[1] + i_2 * stride_[2];
  }
  T &value(size_t i) {
    return reinterpret_cast<T &>(
        reinterpret_cast<uint8_t *>(data_.begin())[i]);
  }
  const T &value(size_t i) const {
    return reinterpret_cast<const T &>(
        reinterpret_cast<const uint8_t *>(data_.begin())[i]);
  }
  T &operator[](size_t i_0) { return value(linear(i_0)); }
  T &operator()(size_t i_0) { return value(linear(i_0)); }
  const T &operator()(size_t i_0) const { return value(linear(i_0)); }
  T &operator()(size_t i_0, size_t i_1) { return value(linear(i_0, i_1)); }
  const T &operator()(size_t i_0, size_t i_1) const {
    return value(linear(i_0, i_1));
  }
  T &operator()(size_t i_0, size_t i_1, size_t i_2) {
    return value(linear(i_0, i_1, i_2));
  }
  const T &operator()(size_t i_0, size_t i_1, size_t i_2) const {
    return value(linear(i_0, i_1, i_2));
  }

public:
  size_t size_[D];
  size_t stride_[D]; // Stride in bytes
  Buffer<T> data_;   // Must come after size_ to use numel() in initializer.
};

typedef Array<2, int8_t> Array2Int8;
typedef Array<2, uint8_t> Array2UInt8;
typedef Array<2, int16_t> Array2Int16;
typedef Array<2, uint16_t> Array2UInt16;
typedef Array<2, int32_t> Array2Int32;
typedef Array<2, uint32_t> Array2UInt32;
typedef Array<2, int64_t> Array2Int64;
typedef Array<2, uint64_t> Array2UInt64;
typedef Array<2, float> Array2Float32;
typedef Array<2, double> Array2Float64;

} // namespace naex
