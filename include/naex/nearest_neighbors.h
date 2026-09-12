#pragma once

#include <naex/array.h>
#include <naex/buffer.h>
#include <naex/types.h>
#include <cmath>
#include <flann/flann.hpp>
#include <limits>
#include <vector>

namespace naex {

template <typename I> I invalid_index();

template <> inline int invalid_index<int>() { return -1; }

template <> inline size_t invalid_index<size_t>() {
  return std::numeric_limits<size_t>::max();
}

inline bool invalid_index(int i) { return i < 0; }

inline bool invalid_index(size_t i) {
  return i == std::numeric_limits<size_t>::max();
}

template <typename V> V invalid_distance();

template <> inline float invalid_distance<float>() {
  return std::numeric_limits<float>::infinity();
}

template <> inline double invalid_distance<double>() {
  return std::numeric_limits<double>::infinity();
}

template <> inline long invalid_distance<long>() { return 0L; }

inline bool invalid_distance(float d) { return std::isinf(d); }

inline bool invalid_distance(double d) { return std::isinf(d); }

inline bool invalid_distance(long d) { return d == 0L; }

template <typename I, typename D> inline bool valid_neighbor(I i, D d) {
  return !(invalid_index(i) && invalid_distance(d));
}

template <typename I, typename V> class NearestNeighborTraits {
public:
  I invalid_index() { return naex::invalid_index<I>(); }
  V invalid_distance() { return naex::invalid_distance<V>(); }
};

template <typename I, typename V> class NearestNeighbors {
public:
  explicit NearestNeighbors(I n) : nn_(n), dist_(n) {}
  std::vector<std::vector<I>> nn_;
  std::vector<std::vector<V>> dist_;
};

template <typename I, typename V> class KNearestNeighbors {
public:
  KNearestNeighbors(I n, I k)
      : nn_buf_(n * k), dist_buf_(n * k), nn_(nn_buf_.begin(), n, k),
        dist_(dist_buf_.begin(), n, k) {}
  std::vector<I> nn_buf_;
  std::vector<V> dist_buf_;
  flann::Matrix<I> nn_;
  flann::Matrix<V> dist_;
};

/// k-NN (or capped radius) query against a 3-D L2 index.
template <typename T> class Query {
public:
  Query(const flann::Index<flann::L2_3D<T>> &index,
        const flann::Matrix<T> &queries, const int k = 1,
        const T radius = std::numeric_limits<T>::infinity())
      : nn_buf_(queries.rows * k), dist_buf_(queries.rows * k),
        nn_(nn_buf_.begin(), queries.rows, k),
        dist_(dist_buf_.begin(), queries.rows, k) {
    flann::SearchParams params;
    params.checks = 64;
    params.cores = 0;
    params.sorted = true;
    if (radius < std::numeric_limits<T>::infinity()) {
      params.max_neighbors = k;
      index.radiusSearch(queries, nn_, dist_, radius, params);
    } else {
      index.knnSearch(queries, nn_, dist_, k, params);
    }
  }
  Buffer<int> nn_buf_;
  Buffer<T> dist_buf_;
  flann::Matrix<int> nn_;
  flann::Matrix<T> dist_;
};

/// Radius query against a 3-D L2 index, with unbounded result sets.
template <typename T> class RadiusQuery {
public:
  RadiusQuery(const flann::Index<flann::L2_3D<T>> &index,
              const flann::Matrix<T> &queries, T radius, int checks = 32)
      : nn_(queries.rows), dist_(queries.rows) {
    flann::SearchParams params;
    params.checks = checks;
    params.cores = 0;
    params.sorted = true;
    params.use_heap = flann::FLANN_True;
    const auto radius_2 = radius * radius;
    index.radiusSearch(queries, nn_, dist_, radius_2, params);
  }
  std::vector<std::vector<Index>> nn_;
  std::vector<std::vector<T>> dist_;
};

} // namespace naex
