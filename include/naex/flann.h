#pragma once

#include <naex/types.h>
#include <cstddef>
#include <flann/flann.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <string>

namespace naex {

/// Matrix and index types used throughout the point map.
typedef flann::Matrix<Value> FlannMat;
typedef flann::Index<flann::L2_3D<Value>> FlannIndex;

template <typename T>
flann::Matrix<T> flann_matrix_view(sensor_msgs::msg::PointCloud2 &cloud,
                                   const std::string &field,
                                   const size_t count = 1) {
  sensor_msgs::PointCloud2Iterator<T> it(cloud, field);
  return flann::Matrix<T>(&it[0], static_cast<size_t>(cloud.height) * cloud.width,
                          count, cloud.point_step);
}

template <typename T>
flann::Matrix<const T>
const_flann_matrix_view(const sensor_msgs::msg::PointCloud2 &cloud,
                        const std::string &field, const size_t count = 1) {
  sensor_msgs::PointCloud2ConstIterator<T> it(cloud, field);
  return flann::Matrix<const T>(
      &it[0], static_cast<size_t>(cloud.height) * cloud.width, count,
      cloud.point_step);
}

} // namespace naex
