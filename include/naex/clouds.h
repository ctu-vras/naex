#pragma once
// Point cloud field manipulation.
//
// The templates below have to stay in the header; the non-template helpers
// are defined in src/clouds.cpp (library naex_core).
#include "naex/geom.h"
#include "naex/point_field_traits.h"
#include "naex/timer.h"
#include "naex/types.h"
#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <string>
#include <type_traits>

namespace naex {

/// True if the host is big endian.
bool bigendian();

/// Number of points in the cloud, height * width.
size_t num_points(const sensor_msgs::msg::PointCloud2 &cloud);

/// Find a field by name, nullptr if there is no such field.
const sensor_msgs::msg::PointField *
find_field(const sensor_msgs::msg::PointCloud2 &cloud, const std::string &name);

template <typename T>
void fill_field(const std::string &name, const T *it,
                sensor_msgs::msg::PointCloud2 &cloud) {
  size_t n = cloud.height * cloud.width;
  sensor_msgs::PointCloud2Iterator<T> field_it(cloud, name);
  const auto end = it + n;
  for (; it != end; ++it, ++field_it) {
    *field_it = *it;
  }
}

template <typename T>
void fill_const_field(const std::string &name, const T &value,
                      sensor_msgs::msg::PointCloud2 &cloud) {
  size_t n = cloud.height * cloud.width;
  sensor_msgs::PointCloud2Iterator<T> field_it(cloud, name);
  const auto end = field_it + n;
  for (; field_it != end; ++field_it) {
    *field_it = value;
  }
}

/// Drop all fields and reset the point step.
void reset_fields(sensor_msgs::msg::PointCloud2 &cloud);

template <typename T>
void append_field(const std::string &name, const uint32_t count,
                  sensor_msgs::msg::PointCloud2 &cloud)
//        const uint32_t offset = cloud.point_step)
{
  typedef typename std::remove_reference<T>::type C;
  sensor_msgs::msg::PointField field;
  field.name = name;
  field.offset = cloud.point_step;
  //        field.offset = offset;
  field.datatype = PointFieldTraits<C>::datatype();
  field.count = count;
  cloud.fields.emplace_back(field);
  cloud.point_step += count * PointFieldTraits<C>::value_size();
  // Setting row_step is up to caller.
}

template <typename T = float>
void append_position_fields(sensor_msgs::msg::PointCloud2 &cloud) {
  append_field<T>("x", 1, cloud);
  append_field<T>("y", 1, cloud);
  append_field<T>("z", 1, cloud);
}

template <typename T = float>
void append_normal_fields(sensor_msgs::msg::PointCloud2 &cloud) {
  append_field<T>("nx", 1, cloud);
  append_field<T>("ny", 1, cloud);
  append_field<T>("nz", 1, cloud);
}

void append_occupancy_fields(sensor_msgs::msg::PointCloud2 &cloud);

void append_traversability_fields(sensor_msgs::msg::PointCloud2 &cloud);

void append_planning_fields(sensor_msgs::msg::PointCloud2 &cloud);

/// Set cloud size and (re)allocate the data buffer.
void resize_cloud(sensor_msgs::msg::PointCloud2 &cloud, uint32_t height,
                  uint32_t width);

/// Log an 8-by-8 sample of the azimuth and elevation of an organized cloud.
void print_cloud_summary(const sensor_msgs::msg::PointCloud2 &cloud);

/// Copy header, fields and per-point layout, but no data.
void copy_cloud_metadata(const sensor_msgs::msg::PointCloud2 &input,
                         sensor_msgs::msg::PointCloud2 &output);

/**
 * Copy selected points.
 * @tparam C A container type, with begin, end and size methods.
 * @param input Input cloud.
 * @param indices Container of indices.
 * @param output Output cloud.
 */
template <typename C>
void copy_points(const sensor_msgs::msg::PointCloud2 &input, const C &indices,
                 sensor_msgs::msg::PointCloud2 &output) {
  Timer t;
  output.header = input.header;
  output.height = 1;
  output.width = decltype(output.width)(indices.size());
  output.fields = input.fields;
  output.is_bigendian = input.is_bigendian;
  output.point_step = input.point_step;
  output.row_step = output.width * output.point_step;
  output.data.resize(indices.size() * output.point_step);
  output.is_dense = input.is_dense;
  const auto in_ptr = input.data.data();
  uint8_t *out_ptr = output.data.data();
  auto it = indices.begin();
  for (size_t i = 0; i != indices.size(); ++i, ++it) {
    std::copy(in_ptr + (*it) * input.point_step,
              in_ptr + (*it + 1) * input.point_step,
              out_ptr + i * output.point_step);
  }
  RCLCPP_DEBUG(rclcpp::get_logger("naex"), "%lu / %lu points copied (%.6f s).",
               indices.size(), size_t(input.height * input.width),
               t.seconds_elapsed());
}

} // namespace naex
