#include "naex/clouds.h"

#include "naex/geom.h"
#include <algorithm>
#include <cstdint>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <sstream>
#include <string>

namespace naex {

bool bigendian() {
  uint16_t num = 1;
  return !(*(uint8_t *)&num == 1);
}

size_t num_points(const sensor_msgs::msg::PointCloud2 &cloud) {
  return size_t(cloud.height) * cloud.width;
}

const sensor_msgs::msg::PointField *
find_field(const sensor_msgs::msg::PointCloud2 &cloud,
           const std::string &name) {
  for (const auto &f : cloud.fields) {
    if (f.name == name) {
      return &f;
    }
  }
  return nullptr;
}

void resize_cloud(sensor_msgs::msg::PointCloud2 &cloud, uint32_t height,
                  uint32_t width) {
  cloud.height = height;
  cloud.width = width;
  cloud.row_step = width * cloud.point_step;
  cloud.data.resize(height * cloud.row_step, 0);
}

void print_cloud_summary(const sensor_msgs::msg::PointCloud2 &cloud) {
  sensor_msgs::PointCloud2ConstIterator<float> x_begin(cloud, "x");
  std::stringstream az_ss, el_ss;

  const uint32_t r_step = std::max<uint32_t>(1, cloud.height / 8);
  const uint32_t c_step = std::max<uint32_t>(1, cloud.width / 8);
  for (uint32_t r = 0; r < cloud.height; r += r_step) {
    if (r > 0) {
      az_ss << std::endl;
      el_ss << std::endl;
    }
    for (uint32_t c = 0; c < cloud.width; c += c_step) {
      const auto it = (x_begin + r * cloud.width + c);
      float az, el, radius;
      cartesian_to_spherical(it[0], it[1], it[2], az, el, radius);
      if (c > 0) {
        az_ss << " ";
        el_ss << " ";
      }
      az_ss << degrees(az);
      el_ss << degrees(el);
    }
  }

  RCLCPP_INFO(rclcpp::get_logger("naex"), "Azimuth sample:\n%s",
              az_ss.str().c_str());
  RCLCPP_INFO(rclcpp::get_logger("naex"), "Elevation sample:\n%s",
              el_ss.str().c_str());
}

} // namespace naex
