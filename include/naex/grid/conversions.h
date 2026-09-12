#pragma once

/**
 * @file
 * Small conversions between the ROS message types, Eigen and the grid's own
 * Point2f, shared by the grid planners.
 *
 * Header-only on purpose: every one of these is a couple of instructions and
 * is called from inner loops and from log statements in both planners.
 */

#include "naex/grid/grid.h"
#include "naex/types.h"
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <cmath>
#include <sstream>
#include <string>

namespace naex {
namespace grid {

template <typename T> inline std::string format(T x, T y, T z) {
  std::stringstream s;
  s << "(" << x << ", " << y << ", " << z << ")";
  return s.str();
}
inline std::string format(const geometry_msgs::msg::Vector3 &v) {
  return format(v.x, v.y, v.z);
}
inline std::string format(const geometry_msgs::msg::Point &v) {
  return format(v.x, v.y, v.z);
}
inline std::string format(const Vec3 &v) {
  return format(v.x(), v.y(), v.z());
}

inline Vec3 toVec3(const geometry_msgs::msg::Point &p) {
  return Vec3(p.x, p.y, p.z);
}
inline Vec3 toVec3(const geometry_msgs::msg::Vector3 &v) {
  return Vec3(v.x, v.y, v.z);
}
inline Vec3 toVec3(const Point2f &v) { return Vec3(v.x, v.y, 0.f); }

template <typename T> inline bool isValid(T x, T y, T z) {
  return std::isfinite(x) && std::isfinite(y) && std::isfinite(z);
}
inline bool isValid(const Vec3 &p) { return isValid(p.x(), p.y(), p.z()); }
inline bool isValid(const geometry_msgs::msg::Point &p) {
  return isValid(p.x, p.y, p.z);
}
inline bool isValid(const geometry_msgs::msg::Vector3 &p) {
  return isValid(p.x, p.y, p.z);
}

} // namespace grid
} // namespace naex
