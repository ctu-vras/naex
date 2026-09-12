#pragma once

#include "naex/types.h"
#include <Eigen/Dense>
#include <cmath>

namespace naex {

template <typename T> inline T radians(const T &x) {
  return x / T(180) * T(M_PI);
}

template <typename T> inline T degrees(const T &x) {
  return x / T(M_PI) * T(180);
}

template <typename T> inline T azimuth(const T x, const T y) {
  return std::atan2(y, x);
}

template <typename T> inline T elevation(const T x, const T y, const T z) {
  return std::atan2(z, std::hypot(x, y));
}

template <typename T>
inline void cartesian_to_spherical(const T &x, const T &y, const T &z,
                                   T &azimuth, T &elevation, T &radius) {
  azimuth = std::atan2(y, x);
  elevation = std::atan2(z, std::hypot(x, y));
  radius = std::hypot(std::hypot(x, y), z);
}

template <typename T>
inline void spherical_to_cartesian(const T &azimuth, const T &elevation,
                                   const T &radius, T &x, T &y, T &z) {
  x = radius * std::cos(elevation) * std::cos(azimuth);
  y = radius * std::cos(elevation) * std::sin(azimuth);
  z = radius * std::sin(elevation);
}

/**
 * Inclination w.r.t. x-y plane.
 * @param x Unit vector.
 * @return Inclination w.r.t. x-y plane.
 */
Value inline inclination(const Vec3 &x) {
  return std::atan2(x(2), std::hypot(x(0), x(1)));
}

/**
 * Plane from three points (in non-degenerate configuration).
 * @tparam Derived
 * @param p0
 * @param p1
 * @param p2
 * @return
 */
template <typename Derived>
inline Vec4 plane_from_points(const Eigen::MatrixBase<Derived> &p0,
                              const Eigen::MatrixBase<Derived> &p1,
                              const Eigen::MatrixBase<Derived> &p2) {
  Vec3 n = (p1 - p0).cross(p2 - p0).normalized();
  Value d = -n.dot(p0);
  return Vec4(n(0), n(1), n(2), d);
}

inline Vec4 e2p(Vec3 &v) { return Vec4(v(0), v(1), v(2), 1.); }

} // namespace naex
