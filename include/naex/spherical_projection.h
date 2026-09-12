#pragma once
// Spherical projection model of an organized point cloud (a rotating lidar).
#include "naex/geom.h"
#include "naex/types.h"
#include <cstdint>
#include <sensor_msgs/msg/point_cloud2.hpp>

namespace naex {

/**
 * Azimuth-elevation model of an organized cloud.
 *
 * Rows correspond to elevation, columns to azimuth, both sampled uniformly.
 * Fitting estimates the start and step of both from the cloud itself.
 */
class SphericalProjection {
public:
  SphericalProjection() {}

  /// Log the mean angular error of the model on the given cloud.
  bool check(const sensor_msgs::msg::PointCloud2 &cloud);

  /// Log an 8-by-8 sample of the modelled azimuth and elevation.
  void print_model_summary();

  /// Fit from median of models generated from pairs of valid points.
  bool fit_robust(const sensor_msgs::msg::PointCloud2 &cloud);

  bool fit(const sensor_msgs::msg::PointCloud2 &cloud);

  template <typename T>
  inline void unproject(const T &r, const T &c, T &x, T &y, T &z) {
    const T azimuth = azimuth_start_ + c * azimuth_step_;
    const T elevation = elevation_start_ + r * elevation_step_;
    spherical_to_cartesian(azimuth, elevation, T(1), x, y, z);
  }

  template <typename T>
  void project(const T x, const T y, const T z, T &r, T &c) {
    T azimuth, elevation, radius;
    cartesian_to_spherical(x, y, z, azimuth, elevation, radius);
    r = (elevation - elevation_start_) / elevation_step_;
    c = (azimuth - azimuth_start_) / azimuth_step_;
  }

  // Azimuth, angle in xy plane, positive for x to y direction;
  // azimuth at image[:, 0].
  float azimuth_start_{0};
  float azimuth_step_{0};
  // Elevation, angle from from xy plane to point;
  // elevation at image[0, :].
  float elevation_start_{0};
  float elevation_step_{0};
  // Cloud 2D grid size.
  uint32_t height_{0};
  uint32_t width_{0};
};

} // namespace naex
