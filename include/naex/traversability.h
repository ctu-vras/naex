#pragma once
// Traversability estimation from a raw point cloud.
#include "naex/types.h"
#include <geometry_msgs/msg/transform.hpp>
#include <limits>
#include <sensor_msgs/msg/point_cloud2.hpp>

namespace naex {

/**
 * @brief Traversability estimation.
 *
 * @param input Input point cloud.
 * @param transform Transform to fixed frame.
 * @param min_z Minimum height in the fixed frame.
 * @param max_z Maximum height in the fixed frame.
 * @param support_radius Local support radius.
 * @param min_support Minimum support to consider point further.
 * @param inclination_radius Radius for local surface orientation.
 * @param inclination_weight Weight of inclination in cost.
 * @param normal_std_weight Weight of normal standard deviation in cost.
 * @param clearance_radius Radius of clearance cylinder.
 * @param clearance_low Height of cylinder base.
 * @param clearance_high Height of cylinder top.
 * @param obstacle_weight Weight of each obstacle point.
 * @param output Output point cloud.
 */
void compute_traversability(
    const sensor_msgs::msg::PointCloud2 &input,
    const geometry_msgs::msg::Transform &transform, const float min_z,
    const float max_z, const float support_radius, const int min_support,
    const float inclination_radius, const float inclination_weight,
    const float normal_std_weight, const float clearance_radius,
    const float clearance_low, const float clearance_high,
    const float obstacle_weight, sensor_msgs::msg::PointCloud2 &output);

/// Copy points with at least @p min_support neighbors into @p output.
void remove_low_support(const sensor_msgs::msg::PointCloud2 &input,
                        const int min_support,
                        sensor_msgs::msg::PointCloud2 &output);

/// Traversability estimation parameters and entry point.
class Traversability {
public:
  float min_z_ = std::numeric_limits<float>::quiet_NaN();
  float max_z_ = std::numeric_limits<float>::quiet_NaN();

  float support_radius_ = 0.25;
  int min_support_ = 3;

  float inclination_radius_ = 0.5;
  float inclination_weight_ = 1;
  float normal_std_weight_ = 1;

  float clearance_radius_ = 0.5;
  float clearance_low_ = 0.1;
  float clearance_high_ = 0.5;
  float obstacle_weight_ = 1;

  bool remove_low_support_ = false;

  void process(const sensor_msgs::msg::PointCloud2 &input,
               const geometry_msgs::msg::Transform &transform,
               sensor_msgs::msg::PointCloud2 &output);
};

} // namespace naex
