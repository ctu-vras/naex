// Occupancy updates: which map points were seen through.
#include "naex/map.h"

#include "naex/clouds.h"
#include "naex/flann.h"
#include "naex/geom.h"
#include "naex/nearest_neighbors.h"
#include "naex/spherical_projection.h"
#include "naex/timer.h"
#include "naex/types.h"
#include <algorithm>
#include <cassert>
#include <cmath>
#include <cstddef>
#include <geometry_msgs/msg/transform.hpp>
#include <limits>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <tf2_eigen/tf2_eigen.hpp>

namespace naex {

void Map::update_occupancy_projection(
    const sensor_msgs::msg::PointCloud2 &cloud,
    const geometry_msgs::msg::Transform &cloud_to_map_tf) {
  Timer t;
  Timer t_part;

  if (empty()) {
    RCLCPP_INFO(map_logger(), "No map points for updating occupancy.");
    return;
  }

  assert(cloud.height > 1);
  const size_t n_pts = num_points(cloud);
  if (n_pts == 0) {
    RCLCPP_INFO(map_logger(), "No input points for updating occupancy.");
    return;
  }

  t_part.reset();
  SphericalProjection model;
  if (!model.fit(cloud)) {
    RCLCPP_WARN(map_logger(), "Could not fit cloud model (%.6f s).",
                t_part.seconds_elapsed());
    return;
  }

  sensor_msgs::PointCloud2ConstIterator<float> x_begin(cloud, "x");
  Eigen::Isometry3f cloud_to_map(tf2::transformToEigen(cloud_to_map_tf));
  Eigen::Isometry3f map_to_cloud = cloud_to_map.inverse(Eigen::Isometry);

  // Find closest map points to sensor origin.
  Lock cloud_lock(cloud_mutex_);
  Lock index_lock(index_mutex_);
  t_part.reset();
  Vec3 origin = cloud_to_map.translation();
  RadiusQuery<Value> q_map(*index_, FlannMat(origin.data(), 1, 3), 5.f);
  RCLCPP_DEBUG(map_logger(), "%lu closest map points found (%.6f s).",
               q_map.nn_[0].size(), t_part.seconds_elapsed());

  Lock removed_lock(updated_mutex_);
  Lock dirty_lock(dirty_mutex_);
  // Test each map point nearby, whether we can see through.
  Index n_occupied = 0;
  Index n_empty = 0;
  Index n_occluded = 0;
  Index n_modified = 0;
  const Value eps = points_min_dist_ / 2;
  for (const auto i : q_map.nn_[0]) {
    cloud_[i].dist_to_plane_ = std::numeric_limits<Value>::quiet_NaN();
    Vec3 p = map_to_cloud * ConstVec3Map(cloud_[i].position_);
    Value r, c;
    model.project(p(0), p(1), p(2), r, c);

    if (r < 1 || r > Value(cloud.height) - 2)
      continue;
    int r0 = int(std::round(r));

    if (c < 1 || c > Value(cloud.width) - 2)
      continue;
    int c0 = int(std::round(c));

    Index i0 = r0 * Index(cloud.width) + c0;

    Value rint;
    Index i1 = std::modf(r, &rint) >= Value(0)
                   ? (r0 + 1) * Index(cloud.width) + c0
                   : (r0 - 1) * Index(cloud.width) + c0;

    Value cint;
    Index i2 = std::modf(c, &cint) >= Value(0)
                   ? r0 * Index(cloud.width) + c0 + 1
                   : r0 * Index(cloud.width) + c0 - 1;

    ConstVec3Map p0(&(x_begin + i0)[0]);
    if (!std::isfinite(p0(0)) || !std::isfinite(p0(1)) || !std::isfinite(p0(2)))
      continue;
    ConstVec3Map p1(&(x_begin + i1)[0]);
    if (!std::isfinite(p1(0)) || !std::isfinite(p1(1)) || !std::isfinite(p1(2)))
      continue;
    ConstVec3Map p2(&(x_begin + i2)[0]);
    if (!std::isfinite(p2(0)) || !std::isfinite(p2(1)) || !std::isfinite(p2(2)))
      continue;
    Vec4 plane = plane_from_points(p0, p1, p2);

    // Make sure positive distance is towards sensor at [0, 0, 0],
    // i.e., outside from surface.
    if (plane(3) < 0.)
      plane *= -1;

    Value signed_dist = plane.dot(e2p(p));
    cloud_[i].dist_to_plane_ = signed_dist;

    if (signed_dist < -eps) {
      // Map point is occluded. Do nothing.
      ++n_occluded;
      continue;
    }

    if (signed_dist < eps) {
      // Known surface measured again.
      if (cloud_[i].num_occupied_ >= max_occ_counter_) {
        cloud_[i].num_occupied_ /= 2;
        cloud_[i].num_empty_ /= 2;
      }
      ++cloud_[i].num_occupied_;
      ++n_occupied;
    } else {
      // Known surface seen through,
      // indicating it may be noise or it moved somewhere else.

      // Check the incidence angle is not too high.
      Vec3 n = plane.head(3);
      const auto abs_cos = std::abs(p0.normalized().dot(n));
      if (abs_cos < min_empty_cos_) {
        continue;
      }

      if (cloud_[i].num_empty_ >= max_occ_counter_) {
        cloud_[i].num_occupied_ /= 2;
        cloud_[i].num_empty_ /= 2;
      }
      ++cloud_[i].num_empty_;
      ++n_empty;
    }

    if (point_empty(cloud_[i])) {
      if (cloud_[i].flags_ & STATIC) {
        cloud_[i].flags_ &= ~STATIC;
        // Don't update the point we remove.
        dirty_indices_.erase(i);
        for (const auto j : graph_[i].neighbors_) {
          // Don't add removed points.
          if (!(cloud_[j].flags_ & STATIC)) {
            continue;
          }
          dirty_indices_.insert(j);
        }
        index_->removePoint(i);
        updated_indices_.push_back(i);
        ++n_modified;
      }
    }
  }
  RCLCPP_INFO(map_logger(),
              "Occupancy of %lu points updated, %lu modified state, "
              "%lu occluded, %lu occupied, %lu empty (%.6f s).",
              q_map.nn_[0].size(), size_t(n_modified), size_t(n_occluded),
              size_t(n_occupied), size_t(n_empty), t_part.seconds_elapsed());
}

} // namespace naex
