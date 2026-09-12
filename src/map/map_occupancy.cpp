// Occupancy updates: which map points were seen through.
#include "naex/map.h"

#include "naex/buffer.h"
#include "naex/clouds.h"
#include "naex/exceptions.h"
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
#include <cstdint>
#include <limits>
#include <geometry_msgs/msg/transform.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <tf2_eigen/tf2_eigen.hpp>
#include <utility>
#include <vector>

namespace naex {

void Map::update_occupancy_organized(
    const sensor_msgs::msg::PointCloud2 &cloud,
    const geometry_msgs::msg::Transform &cloud_to_map_tf) {
  if (cloud.width == 0) {
    throw Exception("Zero cloud width.");
  }
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

  // TODO: Improve efficiency with camera model and cloud structure.
  Eigen::Isometry3f cloud_to_map(tf2::transformToEigen(cloud_to_map_tf));
  Eigen::Isometry3f map_to_cloud = cloud_to_map.inverse(Eigen::Isometry);

  // Create sensor ray directions with an index.
  t_part.reset();
  sensor_msgs::PointCloud2ConstIterator<float> x_begin(cloud, "x");
  std::vector<Value> dirs(3 * n_pts);
  Value *dir_ptr = dirs.data();
  for (auto x_it = x_begin; x_it != x_it.end(); ++x_it, dir_ptr += 3) {
    Vec3Map dir(dir_ptr);
    dir = ConstVec3Map(&x_it[0]).normalized();
  }
  FlannIndex dir_index(FlannMat(dirs.data(), n_pts, 3),
                       flann::KDTreeSingleIndexParams());
  dir_index.buildIndex();
  RCLCPP_INFO(map_logger(), "%lu sensor rays and index created (%.6f s).",
              n_pts, t_part.seconds_elapsed());

  // Find closest map points to sensor origin.
  Lock cloud_lock(cloud_mutex_);
  Lock index_lock(index_mutex_);
  Lock dirty_lock(dirty_mutex_);
  t_part.reset();
  Vec3 origin = cloud_to_map.translation();
  RadiusQuery<Value> q_map(*index_, FlannMat(origin.data(), 1, 3), 10.f);
  RCLCPP_INFO(map_logger(), "%lu closest map points found (%.6f s).",
              q_map.nn_[0].size(), t_part.seconds_elapsed());

  // Test each map point, whether we can see through.
  t_part.reset();
  std::vector<Index> nn_buf(1, INVALID_VERTEX);
  std::vector<Value> dist_buf(1, std::numeric_limits<Value>::quiet_NaN());
  flann::Matrix<Index> nn(nn_buf.data(), 1, 1);
  flann::Matrix<Value> dist(dist_buf.data(), 1, 1);
  flann::SearchParams params;
  params.checks = 64;
  params.cores = 0;
  params.sorted = true;
  typedef std::pair<Index, Value> IVP;
  std::vector<IVP> fan(4,
                       {INVALID_VERTEX, std::numeric_limits<Value>::quiet_NaN()});
  // Comparator for descending order of second member.
  const auto comp = [](const IVP &a, const IVP &b) {
    return a.second > b.second;
  };
  Index n_occupied = 0;
  Index n_empty = 0;
  Index n_above = 0;
  Index n_modified = 0;
  for (const auto i : q_map.nn_[0]) {
    Vec3 p = map_to_cloud * ConstVec3Map(cloud_[i].position_);
    Vec3 dir = p.normalized();
    dir_index.knnSearch(FlannMat(dir.data(), 1, 3), nn, dist, 1, params);
    Index j = nn_buf[0];
    Index r = j / Index(cloud.width);
    Index c = j % Index(cloud.width);
    // Seek containing triangle among incident ones.
    // Test occlusion of the containing triangle.
    Index k = 0;
    fan[k].first =
        r > 0 ? (r - 1) * Index(cloud.width) + c : Index(INVALID_VERTEX);
    fan[k].second = fan[k].first != INVALID_VERTEX
                        ? ConstVec3Map(&dirs[3 * fan[k].first]).dot(dir)
                        : -std::numeric_limits<Value>::infinity();

    ++k;
    fan[k].first =
        c > 0 ? r * Index(cloud.width) + c - 1 : Index(INVALID_VERTEX);
    fan[k].second = fan[k].first != INVALID_VERTEX
                        ? ConstVec3Map(&dirs[3 * fan[k].first]).dot(dir)
                        : -std::numeric_limits<Value>::infinity();

    ++k;
    fan[k].first = c < Index(cloud.width) - 1
                       ? r * Index(cloud.width) + c + 1
                       : Index(INVALID_VERTEX);
    fan[k].second = fan[k].first != INVALID_VERTEX
                        ? ConstVec3Map(&dirs[3 * fan[k].first]).dot(dir)
                        : -std::numeric_limits<Value>::infinity();

    ++k;
    fan[k].first = r < Index(cloud.height) - 1
                       ? (r + 1) * Index(cloud.width) + c
                       : Index(INVALID_VERTEX);
    fan[k].second = fan[k].first != INVALID_VERTEX
                        ? ConstVec3Map(&dirs[3 * fan[k].first]).dot(dir)
                        : -std::numeric_limits<Value>::infinity();

    // Sort nearby indices by cosine distance.
    std::sort(fan.begin(), fan.end(), comp);
    // Construct plane from three points of nearest directions.
    // Test signed distance from the plane to assess occlusion.
    Vec4 plane =
        plane_from_points(ConstVec3Map(&(x_begin + i)[0]),
                          ConstVec3Map(&(x_begin + fan[0].first)[0]),
                          ConstVec3Map(&(x_begin + fan[1].first)[0]));

    // Make sure positive distance is towards sensor at [0, 0, 0],
    // i.e., outside from surface.
    if (plane(3) < 0.) {
      plane = -plane;
    }
    Value signed_dist = plane.dot(e2p(p));
    Value eps = points_min_dist_ / Value(2.);
    if (signed_dist > eps) {
      // New point above surface.
      ++n_above;
    } else if (signed_dist >= -eps) {
      // Known surface measured again.
      if (cloud_[i].num_occupied_ ==
          std::numeric_limits<decltype(cloud_[i].num_occupied_)>::max()) {
        cloud_[i].num_occupied_ /= 2;
        cloud_[i].num_empty_ /= 2;
      }
      ++cloud_[i].num_occupied_;
      ++n_occupied;
    } else {
      // Known surface seen through,
      // indicating it may be noise or it moved somewhere else.
      if (cloud_[i].num_empty_ ==
          std::numeric_limits<decltype(cloud_[i].num_empty_)>::max()) {
        cloud_[i].num_occupied_ /= 2;
        cloud_[i].num_empty_ /= 2;
      }
      ++cloud_[i].num_empty_;
      ++n_empty;
    }
    if (point_empty(cloud_[i])) {
      if (cloud_[i].flags_ & STATIC) {
        cloud_[i].flags_ &= ~STATIC;
        dirty_indices_.insert(i);
        ++n_modified;
      }
    } else {
      if (!(cloud_[i].flags_ & STATIC)) {
        cloud_[i].flags_ |= STATIC;
        dirty_indices_.insert(i);
        ++n_modified;
      }
    }
  }
  RCLCPP_INFO(map_logger(),
              "Occupancy of %lu points updated, %lu modified state, "
              "%lu above, %lu occupied, %lu empty (%.6f s).",
              q_map.nn_[0].size(), size_t(n_modified), size_t(n_above),
              size_t(n_occupied), size_t(n_empty), t_part.seconds_elapsed());
  RCLCPP_INFO(map_logger(), "Occupancy updated (%.3f s).",
              t.seconds_elapsed());
}

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
    if (!std::isfinite(p0(0)) || !std::isfinite(p0(1)) ||
        !std::isfinite(p0(2)))
      continue;
    ConstVec3Map p1(&(x_begin + i1)[0]);
    if (!std::isfinite(p1(0)) || !std::isfinite(p1(1)) ||
        !std::isfinite(p1(2)))
      continue;
    ConstVec3Map p2(&(x_begin + i2)[0]);
    if (!std::isfinite(p2(0)) || !std::isfinite(p2(1)) ||
        !std::isfinite(p2(2)))
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

void Map::update_occupancy_unorganized(const flann::Matrix<Elem> &points,
                                       const flann::Matrix<Elem> &origin) {
  // TODO: Update static (hit) / dynamic (see through) points.
  Timer t_dyn;
  // Compute input point directions.
  ConstVec3Map x_origin(origin[0]);
  Buffer<Value> dirs_buf(points.rows * points.cols);
  FlannMat dirs(dirs_buf.begin(), points.rows, points.cols);
  for (size_t i = 0; i < points.rows; ++i) {
    Vec3Map dir(dirs[i]);
    dir = ConstVec3Map(points[i]) - x_origin;
    const Elem norm = dir.norm();
    if (std::isnan(norm) || std::isinf(norm) || norm < 1e-3) {
      RCLCPP_INFO_THROTTLE(
          map_logger(), clock_, 1000,
          "Could not normalize direction [%.1g, %.1g, %.1g], norm %.1g.",
          dir.x(), dir.y(), dir.z(), norm);
      dir = Vec3::Zero();
    } else {
      dir /= norm;
    }
  }
  // Create index of view directions.
  FlannIndex dirs_index(dirs, flann::KDTreeSingleIndexParams());
  dirs_index.buildIndex();

  // Get map points nearby to check.
  Elem nearby_dist = 25.;
  RadiusQuery<Value> q_nearby(*index_, origin, nearby_dist);
  const size_t n_nearby = q_nearby.nn_[0].size();
  RCLCPP_DEBUG(map_logger(),
               "Found %lu points up to %.1f m from sensor (%.3f s).", n_nearby,
               nearby_dist, t_dyn.seconds_elapsed());
  Buffer<Value> nearby_dirs_buf(3 * n_nearby);
  FlannMat nearby_dirs(nearby_dirs_buf.begin(), n_nearby, 3);
  for (size_t i = 0; i < n_nearby; ++i) {
    const Index v0 = q_nearby.nn_[0][i];
    Vec3Map dir(nearby_dirs[i]);
    dir = ConstVec3Map(&cloud_[v0].position_[0]) - x_origin;
    const Elem norm = dir.norm();
    if (std::isnan(norm) || std::isinf(norm) || norm < 1e-3) {
      RCLCPP_INFO_THROTTLE(
          map_logger(), clock_, 1000,
          "Could not normalize direction [%.1g, %.1g, %.1g], norm %.1g.",
          dir.x(), dir.y(), dir.z(), norm);
      dir = Vec3::Zero();
    } else {
      dir /= norm;
    }
  }
  int k_dirs = 5; // Number of closest rays to check.
  uint16_t max_observations = 15;
  Query<Value> q_dirs(dirs_index, nearby_dirs, k_dirs);
  for (size_t i = 0; i < n_nearby; ++i) {
    const Index v_map = q_nearby.nn_[0][i];
    ConstVec3Map x_map(position(v_map));
    for (Index j = 0; j < k_dirs; ++j) {
      const Index v_new = q_dirs.nn_[i][j];
      // TODO: Avoid segfault v_new > size?
      // Discarding small clouds seems to do the trick?
      if (size_t(v_new) >= points.rows) {
        RCLCPP_WARN_THROTTLE(map_logger(), clock_, 1000,
                             "Skipping invalid NN (%i >= %lu) during merge.",
                             v_new, points.rows);
        continue;
      }
      ConstVec3Map x_new(points[v_new]);
      // Match close neighbors as a single static object.
      const Elem d = (x_new - x_map).norm();
      if (d <= points_min_dist_) {
        // Static / occupied.
        if (cloud_[v_map].num_occupied_ >= max_observations) {
          cloud_[v_map].num_occupied_ /= 2;
          cloud_[v_map].num_empty_ /= 2;
        }
        ++cloud_[v_map].num_occupied_;
        break;
      }
      const Value d_new = (x_new - x_origin).norm();
      if (std::isnan(d_new) || std::isinf(d_new) || d_new < 1e-3) {
        RCLCPP_WARN_THROTTLE(map_logger(), clock_, 1000,
                             "Discarding invalid point [%.1g, %.1g, %.1g], "
                             "distance to origin %.1g.",
                             x_new.x(), x_new.y(), x_new.z(), d_new);
        continue;
      }
      const Elem line_dist =
          (x_new - x_origin).cross(x_origin - x_map).norm() / d_new;
      if (line_dist > points_min_dist_ / 2) {
        continue;
      }
      // Does farther point see through the map surface?
      const Elem d_map = (x_map - x_origin).norm();
      // TODO: To be usable as this normal must have correct orientation.
      ConstVec3Map n_map(normal(v_map));
      Elem cos = n_map.dot(ConstVec3Map(dirs[v_new]));
      if (d_new > d_map && std::abs(cos) > min_empty_cos_) {
        // Dynamic / see through.
        if (cloud_[v_map].num_empty_ >= max_observations) {
          cloud_[v_map].num_occupied_ /= 2;
          cloud_[v_map].num_empty_ /= 2;
        }
        ++cloud_[v_map].num_empty_;
      }
    }
    if (point_empty(cloud_[v_map])) {
      if (cloud_[v_map].flags_ & STATIC) {
        cloud_[v_map].flags_ &= ~STATIC;
      }
    } else {
      if (!(cloud_[v_map].flags_ & STATIC)) {
        cloud_[v_map].flags_ |= STATIC;
      }
    }
  }
  RCLCPP_DEBUG(map_logger(),
               "Checking and updating %lu static/dynamic points nearby: "
               "%.3f s.",
               n_nearby, t_dyn.seconds_elapsed());
}

} // namespace naex
