// Map construction, FLANN index maintenance and cloud merging.
#include "naex/map.h"

#include "naex/flann.h"
#include "naex/nearest_neighbors.h"
#include "naex/timer.h"
#include "naex/types.h"
#include <algorithm>
#include <cassert>
#include <cstddef>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <vector>

namespace naex {

Map::Map() {
  updated_indices_.reserve(10000);
  dirty_indices_.reserve(10000);
  cloud_.reserve(DEFAULT_CAPACITY);
  graph_.reserve(DEFAULT_CAPACITY);
}

FlannMat Map::position_matrix(Index start, Index end) {
  if (cloud_.empty()) {
    return FlannMat();
  }
  Index size = (start < end) ? end - start : Index(cloud_.size()) - start;
  return FlannMat(cloud_[start].position_, static_cast<size_t>(size), 3,
                  sizeof(Point));
}

void Map::update_index() {
  Timer t;
  if (!empty()) {
    Lock cloud_lock(cloud_mutex_);
    Lock index_lock(index_mutex_);
    index_ = std::make_shared<FlannIndex>(position_matrix(),
                                          flann::KDTreeSingleIndexParams());
    index_->buildIndex();
    RCLCPP_DEBUG(map_logger(), "Index updated for %lu points (%.3f s).",
                 index_->size(), t.seconds_elapsed());
  } else {
    RCLCPP_INFO(map_logger(), "Index not updated due to empty map.");
  }
}

void Map::update_dirty() {
  Lock lock(dirty_mutex_);
  Timer t;

  update_neighborhood(dirty_indices_.begin(), dirty_indices_.end());
  compute_features(dirty_indices_.begin(), dirty_indices_.end());
  compute_labels(dirty_indices_.begin(), dirty_indices_.end());
  compute_edge_costs(dirty_indices_.begin(), dirty_indices_.end());

  RCLCPP_DEBUG(map_logger(), "%lu points updated (%.3f s).",
               dirty_indices_.size(), t.seconds_elapsed());
}

void Map::clear_updated() {
  Lock lock(updated_mutex_);
  const auto n = updated_indices_.size();
  updated_indices_.clear();
  RCLCPP_DEBUG(map_logger(), "%lu updated indices cleared.", n);
}

void Map::clear_dirty() {
  Lock lock(dirty_mutex_);
  const auto n = dirty_indices_.size();
  dirty_indices_.clear();
  RCLCPP_DEBUG(map_logger(), "%lu dirty indices cleared.", n);
}

std::vector<Index> Map::nearby_indices(Value *origin, Value radius) {
  Lock cloud_lock(cloud_mutex_);
  Lock index_lock(index_mutex_);
  RadiusQuery<Value> q(*index_, FlannMat(origin, 1, 3), radius);
  return q.nn_[0];
}

void Map::initialize(const flann::Matrix<Elem> &points,
                     const flann::Matrix<Elem> &origin) {
  (void)origin;
  Lock cloud_lock(cloud_mutex_);
  assert(cloud_.empty());
  Lock index_lock(index_mutex_);
  Lock dirty_lock(dirty_mutex_);
  Timer t;
  for (size_t i = 0; i < points.rows; ++i) {
    dirty_indices_.insert(static_cast<Index>(cloud_.size()));

    Point point;
    std::copy(points[i], points[i] + points.cols, point.position_);
    point.flags_ |= STATIC;
    cloud_.push_back(point);

    Neighborhood neigh;
    std::copy(points[i], points[i] + points.cols, neigh.position_);
    graph_.push_back(neigh);
  }
  assert(cloud_.size() == points.rows);
  RCLCPP_DEBUG(map_logger(), "%lu dirty indices.", dirty_indices_.size());
  update_index();
  RCLCPP_INFO(map_logger(), "Map initialized with %lu points (%.3f s).",
              points.rows, t.seconds_elapsed());
}

void Map::merge(const flann::Matrix<Elem> &points,
                const flann::Matrix<Elem> &origin) {
  Timer t;
  RCLCPP_DEBUG(map_logger(), "Merging cloud started. Capacity %lu points.",
               capacity());

  if (empty()) {
    initialize(points, origin);
    RCLCPP_INFO(map_logger(), "Map initialized with %lu points (%.3f s).",
                points.rows, t.seconds_elapsed());
    return;
  }

  // TODO: Switch to organized occupancy updates.
  // TODO: Flag dirty points also based on occupancy updates.

  // Find NN distance within current map.
  Lock cloud_lock(cloud_mutex_);
  Lock index_lock(index_mutex_);
  Query<Elem> q(*index_, points, Neighborhood::K_NEIGHBORS,
                neighborhood_radius_);
  RCLCPP_DEBUG(map_logger(), "Got neighbors for %lu points (%.3f s).",
               points.rows, t.seconds_elapsed());

  // Merge points with distance to NN higher than threshold.
  // We'll assume that input points also (approx.) comply to the
  // same min. distance threshold, so each added point can be tested
  // separately.
  Lock added_lock(updated_mutex_);
  Lock dirty_lock(dirty_mutex_);
  Index start = static_cast<Index>(size());
  for (size_t i = 0; i < points.rows; ++i) {
    // Collect dirty indices.
    bool add = true;
    for (size_t j = 0; j < q.dist_.cols; ++j) {
      // Halt once all valid neighbors have been processed.
      // Relevant for radius search with a limit on the num. of neighbors.
      if (!valid_neighbor(q.nn_[i][j], q.dist_[i][j])) {
        break;
      }
      // Neglect points which are not static (or, removed from map).
      if (!(cloud_[q.nn_[i][j]].flags_ & STATIC)) {
        continue;
      }
      // For ordered distances we should encounter minimum first.
      // Don't add the point if it is too close to other points.
      if (q.dist_[i][j] < points_min_dist_ * points_min_dist_) {
        add = false;
        break;
      }
      // Points beyond neighborhood radius are not affected.
      if (q.dist_[i][j] > neighborhood_radius_ * neighborhood_radius_) {
        break;
      }
      // A neighbor of added point within specified distance.
      dirty_indices_.insert(q.nn_[i][j]);
    }
    if (add) {
      updated_indices_.push_back(Index(cloud_.size()));
      dirty_indices_.insert(Index(cloud_.size()));

      Point point;
      std::copy(points[i], points[i] + points.cols, point.position_);
      // TODO: Or only increment occupied flag?
      point.flags_ |= STATIC;
      cloud_.push_back(point);

      Neighborhood neigh;
      std::copy(points[i], points[i] + points.cols, neigh.position_);
      graph_.push_back(neigh);
    }
  }

  // TODO: Rebuild index time to time, don't wait till it doubles in size.
  float rebuild_threshold = (index_->size() + 1000.f) / index_->size();
  index_->addPoints(position_matrix(start), rebuild_threshold);

  RCLCPP_INFO(map_logger(),
              "%lu points merged into map with %lu points "
              "(%.3f s).",
              size_t(size() - start), size_t(size()), t.seconds_elapsed());
}

} // namespace naex
