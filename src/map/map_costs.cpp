// Point predicates and edge costs of the map graph.
#include "naex/map.h"

#include "naex/geom.h"
#include "naex/types.h"
#include <cmath>
#include <cstddef>
#include <limits>
#include <rclcpp/rclcpp.hpp>
#include <vector>

namespace naex {

bool Map::point_empty(const Point &p) const {
  return p.num_empty_ >= min_num_empty_ &&
         Value(p.num_empty_) / p.num_occupied_ >= min_empty_ratio_;
}

bool Map::point_near(Index i, const std::vector<Value> &points,
                     Value radius) const {
  for (size_t j = 0; j + 2 < points.size(); j += 3) {
    if ((ConstVec3Map(cloud_[i].position_) - ConstVec3Map(&points[j]))
            .norm() <= radius) {
      return true;
    }
  }
  return false;
}

Cost Map::compute_edge_cost(const Edge &e) {
  const auto v0 = source(e);
  const auto v1_index = target_index(e);
  const auto v1 = target(e);

  if (v0 == v1) {
    RCLCPP_WARN_THROTTLE(map_logger(), clock_, 1000,
                         "Graph loop at vertex %i.", v0);
    return std::numeric_limits<Cost>::infinity();
  }
  if (!(cloud_[v1].flags_ & TRAVERSABLE) || (cloud_[v1].flags_ & EDGE)) {
    return std::numeric_limits<Cost>::infinity();
  }
  Cost c = graph_[v0].distances_[v1_index];
  if (c > 3 * points_min_dist_) {
    return std::numeric_limits<Cost>::infinity();
  }

  // Check pitch and roll limits.
  const Vec3 forward =
      ConstVec3Map(cloud_[v1].position_) - ConstVec3Map(cloud_[v0].position_);
  const Vec3 left = ConstVec3Map(cloud_[v1].normal_).cross(forward);
  Value pitch = inclination(forward);
  Value roll = inclination(left);
  if (std::abs(pitch) > max_pitch_ || std::abs(roll) > max_roll_) {
    return std::numeric_limits<Cost>::infinity();
  }
  // Initialize with distance computed in NN search.
  // Scale with relative pitch and roll.
  c += c * inclination_penalty_ *
       (std::abs(pitch) / max_pitch_ + std::abs(roll) / max_roll_);
  // Penalize distance to obstacles and other actors.
  if (std::isfinite(cloud_[v1].dist_to_obstacle_)) {
    c *= 1 + std::max(Value(0), 1 - cloud_[v1].dist_to_obstacle_ /
                                        (2 * clearance_radius_));
  }
  // Penalize by edge and obstacle points nearby.
  c *= 1 + Value(cloud_[v1].num_edge_neighbors_) / Neighborhood::K_NEIGHBORS;
  c *= 1 + Value(cloud_[v1].num_obstacle_neighbors_) /
               Neighborhood::K_NEIGHBORS;
  // TODO: Make distance correspond to expected travel time.
  return c;
}

} // namespace naex
