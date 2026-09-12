// Conversion of the map point buffer to a PointCloud2 message.
#include "naex/map.h"

#include "naex/clouds.h"
#include "naex/types.h"
#include <algorithm>
#include <cassert>
#include <cstddef>
#include <cstdint>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

namespace naex {

void Map::initialize_cloud(sensor_msgs::msg::PointCloud2 &cloud) const {
  cloud.point_step = uint32_t(offsetof(Point, position_));
  append_field<decltype(Point().position_[0])>("x", 1, cloud);
  append_field<decltype(Point().position_[0])>("y", 1, cloud);
  append_field<decltype(Point().position_[0])>("z", 1, cloud);

  cloud.point_step = uint32_t(offsetof(Point, normal_));
  append_field<decltype(Point().normal_[0])>("normal_x", 1, cloud);
  append_field<decltype(Point().normal_[0])>("normal_y", 1, cloud);
  append_field<decltype(Point().normal_[0])>("normal_z", 1, cloud);

  cloud.point_step = uint32_t(offsetof(Point, normal_support_));
  append_field<decltype(Point().normal_support_)>("normal_support", 1, cloud);

  cloud.point_step = uint32_t(offsetof(Point, ground_diff_std_));
  append_field<decltype(Point().ground_diff_std_)>("ground_diff_std", 1,
                                                   cloud);

  cloud.point_step = uint32_t(offsetof(Point, min_ground_diff_));
  append_field<decltype(Point().min_ground_diff_)>("min_ground_diff", 1,
                                                   cloud);

  cloud.point_step = uint32_t(offsetof(Point, max_ground_diff_));
  append_field<decltype(Point().max_ground_diff_)>("max_ground_diff", 1,
                                                   cloud);

  cloud.point_step = uint32_t(offsetof(Point, mean_abs_ground_diff_));
  append_field<decltype(Point().mean_abs_ground_diff_)>(
      "mean_abs_ground_diff", 1, cloud);

  cloud.point_step = uint32_t(offsetof(Point, viewpoint_));
  append_field<decltype(Point().viewpoint_[0])>("viewpoint_x", 1, cloud);
  append_field<decltype(Point().viewpoint_[0])>("viewpoint_y", 1, cloud);
  append_field<decltype(Point().viewpoint_[0])>("viewpoint_z", 1, cloud);

  cloud.point_step = uint32_t(offsetof(Point, dist_to_actor_));
  append_field<decltype(Point().dist_to_actor_)>("dist_to_actor", 1, cloud);

  cloud.point_step = uint32_t(offsetof(Point, actor_last_visit_));
  append_field<decltype(Point().actor_last_visit_)>("actor_last_visit", 1,
                                                    cloud);

  cloud.point_step = uint32_t(offsetof(Point, dist_to_other_actors_));
  append_field<decltype(Point().dist_to_other_actors_)>(
      "dist_to_other_actors", 1, cloud);

  cloud.point_step = uint32_t(offsetof(Point, other_actors_last_visit_));
  append_field<decltype(Point().other_actors_last_visit_)>(
      "other_actors_last_visit", 1, cloud);

  cloud.point_step = uint32_t(offsetof(Point, coverage_));
  append_field<decltype(Point().coverage_)>("coverage", 1, cloud);

  cloud.point_step = uint32_t(offsetof(Point, self_coverage_));
  append_field<decltype(Point().self_coverage_)>("self_coverage", 1, cloud);

  cloud.point_step = uint32_t(offsetof(Point, dist_to_obstacle_));
  append_field<decltype(Point().dist_to_obstacle_)>("dist_to_obstacle", 1,
                                                    cloud);

  cloud.point_step = uint32_t(offsetof(Point, flags_));
  append_field<decltype(Point().flags_)>("flags", 1, cloud);

  cloud.point_step = uint32_t(offsetof(Point, num_empty_));
  append_field<decltype(Point().num_empty_)>("num_empty", 1, cloud);

  cloud.point_step = uint32_t(offsetof(Point, num_occupied_));
  append_field<decltype(Point().num_occupied_)>("num_occupied", 1, cloud);

  cloud.point_step = uint32_t(offsetof(Point, dist_to_plane_));
  append_field<decltype(Point().dist_to_plane_)>("dist_to_plane", 1, cloud);

  cloud.point_step = uint32_t(offsetof(Point, num_obstacle_pts_));
  append_field<decltype(Point().num_obstacle_pts_)>("num_obstacle_pts", 1,
                                                    cloud);

  cloud.point_step = uint32_t(offsetof(Point, num_obstacle_neighbors_));
  append_field<decltype(Point().num_obstacle_neighbors_)>(
      "num_obstacle_neighbors", 1, cloud);

  cloud.point_step = uint32_t(offsetof(Point, num_edge_neighbors_));
  append_field<decltype(Point().num_edge_neighbors_)>("num_edge_neighbors", 1,
                                                      cloud);

  cloud.point_step = uint32_t(offsetof(Point, path_cost_));
  append_field<decltype(Point().path_cost_)>("path_cost", 1, cloud);

  cloud.point_step = uint32_t(offsetof(Point, reward_));
  append_field<decltype(Point().reward_)>("reward", 1, cloud);

  cloud.point_step = uint32_t(offsetof(Point, relative_cost_));
  append_field<decltype(Point().relative_cost_)>("relative_cost", 1, cloud);

  cloud.point_step = uint32_t(sizeof(Point));
}

void Map::create_cloud_msg(sensor_msgs::msg::PointCloud2 &cloud) {
  initialize_cloud(cloud);
  sensor_msgs::PointCloud2Modifier modifier(cloud);
  Lock cloud_lock(cloud_mutex_);
  modifier.resize(cloud_.size());
  const auto from = reinterpret_cast<const uint8_t *>(cloud_.data());
  const auto to =
      reinterpret_cast<const uint8_t *>(cloud_.data() + cloud_.size());
  auto out = cloud.data.data();
  assert(size_t(to - from) == cloud.data.size());
  std::copy(from, to, out);
}

} // namespace naex
