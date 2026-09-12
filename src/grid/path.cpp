#include "naex/grid/path.h"

#include "naex/clouds.h"
#include "naex/grid/conversions.h"
#include "naex/types.h"
#include <algorithm>
#include <cassert>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <limits>
#include <rclcpp/logging.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

namespace naex {
namespace grid {

void trace_path_vertices(VertexId v0, VertexId v1,
                         const std::vector<VertexId> &predecessor,
                         std::vector<VertexId> &path_vertices) {
  assert(predecessor[v0] == v0);
  VertexId v = v1;
  size_t steps = 0;
  while (v != v0) {
    path_vertices.push_back(v);
    // A self-loop other than v0 is an unreached vertex (boost leaves those
    // as their own predecessor); a longer walk than the map is a cycle.
    if (predecessor[v] == v || ++steps > predecessor.size()) {
      break;
    }
    v = predecessor[v];
  }
  if (v == v0) {
    path_vertices.push_back(v);
  }
  std::reverse(path_vertices.begin(), path_vertices.end());
}

std::vector<VertexId>
trace_path_vertices(VertexId v0, VertexId v1,
                    const std::vector<VertexId> &predecessor) {
  std::vector<VertexId> path_vertices;
  trace_path_vertices(v0, v1, predecessor, path_vertices);
  return path_vertices;
}

void append_path(const std::vector<VertexId> &path_vertices, const Grid &grid,
                 nav_msgs::msg::Path &path) {
  if (path_vertices.empty()) {
    return;
  }
  path.poses.reserve(path.poses.size() + path_vertices.size());
  for (const auto &v : path_vertices) {
    geometry_msgs::msg::PoseStamped pose;
    pose.header.frame_id = path.header.frame_id;
    pose.header.stamp = path.header.stamp;
    pose.pose.position.x = grid.point(v).x;
    pose.pose.position.y = grid.point(v).y;
    pose.pose.position.z = 0.0;
    pose.pose.orientation.w = 1.;
    if (!path.poses.empty()) {
      Vec3 x(pose.pose.position.x - path.poses.back().pose.position.x,
             pose.pose.position.y - path.poses.back().pose.position.y,
             pose.pose.position.z - path.poses.back().pose.position.z);
      x.normalize();
      Vec3 z(0, 0, 1);
      Mat3 m;
      m.col(0) = x;
      m.col(1) = z.cross(x);
      m.col(2) = z;
      Quat q;
      q = m;
      pose.pose.orientation.x = q.x();
      pose.pose.orientation.y = q.y();
      pose.pose.orientation.z = q.z();
      pose.pose.orientation.w = q.w();
    }
    path.poses.push_back(pose);
  }
}

void fill_map_cloud(sensor_msgs::msg::PointCloud2 &cloud, const Grid &grid,
                    const std::vector<Cost> &path_costs,
                    const std::vector<Cost> &f_values) {
  append_field<float>("x", 1, cloud);
  append_field<float>("y", 1, cloud);
  append_field<float>("z", 1, cloud);
  append_field<float>("cost", 1, cloud);
  append_field<float>("path_cost", 1, cloud);
  append_field<float>("f_value", 1, cloud);
  const VertexId num_cells = static_cast<VertexId>(grid.size());
  resize_cloud(cloud, 1, num_cells);

  sensor_msgs::PointCloud2Iterator<float> x_it(cloud, "x");
  sensor_msgs::PointCloud2Iterator<float> cost_it(cloud, "cost");
  sensor_msgs::PointCloud2Iterator<float> path_cost_it(cloud, "path_cost");
  sensor_msgs::PointCloud2Iterator<float> f_value_it(cloud, "f_value");
  for (VertexId v = 0; v < num_cells;
       ++v, ++x_it, ++cost_it, ++path_cost_it, ++f_value_it) {
    const auto p = grid.point(v);
    x_it[0] = p.x;
    x_it[1] = p.y;
    x_it[2] = 0.f;
    cost_it[0] = grid.costs(v).total();
    path_cost_it[0] = path_costs[v];
    f_value_it[0] = f_values[v];
  }
}

std::pair<float, VertexId>
get_nearest_traversable_vertex(const rclcpp::Logger &log, const Grid &grid,
                               const Costs &max_costs, const Vec3 &p0) {
  float best_dist = std::numeric_limits<float>::infinity();
  VertexId best_v = INVALID_VERTEX_ID;
  const VertexId n = static_cast<VertexId>(grid.size());
  for (VertexId v = 0; v < n; ++v) {
    if (!costs_in_bounds(grid.costs(v), max_costs)) {
      continue;
    }
    const Value dist = (to_vec3(grid.point(v)) - p0).norm();
    if (dist < best_dist) {
      best_v = v;
      best_dist = dist;
    }
  }
  if (best_v != INVALID_VERTEX_ID) {
    RCLCPP_INFO(log, "Closest traversable point to start: %s (dist %.3f).",
                format(to_vec3(grid.point(best_v))).c_str(), best_dist);
  } else {
    RCLCPP_ERROR(log, "No traversable points in graph!");
  }
  return std::make_pair(best_dist, best_v);
}

} // namespace grid
} // namespace naex
