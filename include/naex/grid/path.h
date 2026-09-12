#pragma once

/**
 * @file
 * Turning a predecessor map into a nav_msgs/Path.
 *
 * Used by both grid planners; the bodies live in src/grid/path.cpp (nothing
 * here is a template, and append_path pulls Eigen in).
 */

#include "naex/grid/graph.h"
#include "naex/grid/grid.h"
#include "naex/types.h"
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/logger.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <utility>
#include <vector>

namespace naex {
namespace grid {

/**
 * Append the cells from @p v0 to @p v1 to @p path_vertices, start first.
 *
 * @p predecessor must be a predecessor map rooted at @p v0, i.e.
 * predecessor[v0] == v0, and @p v1 must be reachable in it.  A predecessor map
 * is a tree rooted at v0, so the walk back from @p v1 must reach it in at most
 * predecessor.size() steps; more than that means a cycle -- a bug in whatever
 * built the map -- and the walk stops instead of looping forever.
 */
void trace_path_vertices(VertexId v0, VertexId v1,
                         const std::vector<VertexId> &predecessor,
                         std::vector<VertexId> &path_vertices);

/// trace_path_vertices() into a fresh vector.
std::vector<VertexId>
trace_path_vertices(VertexId v0, VertexId v1,
                    const std::vector<VertexId> &predecessor);

/**
 * Append the centres of @p path_vertices to @p path as poses.
 *
 * Every pose but the first is oriented along the segment that reaches it; the
 * first keeps the identity orientation.
 */
void append_path(const std::vector<VertexId> &path_vertices, const Grid &grid,
                 nav_msgs::msg::Path &path);

/**
 * Render @p grid, together with the path costs and f-values of the last
 * search over it, into @p cloud as x/y/z/cost/path_cost/f_value fields.
 *
 * Shared, verbatim, by the rviz-only debug cloud of both grid planners.
 */
void fill_map_cloud(sensor_msgs::msg::PointCloud2 &cloud, const Grid &grid,
                    const std::vector<Cost> &path_costs,
                    const std::vector<Cost> &f_values);

/**
 * Nearest cell to @p p0 whose costs are within @p max_costs, and its distance.
 *
 * O(N) scan, shared by both planners' start-vertex fallback; logs the result
 * (or its absence) at @p log the way both callers always have.
 */
std::pair<float, VertexId>
get_nearest_traversable_vertex(const rclcpp::Logger &log, const Grid &grid,
                               const Costs &max_costs, const Vec3 &p0);

} // namespace grid
} // namespace naex
