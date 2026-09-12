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
#include <nav_msgs/msg/path.hpp>
#include <vector>

namespace naex {
namespace grid {

/**
 * Append the cells from @p v0 to @p v1 to @p path_vertices, start first.
 *
 * @p predecessor must be a predecessor map rooted at @p v0, i.e.
 * predecessor[v0] == v0, and @p v1 must be reachable in it.
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

} // namespace grid
} // namespace naex
