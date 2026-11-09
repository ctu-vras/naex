#pragma once

/**
 * @file
 * Saving and reloading the planner's cost grid (crash recovery).
 *
 * Everything here is free of ROS so that it can be unit tested without a node;
 * Planner owns the policy (when to save, when a load may proceed).
 *
 * The cells are stored in the map frame they were observed in, together with
 * the pose of that frame in a global (earth) frame.  They are deliberately not
 * converted to the earth frame: ECEF coordinates are ~6.4e6 m, which neither
 * Point2f nor an int16_t cell index can hold.
 */

#include "naex/grid/grid.h"
#include <array>
#include <cstdint>
#include <string>
#include <vector>

namespace naex {
namespace grid {

/// Pose of a frame: translation x, y, z then quaternion x, y, z, w.
typedef std::array<double, 7> MapPose;

/**
 * Everything that decides what a stored cost means.
 *
 * The grid stores weighted costs per layer, so a map saved under a different
 * configuration is not merely less accurate but means something else.
 */
struct MapConfig {
  float cell_size{0.f};
  std::string earth_frame;
  std::vector<std::string> cost_fields;
  std::vector<int64_t> cloud_levels;
  std::vector<double> cloud_weights;
  Costs default_costs;
};

/// Parallel arrays: costs[i] belongs to cells[i].
struct MapCells {
  std::vector<Cell> cells;
  std::vector<Costs> costs;
};

/// Contents of a map file.
struct MapSnapshot {
  MapConfig config;
  /// Informational: the map frame may legitimately differ between runs.
  std::string map_frame;
  /// Pose of the map frame in config.earth_frame when the map was saved.
  MapPose earth_from_map{};
  MapCells cells;
};

/// 2-D rigid transform: x' = c x - s y + tx, y' = s x + c y + ty.
struct Rigid2 {
  double c{1.0};
  double s{0.0};
  double tx{0.0};
  double ty{0.0};
};

/**
 * Empty if a map saved under @p saved can be used under @p current, otherwise
 * a description of the first difference.
 */
std::string describe_config_mismatch(const MapConfig &saved,
                                     const MapConfig &current);

/**
 * Write @p map to @p path, creating its parent directories.
 *
 * Crash safe: the data goes to `path.tmp`, is fsynced and is then renamed over
 * @p path, so @p path always holds a complete map, the old one or the new one.
 *
 * @throw std::runtime_error when any step fails; @p path is left untouched.
 */
void write_map_file(const std::string &path, const MapSnapshot &map);

/**
 * Read a file written by write_map_file().
 *
 * @throw std::runtime_error when it is unreadable, not a map file, of another
 *        format version, truncated or has trailing bytes.
 */
MapSnapshot read_map_file(const std::string &path);

/// Copy of every cell of @p grid in CellId order.
MapCells copy_cells(const Grid &grid);

/**
 * Horizontal part of the transform from the old map frame to the new one, given
 * the pose of both in the same earth frame.
 */
Rigid2 relative_rigid2(const MapPose &earth_from_old,
                       const MapPose &earth_from_new);

/**
 * Express @p old (cells of size @p cell_size) on the cell lattice of a frame
 * related to it by @p new_from_old.
 *
 * Between runs the map frame origin is generally not a whole number of cells
 * away, so the lattices do not line up.  Every target cell whose centre falls
 * into an old cell takes that cell's costs (nearest neighbour, so a shift of at
 * most half a cell), which leaves no holes for any rotation, unlike moving the
 * old cells across.  An identity transform reproduces @p old exactly.
 *
 * Target cells outside the range Grid can address are dropped.  Runs in
 * O(cells), however sparse the map is.
 */
MapCells resample_cells(const MapCells &old, float cell_size,
                        const Rigid2 &new_from_old);

/**
 * Add @p cells to @p grid.  A cell that already exists keeps its costs: it has
 * been observed in this run, which is newer than anything in a file.
 *
 * CellIds already in the grid stay valid.
 *
 * @return the number of cells added.
 */
size_t merge_cells(Grid &grid, const MapCells &cells);

} // namespace grid
} // namespace naex
