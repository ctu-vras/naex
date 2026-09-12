#pragma once

#include "naex/hash.h"
#include <algorithm>
#include <cassert>
#include <cmath>
#include <cstdint>
#include <limits>
#include <unordered_map>
#include <vector>

namespace naex {
namespace grid {

typedef uint32_t CellId;
typedef float Cost;

template <typename T> struct Point2 {
  Point2(T x, T y) : x(x), y(y) {}
  Point2();

  bool operator==(const Point2 &other) const {
    return x == other.x && y == other.y;
  }

  T x;
  T y;
};
template <>
inline Point2<float>::Point2()
    : Point2(std::numeric_limits<float>::quiet_NaN(),
             std::numeric_limits<float>::quiet_NaN()) {}
template <> inline Point2<int16_t>::Point2() : Point2(0, 0) {}

template struct Point2<float>;
template struct Point2<int16_t>;

typedef Point2<float> Point2f;
typedef Point2<int16_t> Point2s;
typedef Point2s Cell;

template <typename T> struct Point2Hasher {
  std::size_t operator()(const Point2<T> &v) const {
    std::size_t seed = 0;
    hash_combine(seed, v.x);
    hash_combine(seed, v.y);
    return seed;
  }
};

template struct Point2Hasher<float>;
template struct Point2Hasher<int16_t>;

typedef Point2Hasher<float> Point2fHasher;
typedef Point2Hasher<int16_t> Point2sHasher;
typedef Point2sHasher CellHasher;

struct Costs {
  /// Number of cost layers.
  static constexpr size_t kSize = 4;

  Costs()
      : data{std::numeric_limits<Cost>::quiet_NaN(),
             std::numeric_limits<Cost>::quiet_NaN(),
             std::numeric_limits<Cost>::quiet_NaN(),
             std::numeric_limits<Cost>::quiet_NaN()} {}
  /// Explicit so that a scalar (e.g. a vertex id) never silently converts to
  /// Costs; see B1 in the 2026-09-12 review.
  explicit Costs(Cost c0,
                 Cost c1 = std::numeric_limits<Cost>::quiet_NaN(),
                 Cost c2 = std::numeric_limits<Cost>::quiet_NaN(),
                 Cost c3 = std::numeric_limits<Cost>::quiet_NaN())
      : data{c0, c1, c2, c3} {}
  Cost data[kSize];

  template <typename T> Costs &operator=(const std::vector<T> &costs) {
    for (size_t i = 0; i < size(); ++i) {
      data[i] = (i < costs.size()) ? costs[i]
                                   : std::numeric_limits<Cost>::quiet_NaN();
    }
    return *this;
  }
  const Cost &operator[](size_t i) const { return data[i]; }
  Cost &operator[](size_t i) { return data[i]; }
  size_t size() const { return kSize; }
  Cost total() const {
    Cost total = 0;
    for (size_t i = 0; i < size(); ++i) {
      if (!std::isnan(data[i])) {
        total += data[i];
      }
    }
    return total;
  }
};

/// Sentinel returned by nearestCell() and Grid::findCell() when there is no
/// such cell, and by Eviction::old_to_new for a cell that was removed.
inline constexpr CellId INVALID_CELL_ID = std::numeric_limits<CellId>::max();

/**
 * Result of a Grid compaction (Grid::eraseCells(), Grid::evictOutside()).
 *
 * This is the contract every holder of a CellId must honour (the planner's
 * ad-hoc dirty list today, P2's neighbour table and P7's plan cache later):
 *
 * * `removed == 0`: nothing was dropped, no cell was renumbered, every CellId
 *   stays valid, `version` is unchanged and `old_to_new` is empty.  A cached
 *   structure may be kept as is.
 * * `removed > 0`: the surviving cells are renumbered, keeping their relative
 *   order, so `old_to_new[old]` is the new CellId of the cell that used to be
 *   `old`, or INVALID_CELL_ID if that cell was dropped, and
 *   `old_to_new[old] <= old` for every survivor.  Everything keyed on a CellId
 *   must either be rebuilt or be remapped through `old_to_new`; `version` is
 *   the Grid::version() after the change, so a cache can simply compare it.
 *
 * Costs and cells move together: `costs(old_to_new[v])` before the call is
 * `costs(v)` after it.
 */
struct Eviction {
  /// Cells in the grid before the compaction.
  size_t before{0};
  /// Cells in the grid after the compaction.
  size_t after{0};
  /// before - after.
  size_t removed{0};
  /// Grid::version() after the compaction.
  uint64_t version{0};
  /// Old CellId to new CellId, empty when removed == 0.  See above.
  std::vector<CellId> old_to_new;

  /// True if CellIds were invalidated, i.e. old_to_new must be honoured.
  bool changed() const { return removed != 0; }
};

// TODO: Move max costs and total to grid.
// TODO: Add costs weights for total.
// TODO: Add required flags for total.
class Grid {
public:
  Grid(float cell_size = 1.f, float forget_factor = 1.f,
       const Costs &default_costs = Costs())
      : cell_size_(cell_size), forget_factor_(forget_factor),
        default_costs_(default_costs) {}

  bool hasCell(const Cell &c) const {
    return cell_to_id_.find(c) != cell_to_id_.end();
  }
  void createCell(const Cell &c) {
    cell_to_id_[c] = size();
    id_to_cell_.push_back(c);
    id_to_costs_.push_back(default_costs_);
    ++version_;
  }

  const Cell &cell(const CellId &id) const {
    assert(id < size());
    return id_to_cell_[id];
  }
  Cell &cell(const CellId &id) {
    assert(id < size());
    return id_to_cell_[id];
  }
  Point2f point(const CellId &id) const { return cellToPoint(cell(id)); }

  CellId &cellId(const Cell &c) {
    if (!hasCell(c)) {
      createCell(c);
    }
    return cell_to_id_[c];
  }
  const CellId &cellId(const Cell &c) const {
    assert(hasCell(c));
    return cell_to_id_.find(c)->second;
  }
  /// CellId of @p c, or INVALID_CELL_ID if the cell does not exist.  One hash
  /// lookup, and it never creates a cell (P3).
  CellId findCell(const Cell &c) const;

  Cell pointToCell(const Point2f &p) const {
    return Cell(std::floor(p.x / cell_size_), std::floor(p.y / cell_size_));
  }
  Point2f cellToPoint(const Cell &c) const {
    return Point2f((c.x + 0.5f) * cell_size_, (c.y + 0.5f) * cell_size_);
  }

  const Costs &costs(const CellId &id) const {
    assert(id < size());
    return id_to_costs_[id];
  }
  Costs &costs(const CellId &id) {
    assert(id < size());
    return id_to_costs_[id];
  }
  Costs &cellCosts(const Cell &c) { return costs(cellId(c)); }
  Costs &pointCosts(const Point2f &p) { return cellCosts(pointToCell(p)); }

  Costs &updateCellCost(Cell c, int level, Cost cost) {
    Costs &costs = cellCosts(c);
    if (std::isfinite(costs.data[level])) {
      Cost w0 = (1. - forget_factor_);
      Cost w1 = forget_factor_;
      costs.data[level] = w0 * costs.data[level] + w1 * cost;
    } else {
      costs.data[level] = cost;
    }
    return costs;
  }
  Costs &updatePointCost(Point2f p, int level, Cost cost) {
    return updateCellCost(pointToCell(p), level, cost);
  }
  float cellSize() const { return cell_size_; }

  bool empty() const { return id_to_costs_.empty(); }
  size_t size() const { return id_to_costs_.size(); }
  void clear() {
    id_to_costs_.clear();
    id_to_cell_.clear();
    cell_to_id_.clear();
    ++version_;
  }

  /**
   * Monotone counter of *structural* changes of the grid.
   *
   * Incremented by createCell(), clear() and by a compaction that actually
   * removed something, i.e. exactly by the operations after which a CellId may
   * mean a different cell than before.  Cost updates do **not** bump it, so a
   * cache keyed on version() must still track cost changes itself (that is the
   * separate counter P7 will need).
   */
  uint64_t version() const { return version_; }

  /**
   * Drop every cell for which @p keep (called with its CellId) is false and
   * renumber the survivors, keeping their relative order.
   *
   * This is the **only** place besides createCell() and clear() that writes
   * id_to_cell_/id_to_costs_, and the single hook for everything that caches a
   * CellId: see the Eviction contract.  O(N) in the grid size plus one rebuild
   * of the cell -> id map.  When @p keep accepts everything the grid is left
   * completely untouched, version included, and only the scratch mapping (one
   * CellId per cell) was allocated.
   *
   * @p keep must not modify the grid.
   */
  template <typename Keep> Eviction eraseCells(Keep keep) {
    Eviction ev;
    const CellId n = static_cast<CellId>(size());
    ev.before = n;
    ev.old_to_new.assign(n, INVALID_CELL_ID);
    CellId kept = 0;
    for (CellId v = 0; v < n; ++v) {
      if (keep(v)) {
        ev.old_to_new[v] = kept++;
      }
    }
    ev.after = kept;
    ev.removed = static_cast<size_t>(n) - kept;
    if (ev.removed == 0) {
      // Nothing moved, so no CellId was invalidated; say so with an empty map.
      ev.old_to_new.clear();
      ev.version = version_;
      return ev;
    }
    // new id <= old id for every survivor, so the compaction is safe in place.
    for (CellId v = 0; v < n; ++v) {
      const CellId w = ev.old_to_new[v];
      if (w == INVALID_CELL_ID) {
        continue;
      }
      id_to_cell_[w] = id_to_cell_[v];
      id_to_costs_[w] = id_to_costs_[v];
    }
    id_to_cell_.resize(kept);
    id_to_costs_.resize(kept);
    cell_to_id_.clear();
    cell_to_id_.reserve(kept);
    for (CellId v = 0; v < kept; ++v) {
      cell_to_id_[id_to_cell_[v]] = v;
    }
    ev.version = ++version_;
    return ev;
  }

  /**
   * Drop every cell farther than @p radius_cells from @p center (Chebyshev
   * distance in cells, i.e. the retained region is a square), per the
   * Eviction contract.
   *
   * Chebyshev and not Euclidean on purpose: the bound on the number of
   * retained cells is then exact ((2 r + 1)^2) and the test is two integer
   * comparisons.  A negative radius keeps nothing.
   */
  Eviction evictOutside(const Cell &center, int32_t radius_cells) {
    const int32_t cx = center.x;
    const int32_t cy = center.y;
    return eraseCells([&](CellId v) {
      const Cell &c = id_to_cell_[v];
      return std::abs(static_cast<int32_t>(c.x) - cx) <= radius_cells &&
             std::abs(static_cast<int32_t>(c.y) - cy) <= radius_cells;
    });
  }

protected:
  float cell_size_;
  float forget_factor_;
  Costs default_costs_;
  /// Structural version; see version().
  uint64_t version_{0};

  // CellId to Costs
  std::vector<Costs> id_to_costs_;
  // CellId to Cell
  std::vector<Cell> id_to_cell_;
  // Cell to CellId
  std::unordered_map<Cell, CellId, CellHasher> cell_to_id_;
};

inline CellId Grid::findCell(const Cell &c) const {
  const auto it = cell_to_id_.find(c);
  return it == cell_to_id_.end() ? INVALID_CELL_ID : it->second;
}

/**
 * Smallest |coordinate|, in metres, that Grid::pointToCell() can no longer
 * convert without overflowing the int16_t cell index (+-13.1 km at 0.4 m).
 */
inline float maxCellCoord(float cell_size) { return 32767.f * cell_size; }

/**
 * True if @p p can be converted to a Cell at all: both coordinates finite and
 * strictly inside the int16_t cell range.
 *
 * Casting a NaN, an infinity or an out-of-range float to int16_t is undefined
 * behaviour; before P6 such a point silently created a phantom cell somewhere
 * in the grid (B8).
 */
inline bool inCellRange(const Grid &grid, const Point2f &p) {
  const float limit = maxCellCoord(grid.cellSize());
  return std::isfinite(p.x) && std::isfinite(p.y) && std::abs(p.x) < limit &&
         std::abs(p.y) < limit;
}

/**
 * True if the map-frame point @p p may be inserted into @p grid: it must pass
 * inCellRange() and, when @p range > 0, lie within @p range of @p origin
 * (2-D Euclidean, boundary inclusive).
 *
 * A @p range <= 0 or NaN disables the crop, and so does a non-finite
 * @p origin (a broken transform must not silently empty the map).  This is the
 * per-point predicate of Planner::receiveCloud(), factored out so that it is
 * testable without a ROS node.
 */
inline bool acceptInputPoint(const Grid &grid, const Point2f &p,
                             const Point2f &origin, float range) {
  if (!inCellRange(grid, p)) {
    return false;
  }
  if (!(range > 0.f) || !std::isfinite(origin.x) || !std::isfinite(origin.y)) {
    return true;
  }
  const float dx = p.x - origin.x;
  const float dy = p.y - origin.y;
  return dx * dx + dy * dy <= range * range;
}

/**
 * Chebyshev radius, in cells, that bounds a @p range metre crop around the
 * robot: the retained square has a half-width of at least @p range.
 *
 * Clamped to the int16_t cell range, so a range larger than the grid can hold
 * keeps everything instead of overflowing.
 */
inline int32_t cellRadius(const Grid &grid, float range) {
  if (!(range > 0.f)) {
    return 0;
  }
  const float cells = std::ceil(range / grid.cellSize());
  if (!(cells < 65536.f)) {
    return 65536;
  }
  return static_cast<int32_t>(cells);
}

/// True if @p layer is a valid index into Costs.
inline bool isValidLayer(int layer) {
  return layer >= 0 && static_cast<size_t>(layer) < Costs::kSize;
}

/**
 * Upper bound on the number of cells a @p radius_cells Chebyshev crop keeps,
 * i.e. (2 r + 1)^2.
 *
 * Returned as a double so that a radius large enough to cover the whole int16
 * cell range cannot overflow; the caller compares it against Grid::size().
 */
inline double boundedCellCount(int32_t radius_cells) {
  if (radius_cells < 0) {
    return 0.0;
  }
  const double side = 2.0 * radius_cells + 1.0;
  return side * side;
}

/**
 * Drop every cell of @p grid farther than @p range metres from @p center
 * (Chebyshev, i.e. the retained region is the square that contains the disc).
 *
 * This is the whole P6b decision in one place: a @p range <= 0 or NaN, or a
 * @p center that is not a valid cell, is a **no-op** — nothing is removed, no
 * CellId is invalidated and Grid::version() does not change, which is exactly
 * the pre-P6 behaviour of an unbounded map.
 */
inline Eviction evictOutsideRange(Grid &grid, const Point2f &center,
                                  float range) {
  if (!(range > 0.f) || !inCellRange(grid, center)) {
    Eviction ev;
    ev.before = grid.size();
    ev.after = grid.size();
    ev.version = grid.version();
    return ev;
  }
  return grid.evictOutside(grid.pointToCell(center), cellRadius(grid, range));
}

/**
 * Set one cost layer of every cell to @p cost.
 *
 * Out-of-range layers are ignored.  Kept separate from the planner so that the
 * ad-hoc layer reset can be unit tested and, later, replaced by a dirty-list
 * reset (P3) in exactly one place.
 */
inline void fillLayer(Grid &grid, int layer, Cost cost) {
  if (!isValidLayer(layer)) {
    return;
  }
  const CellId n = static_cast<CellId>(grid.size());
  for (CellId v = 0; v < n; ++v) {
    grid.costs(v)[static_cast<size_t>(layer)] = cost;
  }
}

/**
 * Set @p cost on @p layer of every existing cell whose centre lies within
 * @p radius of @p center (2-D Euclidean, boundary inclusive).
 *
 * Cells are never created.  Out-of-range layers are ignored.  Every cell that
 * was written is appended to @p touched when that pointer is not null, so the
 * caller can restore exactly those cells later instead of sweeping the whole
 * grid (P3); the same cell may be appended more than once by overlapping discs,
 * which is harmless because restoring is idempotent.
 *
 * Only the cell bounding box of the disc is walked: a cell whose centre is
 * within @p radius of @p center has its centre inside
 * [center - radius, center + radius], and pointToCell() is monotone, so the box
 * corners bracket its index.  The distance test is the same expression as the
 * former full-grid pass, hence the selected set is identical.
 */
inline void applyDiscCost(Grid &grid, int layer, const Point2f &center,
                          float radius, Cost cost,
                          std::vector<CellId> *touched = nullptr) {
  if (!isValidLayer(layer)) {
    return;
  }
  // A non-finite centre or radius selected nothing in the full-grid pass (every
  // comparison against NaN is false); it must not reach the int16_t cast below.
  if (!std::isfinite(center.x) || !std::isfinite(center.y) ||
      !std::isfinite(radius)) {
    return;
  }
  // int32 loop counters: the int16 cell range can be exhausted far from the
  // origin, and lo > hi (nothing to do) must not become an infinite loop.
  const Cell lo = grid.pointToCell({center.x - radius, center.y - radius});
  const Cell hi = grid.pointToCell({center.x + radius, center.y + radius});
  for (int32_t x = lo.x; x <= hi.x; ++x) {
    for (int32_t y = lo.y; y <= hi.y; ++y) {
      const CellId v =
          grid.findCell(Cell(static_cast<int16_t>(x), static_cast<int16_t>(y)));
      if (v == INVALID_CELL_ID) {
        continue;
      }
      const Point2f p = grid.point(v);
      const float dx = p.x - center.x;
      const float dy = p.y - center.y;
      if (std::sqrt(dx * dx + dy * dy) <= radius) {
        grid.costs(v)[static_cast<size_t>(layer)] = cost;
        if (touched) {
          touched->push_back(v);
        }
      }
    }
  }
}

/**
 * Cell whose centre is nearest to @p p among the cells accepted by
 * @p accept(CellId), or INVALID_CELL_ID if none is.
 *
 * Ties go to the lowest CellId, i.e. to the cell created first, matching the
 * strict-less argmin loops this replaces in Planner::plan().
 */
template <typename Accept>
CellId nearestCell(const Grid &grid, const Point2f &p, Accept accept) {
  CellId best = INVALID_CELL_ID;
  float best_dist = std::numeric_limits<float>::infinity();
  const CellId n = static_cast<CellId>(grid.size());
  for (CellId v = 0; v < n; ++v) {
    if (!accept(v)) {
      continue;
    }
    const Point2f q = grid.point(v);
    const float dx = q.x - p.x;
    const float dy = q.y - p.y;
    const float dist = std::sqrt(dx * dx + dy * dy);
    if (dist < best_dist) {
      best = v;
      best_dist = dist;
    }
  }
  return best;
}

} // namespace grid
} // namespace naex
