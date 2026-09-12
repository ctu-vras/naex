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

/**
 * Perfect hash for a Cell (P10): the two int16_t coordinates are packed into
 * the low 32 bits, so distinct cells never collide and the hash is two shifts
 * and an or instead of two std::hash calls plus hash_combine.
 *
 * libstdc++ reduces modulo a prime bucket count, so neighbouring cells in y
 * land in neighbouring buckets, which is what the grid inserts in bulk.
 */
template <> struct Point2Hasher<int16_t> {
  std::size_t operator()(const Point2<int16_t> &v) const {
    return (static_cast<std::size_t>(static_cast<uint16_t>(v.x)) << 16) |
           static_cast<std::size_t>(static_cast<uint16_t>(v.y));
  }
};

template struct Point2Hasher<float>;
// No explicit instantiation of Point2Hasher<int16_t>: it is explicitly
// specialized above.

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
  explicit Costs(Cost c0, Cost c1 = std::numeric_limits<Cost>::quiet_NaN(),
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

/// Sentinel returned by nearest_cell() and Grid::find_cell() when there is no
/// such cell, and by Eviction::old_to_new for a cell that was removed.
inline constexpr CellId INVALID_CELL_ID = std::numeric_limits<CellId>::max();

/**
 * Number of neighbour slots per cell in the flat neighbour table (P2).
 *
 * Always 8, also for the 4-neighbourhood: neighbor8(c, 2 * i) == neighbor4(c,
 * i), so the 4-connected search reads only the even slots and one table serves
 * both neighbourhoods.
 */
inline constexpr size_t kNbrStride = 8;

/// The i-th 4-neighbour of @p source (0:+x, 1:+y, 2:-x, 3:-y).
inline Cell neighbor4(const Cell &source, int i) {
  Cell target = source;
  switch (i) {
  case 0:
    target.x += 1;
    break;
  case 1:
    target.y += 1;
    break;
  case 2:
    target.x -= 1;
    break;
  case 3:
    target.y -= 1;
    break;
  default:
    assert(false);
  }
  return target;
}

/**
 * The i-th 8-neighbour of @p source: 0:+x, 1:+x+y, 2:+y, 3:-x+y, 4:-x, 5:-x-y,
 * 6:-y, 7:+x-y.
 *
 * The ordering is antipodal, i.e. neighbor8(neighbor8(c, i), (i + 4) & 7) == c,
 * which is what lets the neighbour table patch the reverse link of a new cell
 * without a second lookup.
 */
inline Cell neighbor8(const Cell &source, int i) {
  Cell target = source;
  switch (i) {
  case 0:
    target.x += 1;
    break;
  case 1:
    target.x += 1;
    target.y += 1;
    break;
  case 2:
    target.y += 1;
    break;
  case 3:
    target.x -= 1;
    target.y += 1;
    break;
  case 4:
    target.x -= 1;
    break;
  case 5:
    target.x -= 1;
    target.y -= 1;
    break;
  case 6:
    target.y -= 1;
    break;
  case 7:
    target.x += 1;
    target.y -= 1;
    break;
  default:
    assert(false);
  }
  return target;
}

/**
 * Length of the i-th 8-neighbour step in cells (P2).
 *
 * std::sqrt is not constexpr before C++26, hence the literal; Graph.Distance8
 * pins that it rounds to the same float as std::sqrt(2.f).
 */
inline constexpr Cost kDist8[kNbrStride] = {
    1.f, 1.41421356237309504880f, 1.f, 1.41421356237309504880f,
    1.f, 1.41421356237309504880f, 1.f, 1.41421356237309504880f};

inline Cost distance8(int i) {
  assert(i >= 0 && i < 8);
  return kDist8[i];
}

/**
 * Result of a Grid compaction (Grid::erase_cells(), Grid::evict_outside()).
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

  bool has_cell(const Cell &c) const {
    return cell_to_id_.find(c) != cell_to_id_.end();
  }
  /**
   * Append a cell that does not exist yet, wiring it into the neighbour table.
   *
   * One hash insert plus the 8 lookups of the neighbour resolution; the reverse
   * links of the neighbours found are patched in place, so the table stays
   * exact without ever being rebuilt (P2).
   */
  void create_cell(const Cell &c) {
    const auto res = cell_to_id_.try_emplace(c, static_cast<CellId>(size()));
    assert(res.second && "create_cell called on an existing cell");
    if (!res.second) {
      return;
    }
    append_cell(c, res.first->second);
  }

  const Cell &cell(const CellId &id) const {
    assert(id < size());
    return id_to_cell_[id];
  }
  Cell &cell(const CellId &id) {
    assert(id < size());
    return id_to_cell_[id];
  }
  Point2f point(const CellId &id) const { return cell_to_point(cell(id)); }

  /**
   * CellId of @p c, creating the cell if it does not exist.
   *
   * One hash lookup on a hit and one insert on a miss (P10); the pre-P10
   * version cost two lookups on a hit and three on a miss.
   */
  CellId &cell_id(const Cell &c) {
    const auto res = cell_to_id_.try_emplace(c, static_cast<CellId>(size()));
    if (res.second) {
      append_cell(c, res.first->second);
    }
    return res.first->second;
  }
  const CellId &cell_id(const Cell &c) const {
    assert(has_cell(c));
    return cell_to_id_.find(c)->second;
  }
  /// CellId of @p c, or INVALID_CELL_ID if the cell does not exist.  One hash
  /// lookup, and it never creates a cell (P3).
  CellId find_cell(const Cell &c) const;

  Cell point_to_cell(const Point2f &p) const {
    return Cell(std::floor(p.x / cell_size_), std::floor(p.y / cell_size_));
  }
  Point2f cell_to_point(const Cell &c) const {
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
  Costs &cell_costs(const Cell &c) { return costs(cell_id(c)); }
  Costs &point_costs(const Point2f &p) { return cell_costs(point_to_cell(p)); }

  /**
   * Blend @p cost into layer @p level of the cell already resolved to @p id.
   *
   * The id-taking half of update_cell_cost(), split out so a caller that
   * already knows the CellId (the "last cell" cache of Planner::receive_cloud)
   * does not pay a second hash lookup for it.
   */
  Costs &update_cost_at(CellId id, int level, Cost cost) {
    Costs &costs = this->costs(id);
    if (std::isfinite(costs.data[level])) {
      Cost w0 = (1. - forget_factor_);
      Cost w1 = forget_factor_;
      costs.data[level] = w0 * costs.data[level] + w1 * cost;
    } else {
      costs.data[level] = cost;
    }
    return costs;
  }
  Costs &update_cell_cost(Cell c, int level, Cost cost) {
    return update_cost_at(cell_id(c), level, cost);
  }
  Costs &update_point_cost(Point2f p, int level, Cost cost) {
    return update_cell_cost(point_to_cell(p), level, cost);
  }
  float cell_size() const { return cell_size_; }

  bool empty() const { return id_to_costs_.empty(); }
  size_t size() const { return id_to_costs_.size(); }
  void clear() {
    id_to_costs_.clear();
    id_to_cell_.clear();
    cell_to_id_.clear();
    nbr_.clear();
    ++version_;
  }

  /**
   * Reserve room for @p n cells in every container, the neighbour table
   * included (P10).
   *
   * Only grows; a smaller @p n is ignored.  Worth calling before a bulk insert
   * (the first cloud), where the rehashing of cell_to_id_ is otherwise the
   * dominant cost of ingestion.
   */
  void reserve(size_t n) {
    if (n <= size()) {
      return;
    }
    id_to_costs_.reserve(n);
    id_to_cell_.reserve(n);
    nbr_.reserve(kNbrStride * n);
    cell_to_id_.reserve(n);
  }

  /**
   * CellId of the i-th 8-neighbour of cell @p id, or INVALID_CELL_ID when that
   * neighbour does not exist (P2).
   *
   * Flat table, no hash lookup: this is the Dijkstra inner loop.  For the
   * 4-neighbourhood pass 2 * i, see kNbrStride.
   */
  CellId neighbor_id(CellId id, int i) const {
    assert(id < size());
    assert(i >= 0 && i < static_cast<int>(kNbrStride));
    return nbr_[kNbrStride * id + static_cast<size_t>(i)];
  }

  /**
   * Monotone counter of *structural* changes of the grid.
   *
   * Incremented by create_cell(), clear() and by a compaction that actually
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
   * This is the **only** place besides create_cell() and clear() that writes
   * id_to_cell_/id_to_costs_, and the single hook for everything that caches a
   * CellId: see the Eviction contract.  O(N) in the grid size plus one rebuild
   * of the cell -> id map.  When @p keep accepts everything the grid is left
   * completely untouched, version included, and only the scratch mapping (one
   * CellId per cell) was allocated.
   *
   * @p keep must not modify the grid.
   */
  template <typename Keep> Eviction erase_cells(Keep keep) {
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
    remap_neighbors(ev);
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
  Eviction evict_outside(const Cell &center, int32_t radius_cells) {
    const int32_t cx = center.x;
    const int32_t cy = center.y;
    return erase_cells([&](CellId v) {
      const Cell &c = id_to_cell_[v];
      return std::abs(static_cast<int32_t>(c.x) - cx) <= radius_cells &&
             std::abs(static_cast<int32_t>(c.y) - cy) <= radius_cells;
    });
  }

protected:
  /**
   * Append @p c, already inserted into cell_to_id_ with id @p id, to the cell
   * and cost vectors and wire it into the neighbour table.
   *
   * The table is exact after this call: the 8 neighbours of @p c that already
   * exist are written into its own row, and @p id is written into the
   * antipodal slot of each of their rows.  A neighbour created later patches
   * the link from its own side, so no rebuild is ever needed.
   */
  void append_cell(const Cell &c, CellId id) {
    id_to_cell_.push_back(c);
    id_to_costs_.push_back(default_costs_);
    nbr_.resize(nbr_.size() + kNbrStride, INVALID_CELL_ID);
    for (int i = 0; i < static_cast<int>(kNbrStride); ++i) {
      const CellId n = find_cell(neighbor8(c, i));
      if (n != INVALID_CELL_ID) {
        nbr_[kNbrStride * id + static_cast<size_t>(i)] = n;
        // neighbor8 is antipodal: c is the ((i + 4) & 7)-th neighbour of n.
        nbr_[kNbrStride * n + static_cast<size_t>((i + 4) & 7)] = id;
      }
    }
    ++version_;
  }

  /**
   * Renumber the neighbour table in place through the mapping of @p ev, which
   * must describe a compaction that actually removed something.
   *
   * O(N), hash-free and allocation-free: old_to_new already yields
   * INVALID_CELL_ID for a neighbour that was evicted, so the boundary of the
   * retained region needs no special case, and new id <= old id makes the pass
   * safe in place — the same argument erase_cells() uses for the cells
   * themselves.  Keeping the capacity also matters: a grid that is compacted
   * again and again while it regrows would otherwise reallocate a
   * multi-megabyte block per eviction.  This lives inside erase_cells() so that
   * no caller can forget it (P2 / the Eviction contract).
   */
  void remap_neighbors(const Eviction &ev) {
    for (CellId v = 0; v < static_cast<CellId>(ev.before); ++v) {
      const CellId w = ev.old_to_new[v];
      if (w == INVALID_CELL_ID) {
        continue;
      }
      for (size_t i = 0; i < kNbrStride; ++i) {
        // w <= v, so the destination row is at or below the source row and the
        // value is read before it can be overwritten.
        const CellId n = nbr_[kNbrStride * v + i];
        nbr_[kNbrStride * w + i] =
            (n == INVALID_CELL_ID) ? INVALID_CELL_ID : ev.old_to_new[n];
      }
    }
    nbr_.resize(kNbrStride * ev.after);
  }

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
  /**
   * Flat neighbour table (P2): nbr_[kNbrStride * id + i] is the CellId of
   * neighbor8(cell(id), i), or INVALID_CELL_ID when that cell does not exist.
   *
   * Maintained incrementally by append_cell(), dropped by clear() and remapped
   * by erase_cells() — the three (and only three) writers of id_to_cell_.
   * 32 B per cell, i.e. 6.9 MB at 216 k cells.
   */
  std::vector<CellId> nbr_;
};

inline CellId Grid::find_cell(const Cell &c) const {
  const auto it = cell_to_id_.find(c);
  return it == cell_to_id_.end() ? INVALID_CELL_ID : it->second;
}

/// Largest cell index a Cell (a pair of int16_t) can hold.
inline constexpr float kMaxCellIndex = 32767.f;

/**
 * Chebyshev radius, in cells, that already covers the whole int16_t cell
 * range; cell_radius() clamps to it instead of overflowing.
 */
inline constexpr int32_t kMaxCellRadius = 65536;

/**
 * Smallest |coordinate|, in metres, that Grid::point_to_cell() can no longer
 * convert without overflowing the int16_t cell index (+-13.1 km at 0.4 m).
 */
inline float max_cell_coord(float cell_size) {
  return kMaxCellIndex * cell_size;
}

/**
 * True if @p p can be converted to a Cell at all: both coordinates finite and
 * strictly inside the int16_t cell range.
 *
 * Casting a NaN, an infinity or an out-of-range float to int16_t is undefined
 * behaviour; before P6 such a point silently created a phantom cell somewhere
 * in the grid (B8).
 */
inline bool in_cell_range(const Grid &grid, const Point2f &p) {
  const float limit = max_cell_coord(grid.cell_size());
  return std::isfinite(p.x) && std::isfinite(p.y) && std::abs(p.x) < limit &&
         std::abs(p.y) < limit;
}

/**
 * True if the map-frame point @p p may be inserted into @p grid: it must pass
 * in_cell_range() and, when @p range > 0, lie within @p range of @p origin
 * (2-D Euclidean, boundary inclusive).
 *
 * A @p range <= 0 or NaN disables the crop, and so does a non-finite
 * @p origin (a broken transform must not silently empty the map).  This is the
 * per-point predicate of Planner::receive_cloud(), factored out so that it is
 * testable without a ROS node.
 */
inline bool accept_input_point(const Grid &grid, const Point2f &p,
                               const Point2f &origin, float range) {
  if (!in_cell_range(grid, p)) {
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
inline int32_t cell_radius(const Grid &grid, float range) {
  if (!(range > 0.f)) {
    return 0;
  }
  const float cells = std::ceil(range / grid.cell_size());
  if (!(cells < static_cast<float>(kMaxCellRadius))) {
    return kMaxCellRadius;
  }
  return static_cast<int32_t>(cells);
}

/**
 * True if every bounded layer of @p costs is within @p max_costs.
 *
 * A non-finite entry of @p max_costs means "this layer is not bounded" and is
 * skipped; the remaining layers are still checked, so an all-NaN max bounds
 * nothing while [inf, 0.8, NaN, NaN] does bound layer 1.  (The pre-astar
 * version stopped at the first non-finite entry, which silently unbounded
 * every layer behind an unbounded one.)
 *
 * Free function (P2) so that the planner can test the robot cell without
 * constructing a Graph: a Graph caches the per-vertex result and would be
 * stale once the ad-hoc layer is rewritten.
 */
inline bool costs_in_bounds(const Costs &costs, const Costs &max_costs) {
  for (size_t i = 0; i < Costs::kSize; ++i) {
    // Skip layers without a finite bound.
    if (!std::isfinite(max_costs[i])) {
      continue;
    }
    if (!(costs[i] <= max_costs[i])) {
      return false;
    }
  }
  return true;
}

/// True if @p layer is a valid index into Costs.
inline bool is_valid_layer(int layer) {
  return layer >= 0 && static_cast<size_t>(layer) < Costs::kSize;
}

/**
 * Upper bound on the number of cells a @p radius_cells Chebyshev crop keeps,
 * i.e. (2 r + 1)^2.
 *
 * Returned as a double so that a radius large enough to cover the whole int16
 * cell range cannot overflow; the caller compares it against Grid::size().
 */
inline double bounded_cell_count(int32_t radius_cells) {
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
inline Eviction evict_outside_range(Grid &grid, const Point2f &center,
                                    float range) {
  if (!(range > 0.f) || !in_cell_range(grid, center)) {
    Eviction ev;
    ev.before = grid.size();
    ev.after = grid.size();
    ev.version = grid.version();
    return ev;
  }
  return grid.evict_outside(grid.point_to_cell(center),
                            cell_radius(grid, range));
}

/**
 * Set one cost layer of every cell to @p cost.
 *
 * Out-of-range layers are ignored.  Kept separate from the planner so that the
 * ad-hoc layer reset can be unit tested and, later, replaced by a dirty-list
 * reset (P3) in exactly one place.
 */
inline void fill_layer(Grid &grid, int layer, Cost cost) {
  if (!is_valid_layer(layer)) {
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
 * [center - radius, center + radius], and point_to_cell() is monotone, so
 * the box corners bracket its index.  The distance test is the same
 * expression as the former full-grid pass, hence the selected set is
 * identical.
 */
inline void apply_disc_cost(Grid &grid, int layer, const Point2f &center,
                            float radius, Cost cost,
                            std::vector<CellId> *touched = nullptr) {
  if (!is_valid_layer(layer)) {
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
  const Cell lo = grid.point_to_cell({center.x - radius, center.y - radius});
  const Cell hi = grid.point_to_cell({center.x + radius, center.y + radius});
  for (int32_t x = lo.x; x <= hi.x; ++x) {
    for (int32_t y = lo.y; y <= hi.y; ++y) {
      const CellId v = grid.find_cell(
          Cell(static_cast<int16_t>(x), static_cast<int16_t>(y)));
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
CellId nearest_cell(const Grid &grid, const Point2f &p, Accept accept) {
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
