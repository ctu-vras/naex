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
  }

protected:
  float cell_size_;
  float forget_factor_;
  Costs default_costs_;

  // CellId to Costs
  std::vector<Costs> id_to_costs_;
  // CellId to Cell
  std::vector<Cell> id_to_cell_;
  // Cell to CellId
  std::unordered_map<Cell, CellId, CellHasher> cell_to_id_;
};

/// Sentinel returned by nearestCell() when no cell was accepted.
inline constexpr CellId INVALID_CELL_ID = std::numeric_limits<CellId>::max();

/// True if @p layer is a valid index into Costs.
inline bool isValidLayer(int layer) {
  return layer >= 0 && static_cast<size_t>(layer) < Costs::kSize;
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
 * Cells are never created.  Out-of-range layers are ignored.  Currently a full
 * grid pass; P3 replaces it with a bounding-box iteration, which must select
 * exactly the same cells (see the SidelobeDisc tests).
 */
inline void applyDiscCost(Grid &grid, int layer, const Point2f &center,
                          float radius, Cost cost) {
  if (!isValidLayer(layer)) {
    return;
  }
  const CellId n = static_cast<CellId>(grid.size());
  for (CellId v = 0; v < n; ++v) {
    const Point2f p = grid.point(v);
    const float dx = p.x - center.x;
    const float dy = p.y - center.y;
    if (std::sqrt(dx * dx + dy * dy) <= radius) {
      grid.costs(v)[static_cast<size_t>(layer)] = cost;
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
