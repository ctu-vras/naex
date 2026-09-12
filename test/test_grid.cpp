// Unit tests for the grid planner data structures.
#include "naex/grid/graph.h"
#include "naex/grid/grid.h"
#include "naex/grid/search.h"

#include <gtest/gtest.h>

#include <cmath>
#include <cstdlib>
#include <limits>
#include <random>
#include <set>
#include <utility>
#include <vector>

using naex::grid::Cell;
using naex::grid::Cost;
using naex::grid::Costs;
using naex::grid::Grid;
using naex::grid::Graph;
using naex::grid::Point2f;
using naex::grid::ShortestPaths;
using naex::grid::VertexId;

namespace {
constexpr Cost kNaN = std::numeric_limits<Cost>::quiet_NaN();
}  // namespace

TEST(Grid, PointToCellRoundTrip) {
  const Grid grid(0.4f);
  for (int16_t x = -5; x <= 5; ++x) {
    for (int16_t y = -5; y <= 5; ++y) {
      const Cell c(x, y);
      const Point2f p = grid.cellToPoint(c);
      const Cell back = grid.pointToCell(p);
      EXPECT_EQ(c.x, back.x) << "x = " << x << ", y = " << y;
      EXPECT_EQ(c.y, back.y) << "x = " << x << ", y = " << y;
    }
  }
}

TEST(Grid, PointToCellNegatives) {
  const Grid grid(0.4f);
  // Floor, not truncation: everything in [-0.4, 0) maps to cell -1.
  EXPECT_EQ(grid.pointToCell({-0.1f, -0.1f}).x, -1);
  EXPECT_EQ(grid.pointToCell({-0.1f, -0.1f}).y, -1);
  EXPECT_EQ(grid.pointToCell({-0.4f, -0.4f}).x, -1);
  EXPECT_EQ(grid.pointToCell({-0.41f, 0.0f}).x, -2);
  EXPECT_EQ(grid.pointToCell({0.0f, 0.0f}).x, 0);
  EXPECT_EQ(grid.pointToCell({0.39f, 0.39f}).x, 0);
  EXPECT_EQ(grid.pointToCell({0.41f, 0.41f}).x, 1);
  // Cell centres.
  EXPECT_FLOAT_EQ(grid.cellToPoint(Cell(-1, -1)).x, -0.2f);
  EXPECT_FLOAT_EQ(grid.cellToPoint(Cell(0, 0)).x, 0.2f);
}

TEST(Costs, DefaultIsAllNaN) {
  const Costs c;
  for (size_t i = 0; i < Costs::kSize; ++i) {
    EXPECT_TRUE(std::isnan(c[i])) << "layer " << i;
  }
  EXPECT_FLOAT_EQ(c.total(), 0.f);
}

TEST(Costs, TotalSkipsNaN) {
  const Costs c(1.f, kNaN, 2.f, kNaN);
  EXPECT_FLOAT_EQ(c.total(), 3.f);
  const Costs all(1.f, 2.f, 3.f, 4.f);
  EXPECT_FLOAT_EQ(all.total(), 10.f);
}

TEST(Costs, AssignFromVectorPadsWithNaN) {
  Costs c;
  c = std::vector<float>{1.f, 2.f};
  EXPECT_FLOAT_EQ(c[0], 1.f);
  EXPECT_FLOAT_EQ(c[1], 2.f);
  EXPECT_TRUE(std::isnan(c[2]));
  EXPECT_TRUE(std::isnan(c[3]));
}

TEST(Grid, UpdateCellCostForgetFactor) {
  // Default costs are NaN, so the first update takes the value verbatim.
  Grid grid(1.f, 0.5f);
  const Cell c(0, 0);
  EXPECT_FLOAT_EQ(grid.updateCellCost(c, 0, 2.f)[0], 2.f);
  // Then w0 * old + w1 * new with w1 = forget_factor.
  EXPECT_FLOAT_EQ(grid.updateCellCost(c, 0, 4.f)[0], 3.f);
  EXPECT_FLOAT_EQ(grid.updateCellCost(c, 0, 4.f)[0], 3.5f);

  // forget_factor == 1 keeps only the newest value.
  Grid fresh(1.f, 1.f);
  EXPECT_FLOAT_EQ(fresh.updateCellCost(c, 0, 2.f)[0], 2.f);
  EXPECT_FLOAT_EQ(fresh.updateCellCost(c, 0, 7.f)[0], 7.f);
}

TEST(Grid, UpdatePointCostCreatesCell) {
  Grid grid(1.f, 1.f);
  EXPECT_TRUE(grid.empty());
  grid.updatePointCost({0.5f, 0.5f}, 0, 1.f);
  EXPECT_EQ(grid.size(), 1u);
  grid.updatePointCost({0.6f, 0.6f}, 0, 1.f);
  EXPECT_EQ(grid.size(), 1u) << "same cell must not be duplicated";
  grid.updatePointCost({-0.5f, 0.5f}, 0, 1.f);
  EXPECT_EQ(grid.size(), 2u);
}

TEST(Graph, Neighbor4) {
  const Cell s(3, 5);
  EXPECT_EQ(naex::grid::neighbor4(s, 0).x, 4);
  EXPECT_EQ(naex::grid::neighbor4(s, 0).y, 5);
  EXPECT_EQ(naex::grid::neighbor4(s, 1).x, 3);
  EXPECT_EQ(naex::grid::neighbor4(s, 1).y, 6);
  EXPECT_EQ(naex::grid::neighbor4(s, 2).x, 2);
  EXPECT_EQ(naex::grid::neighbor4(s, 2).y, 5);
  EXPECT_EQ(naex::grid::neighbor4(s, 3).x, 3);
  EXPECT_EQ(naex::grid::neighbor4(s, 3).y, 4);
}

TEST(Graph, Neighbor8IsDistinctAndAdjacent) {
  const Cell s(0, 0);
  std::set<std::pair<int, int>> seen;
  for (int i = 0; i < 8; ++i) {
    const Cell t = naex::grid::neighbor8(s, i);
    EXPECT_LE(std::abs(t.x), 1);
    EXPECT_LE(std::abs(t.y), 1);
    EXPECT_FALSE(t.x == 0 && t.y == 0) << "index " << i;
    EXPECT_TRUE(seen.insert({t.x, t.y}).second) << "duplicate at index " << i;
  }
  EXPECT_EQ(seen.size(), 8u);
  // Even indices of neighbor8 are the neighbor4 offsets.
  for (int i = 0; i < 4; ++i) {
    EXPECT_EQ(naex::grid::neighbor8(s, 2 * i).x, naex::grid::neighbor4(s, i).x);
    EXPECT_EQ(naex::grid::neighbor8(s, 2 * i).y, naex::grid::neighbor4(s, i).y);
  }
}

TEST(Graph, Distance8) {
  for (int i = 0; i < 8; ++i) {
    const Cost expected = (i % 2 == 0) ? 1.f : std::sqrt(2.f);
    EXPECT_FLOAT_EQ(naex::grid::distance8(i), expected) << "index " << i;
  }
}

TEST(Graph, CostsInBoundsStopsAtNaNMax) {
  Grid grid(1.f, 1.f);
  // Bound layer 0 only; NaN in layer 1 stops the check, so layers 2 and 3 are
  // not bounded even though a finite max is given for layer 3.
  const Graph graph(grid, 8, Costs(1.f, kNaN, 0.f, 0.f));
  EXPECT_TRUE(graph.costsInBounds(Costs(0.5f, 100.f, 100.f, 100.f)));
  EXPECT_TRUE(graph.costsInBounds(Costs(1.f)));
  EXPECT_FALSE(graph.costsInBounds(Costs(1.5f)));
  // NaN cost never compares <= max, so it is out of bounds.
  EXPECT_FALSE(graph.costsInBounds(Costs(kNaN)));

  // An all-NaN max bounds nothing.
  const Graph unbounded(grid, 8, Costs());
  EXPECT_TRUE(unbounded.costsInBounds(Costs(1e6f, 1e6f, 1e6f, 1e6f)));
}

TEST(ShortestPaths, ThreeByThreeWithBlockedCell) {
  // 3x3 grid of unit cells, all free except the centre.
  Grid grid(1.f, 1.f, Costs(0.f));
  VertexId id[3][3];
  for (int16_t x = 0; x < 3; ++x) {
    for (int16_t y = 0; y < 3; ++y) {
      id[x][y] = grid.cellId(Cell(x, y));
    }
  }
  ASSERT_EQ(grid.size(), 9u);
  grid.cellCosts(Cell(1, 1))[0] = 5.f;

  const Costs max_costs(1.f);
  const VertexId start = id[0][0];
  const ShortestPaths sp(grid, start, 4, max_costs);

  EXPECT_FLOAT_EQ(sp.pathCost(start), 0.f);
  EXPECT_EQ(sp.predecessor(start), start);

  // The blocked cell is unreachable.
  EXPECT_FALSE(std::isfinite(sp.pathCost(id[1][1])));

  // Each free step costs 1 + (0 + 0) / 2 times the cell size.
  EXPECT_FLOAT_EQ(sp.pathCost(id[1][0]), 1.f);
  EXPECT_FLOAT_EQ(sp.pathCost(id[0][1]), 1.f);
  EXPECT_FLOAT_EQ(sp.pathCost(id[2][0]), 2.f);
  // Around the blocked centre: 4 steps instead of a diagonal.
  EXPECT_FLOAT_EQ(sp.pathCost(id[2][2]), 4.f);

  // Predecessor chain from the far corner reaches the start.
  VertexId v = id[2][2];
  size_t steps = 0;
  while (v != start && steps < grid.size()) {
    const VertexId p = sp.predecessor(v);
    EXPECT_NE(p, v) << "stuck at vertex " << v;
    EXPECT_NE(p, id[1][1]) << "path must not enter the blocked cell";
    v = p;
    ++steps;
  }
  EXPECT_EQ(v, start);
  EXPECT_EQ(steps, 4u);
}

TEST(ShortestPaths, AllBlockedIsUnreachable) {
  Grid grid(1.f, 1.f, Costs(10.f));
  const VertexId a = grid.cellId(Cell(0, 0));
  const VertexId b = grid.cellId(Cell(1, 0));
  const ShortestPaths sp(grid, a, 8, Costs(1.f));
  EXPECT_FLOAT_EQ(sp.pathCost(a), 0.f);
  EXPECT_FALSE(std::isfinite(sp.pathCost(b)));
}

// --- Regression guards for the performance work ---------------------------
// The tests below pin behaviour that P2, P3, P8 and P9 must preserve.  They
// are deliberately written against exact cell sets and exact path costs, not
// against timings.

namespace {

/// Dense square grid of cells [0, n) x [0, n) with all-zero costs on every
/// layer (Costs(0.f) alone would leave layers 1..3 NaN), created in x-major
/// order so that CellIds (and therefore tie-breaking) are stable.
Grid makeDenseGrid(int16_t n, float cell_size = 1.f) {
  Grid grid(cell_size, 1.f, Costs(0.f, 0.f, 0.f, 0.f));
  for (int16_t x = 0; x < n; ++x) {
    for (int16_t y = 0; y < n; ++y) {
      grid.cellId(Cell(x, y));
    }
  }
  return grid;
}

}  // namespace

TEST(AdHocLayer, FillLayerTouchesOnlyThatLayer) {
  Grid grid = makeDenseGrid(3);
  naex::grid::fillLayer(grid, 3, 7.f);
  for (naex::grid::CellId v = 0; v < grid.size(); ++v) {
    EXPECT_FLOAT_EQ(grid.costs(v)[3], 7.f);
    EXPECT_FLOAT_EQ(grid.costs(v)[0], 0.f);
  }
  // Out-of-range layers are a no-op, not a crash or an out-of-bounds write.
  naex::grid::fillLayer(grid, -1, 1.f);
  naex::grid::fillLayer(grid, static_cast<int>(Costs::kSize), 1.f);
  EXPECT_FLOAT_EQ(grid.costs(0)[3], 7.f);
}

TEST(SidelobeDisc, UnitGridPlusShapeInclusiveBoundary) {
  // 5x5 unit cells, centres at (x + 0.5, y + 0.5).  A radius of exactly 1.0
  // around the centre of cell (2, 2) includes the four edge neighbours (d = 1,
  // boundary is inclusive) and excludes the diagonals (d = sqrt(2)).
  Grid grid = makeDenseGrid(5);
  const size_t size_before = grid.size();
  naex::grid::applyDiscCost(grid, 3, Point2f(2.5f, 2.5f), 1.0f, 10.f);
  EXPECT_EQ(grid.size(), size_before) << "no cell may be created";

  const std::set<std::pair<int, int>> expected = {
      {2, 2}, {1, 2}, {3, 2}, {2, 1}, {2, 3}};
  std::set<std::pair<int, int>> affected;
  for (naex::grid::CellId v = 0; v < grid.size(); ++v) {
    if (grid.costs(v)[3] == 10.f) {
      affected.insert({grid.cell(v).x, grid.cell(v).y});
    } else {
      EXPECT_FLOAT_EQ(grid.costs(v)[3], 0.f);
    }
  }
  EXPECT_EQ(affected, expected);
}

TEST(SidelobeDisc, ProductionSizedLobeHitsTwelveCells) {
  // cell_size 0.4 m and sidelobes_radius 0.81 m, i.e. the production sizing.
  // Cell centres sit at +-0.2, +-0.6, +-1.0 ...; the radius is chosen to fall
  // between 0.632 (0.6, 0.2) and 0.848 (0.6, 0.6) so that no cell is near the
  // boundary and the expected set is unambiguous.
  Grid grid(0.4f, 1.f, Costs(0.f, 0.f, 0.f, 0.f));
  for (int16_t x = -5; x <= 4; ++x) {
    for (int16_t y = -5; y <= 4; ++y) {
      grid.cellId(Cell(x, y));
    }
  }
  const size_t size_before = grid.size();
  naex::grid::applyDiscCost(grid, 3, Point2f(0.f, 0.f), 0.81f, 10.f);
  EXPECT_EQ(grid.size(), size_before);

  std::set<std::pair<int, int>> affected;
  for (naex::grid::CellId v = 0; v < grid.size(); ++v) {
    if (grid.costs(v)[3] == 10.f) {
      affected.insert({grid.cell(v).x, grid.cell(v).y});
    }
  }
  const std::set<std::pair<int, int>> expected = {
      {-1, -1}, {-1, 0}, {0, -1}, {0, 0},    // |cx| = |cy| = 0.2
      {-2, -1}, {-2, 0}, {1, -1}, {1, 0},    // |cx| = 0.6, |cy| = 0.2
      {-1, -2}, {0, -2}, {-1, 1}, {0, 1}};   // |cx| = 0.2, |cy| = 0.6
  EXPECT_EQ(affected.size(), 12u);
  EXPECT_EQ(affected, expected);
}

namespace {

// Reference implementation of applyDiscCost(): the full-grid pass P3 replaced.
// Kept verbatim so the bounding-box walk can be compared against it cell by
// cell on random grids.
void referenceApplyDiscCost(Grid &grid, int layer, const Point2f &center,
                            float radius, Cost cost,
                            std::vector<naex::grid::CellId> *touched) {
  if (!naex::grid::isValidLayer(layer)) {
    return;
  }
  const naex::grid::CellId n =
      static_cast<naex::grid::CellId>(grid.size());
  for (naex::grid::CellId v = 0; v < n; ++v) {
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

}  // namespace

TEST(SidelobeDisc, SparseGridOnlyTouchesExistingCells) {
  // 0.4 m cells covering [-2, 2) in both axes, with (0, 0) -- the cell the disc
  // is centred on -- deliberately missing.
  Grid grid(0.4f, 1.f, Costs(0.f, 0.f, 0.f, 0.f));
  for (int16_t x = -5; x <= 4; ++x) {
    for (int16_t y = -5; y <= 4; ++y) {
      if (x == 0 && y == 0) {
        continue;
      }
      grid.cellId(Cell(x, y));
    }
  }
  const size_t size_before = grid.size();
  std::vector<naex::grid::CellId> dirty;
  naex::grid::applyDiscCost(grid, 3, Point2f(0.f, 0.f), 0.81f, 10.f, &dirty);
  EXPECT_EQ(grid.size(), size_before) << "the hole must not be created";
  EXPECT_FALSE(grid.hasCell(Cell(0, 0)));

  std::set<std::pair<int, int>> affected;
  for (naex::grid::CellId v = 0; v < grid.size(); ++v) {
    if (grid.costs(v)[3] == 10.f) {
      affected.insert({grid.cell(v).x, grid.cell(v).y});
    }
  }
  // ProductionSizedLobeHitsTwelveCells' set minus the missing (0, 0).
  const std::set<std::pair<int, int>> expected = {
      {-1, -1}, {-1, 0}, {0, -1},
      {-2, -1}, {-2, 0}, {1, -1}, {1, 0},
      {-1, -2}, {0, -2}, {-1, 1}, {0, 1}};
  EXPECT_EQ(affected, expected);
  EXPECT_EQ(dirty.size(), expected.size());
}

TEST(SidelobeDisc, DirtyListRestoresExactly) {
  // Distinct per-cell values on layer 3 stand in for "anything but the
  // default"; after the dirty-list clear every cell must be back at the
  // default and no cell outside the two discs may have been written at all.
  Grid grid = makeDenseGrid(10, 0.4f);
  const Cost kDefault = 0.25f;
  for (naex::grid::CellId v = 0; v < grid.size(); ++v) {
    grid.costs(v)[3] = kDefault;
    grid.costs(v)[2] = static_cast<Cost>(v);
  }
  std::vector<Cost> before;
  for (naex::grid::CellId v = 0; v < grid.size(); ++v) {
    before.push_back(grid.costs(v)[2]);
  }

  std::vector<naex::grid::CellId> dirty;
  naex::grid::applyDiscCost(grid, 3, Point2f(1.0f, 1.0f), 0.81f, 10.f, &dirty);
  naex::grid::applyDiscCost(grid, 3, Point2f(1.2f, 1.0f), 0.81f, 7.f, &dirty);
  ASSERT_FALSE(dirty.empty());
  // The discs overlap, so the dirty list holds duplicates; restoring twice is
  // idempotent, which is exactly what the planner relies on.
  const std::set<naex::grid::CellId> unique(dirty.begin(), dirty.end());
  EXPECT_LT(unique.size(), dirty.size()) << "the two discs must overlap";

  size_t written = 0;
  for (naex::grid::CellId v = 0; v < grid.size(); ++v) {
    if (grid.costs(v)[3] != kDefault) {
      ++written;
      EXPECT_EQ(unique.count(v), 1u) << "cell " << v << " written but not dirty";
    }
  }
  EXPECT_EQ(written, unique.size());

  for (const naex::grid::CellId v : dirty) {
    grid.costs(v)[3] = kDefault;
  }
  for (naex::grid::CellId v = 0; v < grid.size(); ++v) {
    EXPECT_FLOAT_EQ(grid.costs(v)[3], kDefault) << "cell " << v;
    EXPECT_FLOAT_EQ(grid.costs(v)[2], before[v]) << "other layer touched";
  }
}

TEST(SidelobeDisc, FarFromOriginDoesNotWrap) {
  // Cell index 30000 at cell_size 0.4, i.e. close to the int16_t limit: the
  // bounding box must not wrap around, and the result must equal the full scan.
  const float kCenter = 12000.0f;
  Grid grid(0.4f, 1.f, Costs(0.f, 0.f, 0.f, 0.f));
  Grid reference(0.4f, 1.f, Costs(0.f, 0.f, 0.f, 0.f));
  for (int32_t x = 29995; x <= 30005; ++x) {
    for (int32_t y = -5; y <= 5; ++y) {
      grid.cellId(Cell(static_cast<int16_t>(x), static_cast<int16_t>(y)));
      reference.cellId(Cell(static_cast<int16_t>(x), static_cast<int16_t>(y)));
    }
  }
  ASSERT_EQ(grid.pointToCell({kCenter, 0.f}).x, 30000);

  std::vector<naex::grid::CellId> dirty;
  naex::grid::applyDiscCost(grid, 3, Point2f(kCenter, 0.f), 0.81f, 10.f,
                            &dirty);
  referenceApplyDiscCost(reference, 3, Point2f(kCenter, 0.f), 0.81f, 10.f,
                         nullptr);
  ASSERT_EQ(grid.size(), reference.size());
  for (naex::grid::CellId v = 0; v < grid.size(); ++v) {
    EXPECT_FLOAT_EQ(grid.costs(v)[3], reference.costs(v)[3])
        << "cell (" << grid.cell(v).x << ", " << grid.cell(v).y << ")";
  }
  EXPECT_FALSE(dirty.empty());
}

TEST(SidelobeDisc, MatchesBruteForceOnRandomGrid) {
  // The bounding-box walk must select bit for bit the same cells as the
  // full-grid pass it replaces, on a sparse grid with holes, for a range of
  // centres and radii (including degenerate ones).
  std::mt19937 rng(12345u);
  std::uniform_real_distribution<float> coord(-6.f, 6.f);
  std::uniform_real_distribution<float> radius(0.f, 2.5f);
  std::bernoulli_distribution present(0.7);

  Grid grid(0.4f, 1.f, Costs(0.f, 0.f, 0.f, 0.f));
  Grid reference(0.4f, 1.f, Costs(0.f, 0.f, 0.f, 0.f));
  for (int16_t x = -20; x <= 20; ++x) {
    for (int16_t y = -20; y <= 20; ++y) {
      if (!present(rng)) {
        continue;
      }
      grid.cellId(Cell(x, y));
      reference.cellId(Cell(x, y));
    }
  }
  ASSERT_GT(grid.size(), 800u);
  ASSERT_EQ(grid.size(), reference.size());
  const size_t size_before = grid.size();

  for (int trial = 0; trial < 200; ++trial) {
    const Point2f center(coord(rng), coord(rng));
    const float r = radius(rng);
    const Cost cost = static_cast<Cost>(trial);
    std::vector<naex::grid::CellId> dirty;
    std::vector<naex::grid::CellId> ref_dirty;
    naex::grid::applyDiscCost(grid, 3, center, r, cost, &dirty);
    referenceApplyDiscCost(reference, 3, center, r, cost, &ref_dirty);
    ASSERT_EQ(grid.size(), size_before) << "no cell may be created";
    ASSERT_EQ(dirty, ref_dirty)
        << "trial " << trial << " centre (" << center.x << ", " << center.y
        << ") radius " << r;
    for (naex::grid::CellId v = 0; v < grid.size(); ++v) {
      ASSERT_FLOAT_EQ(grid.costs(v)[3], reference.costs(v)[3])
          << "trial " << trial << ", cell " << v;
    }
  }

  // Degenerate inputs selected nothing before and must select nothing now.
  const float kNaNf = std::numeric_limits<float>::quiet_NaN();
  std::vector<naex::grid::CellId> dirty;
  naex::grid::applyDiscCost(grid, 3, Point2f(0.f, 0.f), -1.f, 99.f, &dirty);
  naex::grid::applyDiscCost(grid, 3, Point2f(kNaNf, 0.f), 1.f, 99.f, &dirty);
  naex::grid::applyDiscCost(grid, 3, Point2f(0.f, 0.f), kNaNf, 99.f, &dirty);
  EXPECT_TRUE(dirty.empty());
  EXPECT_EQ(grid.size(), size_before);
}

TEST(NearestCell, EmptyPredicateAndTieBreak) {
  Grid grid = makeDenseGrid(3);
  EXPECT_EQ(naex::grid::nearestCell(grid, Point2f(0.f, 0.f),
                                    [](naex::grid::CellId) { return false; }),
            naex::grid::INVALID_CELL_ID);
  // Equidistant from the centres of (0, 0) and (1, 0); the lower CellId, i.e.
  // the cell created first, wins.
  const naex::grid::CellId v = naex::grid::nearestCell(
      grid, Point2f(1.0f, 0.5f), [](naex::grid::CellId) { return true; });
  EXPECT_EQ(grid.cell(v).x, 0);
  EXPECT_EQ(grid.cell(v).y, 0);
}

TEST(Planning, TwentyByTwentyWithGapExactCost) {
  // 20x20 unit cells, free cost 0, wall on column x = 10 for y in [0, 17].
  // The only way across is the gap at (10, 18) / (10, 19).
  Grid grid = makeDenseGrid(20);
  for (int16_t y = 0; y <= 17; ++y) {
    grid.cellCosts(Cell(10, y))[0] = 5.f;
  }
  const Costs max_costs(1.f);
  const VertexId start = grid.cellId(Cell(0, 0));
  const VertexId goal = grid.cellId(Cell(19, 0));
  const ShortestPaths sp(grid, start, 8, max_costs);

  // Every free step costs cell_size * (1 + (0 + 0) / 2) * distance8, so an
  // optimal route is 19 diagonal and 17 straight steps: through (10, 18)
  // it is 10*sqrt(2) + 8 to the gap and 9*sqrt(2) + 9 back down.  Going
  // through (10, 19) instead would cost 19*sqrt(2) + 19.
  const float expected = 19.f * std::sqrt(2.f) + 17.f;
  // Dijkstra accumulates 36 float additions, so compare with a tolerance far
  // below the 2.0 cost difference to the next-best route.
  EXPECT_NEAR(sp.pathCost(goal), expected, 1e-3f);

  // The wall itself is unreachable and no optimal path enters it.
  EXPECT_FALSE(std::isfinite(sp.pathCost(grid.cellId(Cell(10, 5)))));
  size_t vertices = 1;
  VertexId v = goal;
  while (v != start && vertices <= grid.size()) {
    EXPECT_FLOAT_EQ(grid.costs(v)[0], 0.f) << "path entered a blocked cell";
    v = sp.predecessor(v);
    ++vertices;
  }
  ASSERT_EQ(v, start);
  // 19 diagonal + 17 straight steps is the only integer solution of
  // a*sqrt(2) + b = 19*sqrt(2) + 17, so the pose count is exact too.
  EXPECT_EQ(vertices, 37u);
}

TEST(Planning, NearestReachableCellWithUnreachableGoal) {
  // Same grid, but the wall now closes the full height: everything at x > 10
  // is unreachable, and the goal at (19.5, 0.5) is behind it.
  Grid grid = makeDenseGrid(20);
  for (int16_t y = 0; y < 20; ++y) {
    grid.cellCosts(Cell(10, y))[0] = 5.f;
  }
  const VertexId start = grid.cellId(Cell(0, 0));
  const ShortestPaths sp(grid, start, 8, Costs(1.f));
  ASSERT_FALSE(std::isfinite(sp.pathCost(grid.cellId(Cell(19, 0)))));

  const naex::grid::CellId v1 =
      naex::grid::nearestCell(grid, Point2f(19.5f, 0.5f),
                              [&sp](naex::grid::CellId v) {
                                return std::isfinite(sp.pathCost(v));
                              });
  ASSERT_NE(v1, naex::grid::INVALID_CELL_ID);
  EXPECT_EQ(grid.cell(v1).x, 9) << "nearest reachable cell to the goal";
  EXPECT_EQ(grid.cell(v1).y, 0);

  // With nothing reachable the scan must report failure rather than cell 0.
  Grid blocked = makeDenseGrid(3);
  for (naex::grid::CellId v = 0; v < blocked.size(); ++v) {
    blocked.costs(v)[0] = 5.f;
  }
  const ShortestPaths none(blocked, 0, 8, Costs(1.f));
  const naex::grid::CellId unreachable = naex::grid::nearestCell(
      blocked, Point2f(2.5f, 2.5f), [&none](naex::grid::CellId v) {
        return v != 0 && std::isfinite(none.pathCost(v));
      });
  EXPECT_EQ(unreachable, naex::grid::INVALID_CELL_ID);
}

// --- P6: input guards and the map bound ------------------------------------

namespace {

/// The per-point acceptance of Planner::receiveCloud(), without ROS: insert
/// every accepted point of @p points into @p grid on layer 0.  Mirrors the
/// loop in planner.h, so it pins the ingestion contract rather than the helper.
size_t ingestPoints(Grid &grid, const std::vector<Point2f> &points,
                    const Point2f &origin, float input_range) {
  size_t skipped = 0;
  for (const Point2f &p : points) {
    if (!naex::grid::acceptInputPoint(grid, p, origin, input_range)) {
      ++skipped;
      continue;
    }
    grid.updatePointCost(p, 0, 1.f);
  }
  return skipped;
}

}  // namespace

TEST(InputGuard, NonFinitePointsCreateNoCell) {
  // Casting NaN/Inf to int16_t is undefined behaviour and used to create a
  // phantom cell wherever the conversion happened to land (B8).
  const float kInf = std::numeric_limits<float>::infinity();
  Grid grid(0.4f, 1.f, Costs(0.f, 0.f, 0.f, 0.f));
  const std::vector<Point2f> bad = {
      Point2f(kNaN, 0.f),  Point2f(0.f, kNaN),  Point2f(kNaN, kNaN),
      Point2f(kInf, 0.f),  Point2f(0.f, -kInf), Point2f(-kInf, kInf)};
  EXPECT_EQ(ingestPoints(grid, bad, Point2f(0.f, 0.f), 0.f), bad.size());
  EXPECT_EQ(grid.size(), 0u);
  EXPECT_TRUE(grid.empty());

  // One good point still lands, so the guard is not simply rejecting all.
  EXPECT_EQ(ingestPoints(grid, {Point2f(1.f, 1.f)}, Point2f(0.f, 0.f), 0.f),
            0u);
  EXPECT_EQ(grid.size(), 1u);
}

TEST(InputGuard, OutOfInt16RangePointsCreateNoCell) {
  // At cell_size 0.4 the int16_t cell index runs out at +-13106.8 m.
  Grid grid(0.4f, 1.f, Costs(0.f, 0.f, 0.f, 0.f));
  const float limit = naex::grid::maxCellCoord(grid.cellSize());
  EXPECT_FLOAT_EQ(limit, 32767.f * 0.4f);

  const std::vector<Point2f> outside = {
      Point2f(limit, 0.f),         Point2f(-limit, 0.f),
      Point2f(0.f, limit),         Point2f(0.f, -limit),
      Point2f(2.f * limit, 0.f),   Point2f(0.f, -1e9f)};
  EXPECT_EQ(ingestPoints(grid, outside, Point2f(0.f, 0.f), 0.f),
            outside.size());
  EXPECT_EQ(grid.size(), 0u);

  // Just inside the limit is still accepted, and lands where it should.
  const float inside = std::nextafter(limit, 0.f);
  EXPECT_EQ(ingestPoints(grid, {Point2f(inside, -inside)}, Point2f(0.f, 0.f),
                         0.f),
            0u);
  ASSERT_EQ(grid.size(), 1u);
  EXPECT_EQ(grid.cell(0).x, 32766);
  EXPECT_EQ(grid.cell(0).y, -32767);
}

TEST(InputGuard, InputRangeCropKeepsExactlyTheDisc) {
  // 1 m lattice over [-10, 10]^2, crop radius 5 m around (2, -1): exactly the
  // lattice points with |p - origin| <= 5 (boundary inclusive) may create a
  // cell, and each must land in its own cell.
  const Point2f origin(2.f, -1.f);
  const float range = 5.f;
  std::vector<Point2f> points;
  std::set<std::pair<int, int>> expected;
  for (int x = -10; x <= 10; ++x) {
    for (int y = -10; y <= 10; ++y) {
      const Point2f p(static_cast<float>(x), static_cast<float>(y));
      points.push_back(p);
      const float dx = p.x - origin.x;
      const float dy = p.y - origin.y;
      if (dx * dx + dy * dy <= range * range) {
        expected.insert({x, y});
      }
    }
  }
  ASSERT_EQ(points.size(), 441u);
  ASSERT_EQ(expected.size(), 81u) << "lattice points inside a radius 5 disc";

  Grid grid(1.f, 1.f, Costs(0.f, 0.f, 0.f, 0.f));
  EXPECT_EQ(ingestPoints(grid, points, origin, range),
            points.size() - expected.size());
  ASSERT_EQ(grid.size(), expected.size());
  std::set<std::pair<int, int>> got;
  for (naex::grid::CellId v = 0; v < grid.size(); ++v) {
    got.insert({grid.cell(v).x, grid.cell(v).y});
  }
  EXPECT_EQ(got, expected);

  // A boundary point exactly at range is kept; a hair further out is not.
  EXPECT_TRUE(naex::grid::acceptInputPoint(
      grid, Point2f(origin.x + range, origin.y), origin, range));
  EXPECT_FALSE(naex::grid::acceptInputPoint(
      grid, Point2f(std::nextafter(origin.x + range, 100.f), origin.y), origin,
      range));

  // range <= 0 and NaN disable the crop; every lattice point then lands.
  for (const float off : {0.f, -1.f, kNaN}) {
    Grid all(1.f, 1.f, Costs(0.f, 0.f, 0.f, 0.f));
    EXPECT_EQ(ingestPoints(all, points, origin, off), 0u) << "range " << off;
    EXPECT_EQ(all.size(), points.size()) << "range " << off;
  }
  // A broken (non-finite) origin must not silently empty the map either.
  Grid no_origin(1.f, 1.f, Costs(0.f, 0.f, 0.f, 0.f));
  EXPECT_EQ(ingestPoints(no_origin, points, Point2f(kNaN, kNaN), range), 0u);
  EXPECT_EQ(no_origin.size(), points.size());
}

TEST(Eviction, EvictOutsideKeepsExactlyTheSquare) {
  // 20x20 unit cells, evict around (10, 10) with a Chebyshev radius of 3:
  // exactly the 7x7 square [7, 13]^2 survives, renumbered but in order, and
  // every cell keeps its own costs.
  Grid grid = makeDenseGrid(20);
  for (naex::grid::CellId v = 0; v < grid.size(); ++v) {
    grid.costs(v)[0] = static_cast<Cost>(grid.cell(v).x * 100 + grid.cell(v).y);
  }
  const uint64_t version_before = grid.version();
  std::vector<Cell> cells_before;
  std::vector<Cost> costs_before;
  for (naex::grid::CellId v = 0; v < grid.size(); ++v) {
    cells_before.push_back(grid.cell(v));
    costs_before.push_back(grid.costs(v)[0]);
  }

  const naex::grid::Eviction ev = grid.evictOutside(Cell(10, 10), 3);
  EXPECT_EQ(ev.before, 400u);
  EXPECT_EQ(ev.after, 49u);
  EXPECT_EQ(ev.removed, 351u);
  EXPECT_TRUE(ev.changed());
  EXPECT_GT(ev.version, version_before);
  EXPECT_EQ(ev.version, grid.version());
  ASSERT_EQ(ev.old_to_new.size(), 400u);
  EXPECT_EQ(grid.size(), 49u);

  std::set<std::pair<int, int>> expected;
  for (int x = 7; x <= 13; ++x) {
    for (int y = 7; y <= 13; ++y) {
      expected.insert({x, y});
    }
  }
  std::set<std::pair<int, int>> got;
  for (naex::grid::CellId v = 0; v < grid.size(); ++v) {
    got.insert({grid.cell(v).x, grid.cell(v).y});
    // Ids are contiguous and the cell -> id map agrees with id -> cell.
    EXPECT_EQ(grid.cellId(grid.cell(v)), v) << "cell " << v;
    // Costs moved with their cell.
    EXPECT_FLOAT_EQ(grid.costs(v)[0],
                    static_cast<Cost>(grid.cell(v).x * 100 + grid.cell(v).y));
  }
  EXPECT_EQ(got, expected);

  // The mapping describes exactly what happened, for everything that caches a
  // CellId (P2's neighbour table).
  naex::grid::CellId last = 0;
  bool first = true;
  for (naex::grid::CellId v = 0; v < ev.before; ++v) {
    const Cell &c = cells_before[v];
    const bool kept = std::abs(c.x - 10) <= 3 && std::abs(c.y - 10) <= 3;
    if (!kept) {
      EXPECT_EQ(ev.old_to_new[v], naex::grid::INVALID_CELL_ID) << "old " << v;
      continue;
    }
    const naex::grid::CellId w = ev.old_to_new[v];
    ASSERT_NE(w, naex::grid::INVALID_CELL_ID) << "old " << v;
    EXPECT_LE(w, v) << "the compaction never moves a cell up";
    if (!first) {
      EXPECT_GT(w, last) << "relative order is preserved";
    }
    last = w;
    first = false;
    EXPECT_EQ(grid.cell(w).x, c.x);
    EXPECT_EQ(grid.cell(w).y, c.y);
    EXPECT_FLOAT_EQ(grid.costs(w)[0], costs_before[v]);
  }
  // No stale entry survived in the cell -> id map.
  EXPECT_EQ(grid.findCell(Cell(0, 0)), naex::grid::INVALID_CELL_ID);
  EXPECT_EQ(grid.findCell(Cell(14, 10)), naex::grid::INVALID_CELL_ID);
}

TEST(Eviction, RadiusZeroAndRadiusLargerThanTheGrid) {
  Grid grid = makeDenseGrid(20);
  const uint64_t version_before = grid.version();

  // A radius larger than the grid keeps everything and renumbers nothing, so
  // every cached CellId stays valid and the version does not move.
  const naex::grid::Eviction all = grid.evictOutside(Cell(10, 10), 1000);
  EXPECT_EQ(all.before, 400u);
  EXPECT_EQ(all.after, 400u);
  EXPECT_EQ(all.removed, 0u);
  EXPECT_FALSE(all.changed());
  EXPECT_TRUE(all.old_to_new.empty());
  EXPECT_EQ(all.version, version_before);
  EXPECT_EQ(grid.version(), version_before);
  EXPECT_EQ(grid.cellId(Cell(19, 19)), 399u);

  // Radius 0 keeps exactly the centre cell.
  const naex::grid::Eviction one = grid.evictOutside(Cell(10, 10), 0);
  EXPECT_EQ(one.after, 1u);
  EXPECT_EQ(grid.size(), 1u);
  EXPECT_EQ(grid.cell(0).x, 10);
  EXPECT_EQ(grid.cell(0).y, 10);
  EXPECT_FLOAT_EQ(grid.costs(0)[0], 0.f);
  EXPECT_GT(grid.version(), version_before);

  // A negative radius empties the grid (it is not reachable from the planner,
  // which never evicts with map_range <= 0).
  const naex::grid::Eviction none = grid.evictOutside(Cell(10, 10), -1);
  EXPECT_EQ(none.after, 0u);
  EXPECT_TRUE(grid.empty());
}

TEST(Eviction, MapRangeZeroIsANoOp) {
  // The planner's gate: map_range <= 0 or NaN must leave the grid completely
  // alone, i.e. reproduce the pre-P6 unbounded map.
  for (const float range : {0.f, -10.f, kNaN}) {
    Grid grid = makeDenseGrid(20, 0.4f);
    const uint64_t version_before = grid.version();
    const naex::grid::Eviction ev =
        naex::grid::evictOutsideRange(grid, Point2f(0.2f, 0.2f), range);
    EXPECT_EQ(ev.before, 400u) << "range " << range;
    EXPECT_EQ(ev.after, 400u) << "range " << range;
    EXPECT_EQ(ev.removed, 0u) << "range " << range;
    EXPECT_FALSE(ev.changed()) << "range " << range;
    EXPECT_TRUE(ev.old_to_new.empty()) << "range " << range;
    EXPECT_EQ(ev.version, version_before) << "range " << range;
    EXPECT_EQ(grid.size(), 400u) << "range " << range;
    EXPECT_EQ(grid.version(), version_before) << "range " << range;
  }

  // So does a centre that is not a valid cell (a broken robot transform).
  Grid grid = makeDenseGrid(20, 0.4f);
  const uint64_t version_before = grid.version();
  EXPECT_FALSE(
      naex::grid::evictOutsideRange(grid, Point2f(kNaN, 0.f), 5.f).changed());
  EXPECT_FALSE(naex::grid::evictOutsideRange(grid, Point2f(1e9f, 0.f), 5.f)
                   .changed());
  EXPECT_EQ(grid.size(), 400u);
  EXPECT_EQ(grid.version(), version_before);
}

TEST(Eviction, MapRangeInMetresBoundsTheGrid) {
  // 40x40 cells of 0.4 m centred on the origin; map_range 3 m around the robot
  // cell keeps the square of ceil(3 / 0.4) = 8 cells around it, i.e. 17x17.
  Grid grid(0.4f, 1.f, Costs(0.f, 0.f, 0.f, 0.f));
  for (int16_t x = -20; x < 20; ++x) {
    for (int16_t y = -20; y < 20; ++y) {
      grid.cellId(Cell(x, y));
    }
  }
  ASSERT_EQ(grid.size(), 1600u);
  EXPECT_EQ(naex::grid::cellRadius(grid, 3.f), 8);

  const Point2f robot(0.1f, 0.1f);  // cell (0, 0)
  const naex::grid::Eviction ev =
      naex::grid::evictOutsideRange(grid, robot, 3.f);
  EXPECT_EQ(ev.after, 17u * 17u);
  EXPECT_EQ(grid.size(), 17u * 17u);
  for (naex::grid::CellId v = 0; v < grid.size(); ++v) {
    EXPECT_LE(std::abs(grid.cell(v).x), 8);
    EXPECT_LE(std::abs(grid.cell(v).y), 8);
    // The retained square contains the whole map_range disc.
    EXPECT_EQ(grid.cellId(grid.cell(v)), v);
  }
  for (int16_t x = -7; x <= 7; ++x) {
    EXPECT_NE(grid.findCell(Cell(x, 7)), naex::grid::INVALID_CELL_ID)
        << "cell (" << x << ", 7) is within 3 m and must survive";
  }

  // An eviction that removes nothing must not renumber: repeat it.
  const uint64_t version = grid.version();
  const naex::grid::Eviction again =
      naex::grid::evictOutsideRange(grid, robot, 3.f);
  EXPECT_FALSE(again.changed());
  EXPECT_EQ(grid.version(), version);
}

TEST(Eviction, PlanningOnACompactedGridIsExact) {
  // The 20x20 gap fixture of Planning.TwentyByTwentyWithGapExactCost, but
  // reached by compacting a 30x30 grid down to the 20x20 core: the plan after
  // the eviction must be bit for bit the plan on the grid built that way.
  Grid grid = makeDenseGrid(30);
  // Full-height wall on column x = 10 except the gap at y = 18, 19, so the
  // route is unambiguous both before and after the eviction.
  for (int16_t y = 0; y < 30; ++y) {
    if (y == 18 || y == 19) {
      continue;
    }
    grid.cellCosts(Cell(10, y))[0] = 5.f;
  }
  ASSERT_EQ(grid.size(), 900u);

  const naex::grid::Eviction ev =
      grid.eraseCells([&grid](naex::grid::CellId v) {
        return grid.cell(v).x < 20 && grid.cell(v).y < 20;
      });
  ASSERT_EQ(ev.after, 400u);
  ASSERT_EQ(ev.removed, 500u) << "every CellId is renumbered";
  ASSERT_EQ(grid.size(), 400u);

  const Costs max_costs(1.f);
  const VertexId start = grid.cellId(Cell(0, 0));
  const VertexId goal = grid.cellId(Cell(19, 0));
  const ShortestPaths sp(grid, start, 8, max_costs);
  const float expected = 19.f * std::sqrt(2.f) + 17.f;
  EXPECT_NEAR(sp.pathCost(goal), expected, 1e-3f);
  EXPECT_FALSE(std::isfinite(sp.pathCost(grid.cellId(Cell(10, 5)))));

  size_t vertices = 1;
  VertexId v = goal;
  while (v != start && vertices <= grid.size()) {
    EXPECT_FLOAT_EQ(grid.costs(v)[0], 0.f) << "path entered a blocked cell";
    v = sp.predecessor(v);
    ++vertices;
  }
  ASSERT_EQ(v, start);
  EXPECT_EQ(vertices, 37u);

  // The wall is still where it was, addressed by cell rather than by id.
  for (int16_t y = 0; y <= 17; ++y) {
    EXPECT_FLOAT_EQ(grid.cellCosts(Cell(10, y))[0], 5.f) << "y = " << y;
  }
  EXPECT_EQ(grid.findCell(Cell(20, 0)), naex::grid::INVALID_CELL_ID);
}

TEST(Eviction, EraseCellsOnAnEmptyGrid) {
  Grid grid(0.4f);
  const naex::grid::Eviction ev = grid.evictOutside(Cell(0, 0), 3);
  EXPECT_EQ(ev.before, 0u);
  EXPECT_EQ(ev.after, 0u);
  EXPECT_FALSE(ev.changed());
  EXPECT_TRUE(grid.empty());
}

TEST(Grid, VersionBumpsOnStructuralChangesOnly) {
  Grid grid(1.f, 1.f, Costs(0.f, 0.f, 0.f, 0.f));
  const uint64_t v0 = grid.version();
  grid.cellId(Cell(0, 0));
  const uint64_t v1 = grid.version();
  EXPECT_GT(v1, v0);
  // An existing cell is not created again.
  grid.cellId(Cell(0, 0));
  EXPECT_EQ(grid.version(), v1);
  // Reads and cost updates leave the numbering alone.
  grid.updateCellCost(Cell(0, 0), 0, 3.f);
  EXPECT_EQ(grid.version(), v1);
  EXPECT_EQ(grid.size(), 1u);
  // ... but a cost update that creates a cell does bump it.
  grid.updateCellCost(Cell(5, 5), 0, 3.f);
  EXPECT_GT(grid.version(), v1);
  const uint64_t v2 = grid.version();
  grid.clear();
  EXPECT_GT(grid.version(), v2);
}
