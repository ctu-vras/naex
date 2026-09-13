// Unit tests for the grid planner data structures.
#include "naex/grid/graph.h"
#include "naex/grid/grid.h"
#include "naex/grid/path.h"
#include "naex/grid/search.h"

#include <gtest/gtest.h>

#include <algorithm>
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
using naex::grid::Graph;
using naex::grid::Grid;
using naex::grid::INVALID_VERTEX_ID;
using naex::grid::Point2f;
using naex::grid::ShortestPaths;
using naex::grid::trace_path_vertices;
using naex::grid::VertexId;

namespace {
constexpr Cost kNaN = std::numeric_limits<Cost>::quiet_NaN();
} // namespace

TEST(Grid, PointToCellRoundTrip) {
  const Grid grid(0.4f);
  for (int16_t x = -5; x <= 5; ++x) {
    for (int16_t y = -5; y <= 5; ++y) {
      const Cell c(x, y);
      const Point2f p = grid.cell_to_point(c);
      const Cell back = grid.point_to_cell(p);
      EXPECT_EQ(c.x, back.x) << "x = " << x << ", y = " << y;
      EXPECT_EQ(c.y, back.y) << "x = " << x << ", y = " << y;
    }
  }
}

TEST(Grid, PointToCellNegatives) {
  const Grid grid(0.4f);
  // Floor, not truncation: everything in [-0.4, 0) maps to cell -1.
  EXPECT_EQ(grid.point_to_cell({-0.1f, -0.1f}).x, -1);
  EXPECT_EQ(grid.point_to_cell({-0.1f, -0.1f}).y, -1);
  EXPECT_EQ(grid.point_to_cell({-0.4f, -0.4f}).x, -1);
  EXPECT_EQ(grid.point_to_cell({-0.41f, 0.0f}).x, -2);
  EXPECT_EQ(grid.point_to_cell({0.0f, 0.0f}).x, 0);
  EXPECT_EQ(grid.point_to_cell({0.39f, 0.39f}).x, 0);
  EXPECT_EQ(grid.point_to_cell({0.41f, 0.41f}).x, 1);
  // Cell centres.
  EXPECT_FLOAT_EQ(grid.cell_to_point(Cell(-1, -1)).x, -0.2f);
  EXPECT_FLOAT_EQ(grid.cell_to_point(Cell(0, 0)).x, 0.2f);
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
  EXPECT_FLOAT_EQ(grid.update_cell_cost(c, 0, 2.f)[0], 2.f);
  // Then w0 * old + w1 * new with w1 = forget_factor.
  EXPECT_FLOAT_EQ(grid.update_cell_cost(c, 0, 4.f)[0], 3.f);
  EXPECT_FLOAT_EQ(grid.update_cell_cost(c, 0, 4.f)[0], 3.5f);

  // forget_factor == 1 keeps only the newest value.
  Grid fresh(1.f, 1.f);
  EXPECT_FLOAT_EQ(fresh.update_cell_cost(c, 0, 2.f)[0], 2.f);
  EXPECT_FLOAT_EQ(fresh.update_cell_cost(c, 0, 7.f)[0], 7.f);
}

TEST(Grid, UpdatePointCostCreatesCell) {
  Grid grid(1.f, 1.f);
  EXPECT_TRUE(grid.empty());
  grid.update_point_cost({0.5f, 0.5f}, 0, 1.f);
  EXPECT_EQ(grid.size(), 1u);
  grid.update_point_cost({0.6f, 0.6f}, 0, 1.f);
  EXPECT_EQ(grid.size(), 1u) << "same cell must not be duplicated";
  grid.update_point_cost({-0.5f, 0.5f}, 0, 1.f);
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

TEST(Graph, CostsInBoundsSkipsNaNMax) {
  Grid grid(1.f, 1.f);
  // A non-finite max means "layer not bounded" and is skipped; the layers
  // behind it are still checked.  (Before the A* branch the first non-finite
  // entry stopped the loop, which silently unbounded every later layer.)
  const Graph graph(grid, 8, Costs(1.f, kNaN, 0.f, 0.f));
  EXPECT_TRUE(graph.costs_in_bounds(Costs(0.5f, 100.f, 0.f, 0.f)));
  // Layer 1 is unbounded, layers 2 and 3 are bounded at 0.
  EXPECT_FALSE(graph.costs_in_bounds(Costs(0.5f, 100.f, 100.f, 0.f)));
  EXPECT_FALSE(graph.costs_in_bounds(Costs(0.5f, 100.f, 0.f, 100.f)));
  EXPECT_FALSE(graph.costs_in_bounds(Costs(1.5f, kNaN, 0.f, 0.f)));
  // NaN cost never compares <= max, so a bounded layer holding NaN is out of
  // bounds; an unbounded layer holding NaN is not looked at.
  EXPECT_FALSE(graph.costs_in_bounds(Costs(kNaN, kNaN, 0.f, 0.f)));
  EXPECT_TRUE(graph.costs_in_bounds(Costs(1.f, kNaN, 0.f, 0.f)));

  // An all-NaN max bounds nothing.
  const Graph unbounded(grid, 8, Costs());
  EXPECT_TRUE(unbounded.costs_in_bounds(Costs(1e6f, 1e6f, 1e6f, 1e6f)));
  // Only the layers that carry a finite max are bounded.
  const Graph layer3(grid, 8, Costs(kNaN, kNaN, kNaN, 1.f));
  EXPECT_TRUE(layer3.costs_in_bounds(Costs(1e6f, 1e6f, 1e6f, 1.f)));
  EXPECT_FALSE(layer3.costs_in_bounds(Costs(1e6f, 1e6f, 1e6f, 1.5f)));
}

TEST(ShortestPaths, ThreeByThreeWithBlockedCell) {
  // 3x3 grid of unit cells, all free except the centre.
  Grid grid(1.f, 1.f, Costs(0.f));
  VertexId id[3][3];
  for (int16_t x = 0; x < 3; ++x) {
    for (int16_t y = 0; y < 3; ++y) {
      id[x][y] = grid.cell_id(Cell(x, y));
    }
  }
  ASSERT_EQ(grid.size(), 9u);
  grid.cell_costs(Cell(1, 1))[0] = 5.f;

  const Costs max_costs(1.f);
  const VertexId start = id[0][0];
  const ShortestPaths sp(grid, start, 4, max_costs);

  EXPECT_FLOAT_EQ(sp.path_cost(start), 0.f);
  EXPECT_EQ(sp.predecessor(start), start);

  // The blocked cell is unreachable.
  EXPECT_FALSE(std::isfinite(sp.path_cost(id[1][1])));

  // Each free step costs 1 + (0 + 0) / 2 times the cell size.
  EXPECT_FLOAT_EQ(sp.path_cost(id[1][0]), 1.f);
  EXPECT_FLOAT_EQ(sp.path_cost(id[0][1]), 1.f);
  EXPECT_FLOAT_EQ(sp.path_cost(id[2][0]), 2.f);
  // Around the blocked centre: 4 steps instead of a diagonal.
  EXPECT_FLOAT_EQ(sp.path_cost(id[2][2]), 4.f);

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
  const VertexId a = grid.cell_id(Cell(0, 0));
  const VertexId b = grid.cell_id(Cell(1, 0));
  const ShortestPaths sp(grid, a, 8, Costs(1.f));
  EXPECT_FLOAT_EQ(sp.path_cost(a), 0.f);
  EXPECT_FALSE(std::isfinite(sp.path_cost(b)));
}

// --- Regression guards for the performance work ---------------------------
// The tests below pin behaviour that P2, P3, P8 and P9 must preserve.  They
// are deliberately written against exact cell sets and exact path costs, not
// against timings.

namespace {

/// Dense square grid of cells [0, n) x [0, n) with all-zero costs on every
/// layer (Costs(0.f) alone would leave layers 1..3 NaN), created in x-major
/// order so that CellIds (and therefore tie-breaking) are stable.
Grid make_dense_grid(int16_t n, float cell_size = 1.f) {
  Grid grid(cell_size, 1.f, Costs(0.f, 0.f, 0.f, 0.f));
  for (int16_t x = 0; x < n; ++x) {
    for (int16_t y = 0; y < n; ++y) {
      grid.cell_id(Cell(x, y));
    }
  }
  return grid;
}

} // namespace

TEST(AdHocLayer, FillLayerTouchesOnlyThatLayer) {
  Grid grid = make_dense_grid(3);
  naex::grid::fill_layer(grid, 3, 7.f);
  for (naex::grid::CellId v = 0; v < grid.size(); ++v) {
    EXPECT_FLOAT_EQ(grid.costs(v)[3], 7.f);
    EXPECT_FLOAT_EQ(grid.costs(v)[0], 0.f);
  }
  // Out-of-range layers are a no-op, not a crash or an out-of-bounds write.
  naex::grid::fill_layer(grid, -1, 1.f);
  naex::grid::fill_layer(grid, static_cast<int>(Costs::kSize), 1.f);
  EXPECT_FLOAT_EQ(grid.costs(0)[3], 7.f);
}

TEST(SidelobeDisc, UnitGridPlusShapeInclusiveBoundary) {
  // 5x5 unit cells, centres at (x + 0.5, y + 0.5).  A radius of exactly 1.0
  // around the centre of cell (2, 2) includes the four edge neighbours (d = 1,
  // boundary is inclusive) and excludes the diagonals (d = sqrt(2)).
  Grid grid = make_dense_grid(5);
  const size_t size_before = grid.size();
  naex::grid::apply_disc_cost(grid, 3, Point2f(2.5f, 2.5f), 1.0f, 10.f);
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
      grid.cell_id(Cell(x, y));
    }
  }
  const size_t size_before = grid.size();
  naex::grid::apply_disc_cost(grid, 3, Point2f(0.f, 0.f), 0.81f, 10.f);
  EXPECT_EQ(grid.size(), size_before);

  std::set<std::pair<int, int>> affected;
  for (naex::grid::CellId v = 0; v < grid.size(); ++v) {
    if (grid.costs(v)[3] == 10.f) {
      affected.insert({grid.cell(v).x, grid.cell(v).y});
    }
  }
  const std::set<std::pair<int, int>> expected = {
      {-1, -1}, {-1, 0}, {0, -1}, {0, 0},  // |cx| = |cy| = 0.2
      {-2, -1}, {-2, 0}, {1, -1}, {1, 0},  // |cx| = 0.6, |cy| = 0.2
      {-1, -2}, {0, -2}, {-1, 1}, {0, 1}}; // |cx| = 0.2, |cy| = 0.6
  EXPECT_EQ(affected.size(), 12u);
  EXPECT_EQ(affected, expected);
}

namespace {

// Reference implementation of apply_disc_cost(): the full-grid pass P3
// replaced.  Kept verbatim so the bounding-box walk can be compared against it
// cell by cell on random grids.
void reference_apply_disc_cost(Grid &grid, int layer, const Point2f &center,
                               float radius, Cost cost,
                               std::vector<naex::grid::CellId> *touched) {
  if (!naex::grid::is_valid_layer(layer)) {
    return;
  }
  const naex::grid::CellId n = static_cast<naex::grid::CellId>(grid.size());
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

} // namespace

TEST(SidelobeDisc, SparseGridOnlyTouchesExistingCells) {
  // 0.4 m cells covering [-2, 2) in both axes, with (0, 0) -- the cell the disc
  // is centred on -- deliberately missing.
  Grid grid(0.4f, 1.f, Costs(0.f, 0.f, 0.f, 0.f));
  for (int16_t x = -5; x <= 4; ++x) {
    for (int16_t y = -5; y <= 4; ++y) {
      if (x == 0 && y == 0) {
        continue;
      }
      grid.cell_id(Cell(x, y));
    }
  }
  const size_t size_before = grid.size();
  std::vector<naex::grid::CellId> dirty;
  naex::grid::apply_disc_cost(grid, 3, Point2f(0.f, 0.f), 0.81f, 10.f, &dirty);
  EXPECT_EQ(grid.size(), size_before) << "the hole must not be created";
  EXPECT_FALSE(grid.has_cell(Cell(0, 0)));

  std::set<std::pair<int, int>> affected;
  for (naex::grid::CellId v = 0; v < grid.size(); ++v) {
    if (grid.costs(v)[3] == 10.f) {
      affected.insert({grid.cell(v).x, grid.cell(v).y});
    }
  }
  // ProductionSizedLobeHitsTwelveCells' set minus the missing (0, 0).
  const std::set<std::pair<int, int>> expected = {
      {-1, -1}, {-1, 0},  {0, -1}, {-2, -1}, {-2, 0}, {1, -1},
      {1, 0},   {-1, -2}, {0, -2}, {-1, 1},  {0, 1}};
  EXPECT_EQ(affected, expected);
  EXPECT_EQ(dirty.size(), expected.size());
}

TEST(SidelobeDisc, DirtyListRestoresExactly) {
  // Distinct per-cell values on layer 3 stand in for "anything but the
  // default"; after the dirty-list clear every cell must be back at the
  // default and no cell outside the two discs may have been written at all.
  Grid grid = make_dense_grid(10, 0.4f);
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
  naex::grid::apply_disc_cost(grid, 3, Point2f(1.0f, 1.0f), 0.81f, 10.f,
                              &dirty);
  naex::grid::apply_disc_cost(grid, 3, Point2f(1.2f, 1.0f), 0.81f, 7.f, &dirty);
  ASSERT_FALSE(dirty.empty());
  // The discs overlap, so the dirty list holds duplicates; restoring twice is
  // idempotent, which is exactly what the planner relies on.
  const std::set<naex::grid::CellId> unique(dirty.begin(), dirty.end());
  EXPECT_LT(unique.size(), dirty.size()) << "the two discs must overlap";

  size_t written = 0;
  for (naex::grid::CellId v = 0; v < grid.size(); ++v) {
    if (grid.costs(v)[3] != kDefault) {
      ++written;
      EXPECT_EQ(unique.count(v), 1u)
          << "cell " << v << " written but not dirty";
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
      grid.cell_id(Cell(static_cast<int16_t>(x), static_cast<int16_t>(y)));
      reference.cell_id(Cell(static_cast<int16_t>(x), static_cast<int16_t>(y)));
    }
  }
  ASSERT_EQ(grid.point_to_cell({kCenter, 0.f}).x, 30000);

  std::vector<naex::grid::CellId> dirty;
  naex::grid::apply_disc_cost(grid, 3, Point2f(kCenter, 0.f), 0.81f, 10.f,
                              &dirty);
  reference_apply_disc_cost(reference, 3, Point2f(kCenter, 0.f), 0.81f, 10.f,
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
      grid.cell_id(Cell(x, y));
      reference.cell_id(Cell(x, y));
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
    naex::grid::apply_disc_cost(grid, 3, center, r, cost, &dirty);
    reference_apply_disc_cost(reference, 3, center, r, cost, &ref_dirty);
    ASSERT_EQ(grid.size(), size_before) << "no cell may be created";
    ASSERT_EQ(dirty, ref_dirty) << "trial " << trial << " centre (" << center.x
                                << ", " << center.y << ") radius " << r;
    for (naex::grid::CellId v = 0; v < grid.size(); ++v) {
      ASSERT_FLOAT_EQ(grid.costs(v)[3], reference.costs(v)[3])
          << "trial " << trial << ", cell " << v;
    }
  }

  // Degenerate inputs selected nothing before and must select nothing now.
  const float kNaNf = std::numeric_limits<float>::quiet_NaN();
  std::vector<naex::grid::CellId> dirty;
  naex::grid::apply_disc_cost(grid, 3, Point2f(0.f, 0.f), -1.f, 99.f, &dirty);
  naex::grid::apply_disc_cost(grid, 3, Point2f(kNaNf, 0.f), 1.f, 99.f, &dirty);
  naex::grid::apply_disc_cost(grid, 3, Point2f(0.f, 0.f), kNaNf, 99.f, &dirty);
  EXPECT_TRUE(dirty.empty());
  EXPECT_EQ(grid.size(), size_before);
}

TEST(InflationDisc, CreatesCellsAndSkipsCentre) {
  // 1 m cells, forget_factor 1 (pure overwrite on first touch), nothing
  // pre-existing: every cell within radius of (2.5, 2.5) except the centre
  // cell itself must be created and set.
  Grid grid(1.f, 1.f, Costs(0.f, 0.f, 0.f, 0.f));
  naex::grid::inflate_disc_cost(grid, 3, Point2f(2.5f, 2.5f), 1.0f, 10.f);

  const std::set<std::pair<int, int>> expected_new = {{1, 2}, {3, 2},
                                                       {2, 1}, {2, 3}};
  std::set<std::pair<int, int>> affected;
  for (naex::grid::CellId v = 0; v < grid.size(); ++v) {
    affected.insert({grid.cell(v).x, grid.cell(v).y});
    EXPECT_FLOAT_EQ(grid.costs(v)[3], 10.f);
  }
  EXPECT_EQ(affected, expected_new);
  EXPECT_FALSE(grid.has_cell(Cell(2, 2))) << "the centre cell is the caller's job";
}

TEST(InflationDisc, BlendsWithForgetFactorInsteadOfOverwriting) {
  // forget_factor 0.5, default costs unset (NaN): an already-set cell must
  // blend (w0*old + w1*new), not get clobbered like apply_disc_cost() would.
  Grid grid(1.f, 0.5f);
  const Cell existing(1, 2);
  grid.costs(grid.cell_id(existing))[3] = 4.f;
  naex::grid::inflate_disc_cost(grid, 3, Point2f(2.5f, 2.5f), 1.0f, 10.f);
  // w0 * 4 + w1 * 10 with w0 = 1 - forget_factor = 0.5, w1 = forget_factor.
  EXPECT_FLOAT_EQ(grid.costs(grid.cell_id(existing))[3], 7.f);
  // A brand-new cell's layer starts at NaN, so update_cost_at() takes the
  // first cost verbatim instead of blending against it.
  EXPECT_FLOAT_EQ(grid.costs(grid.cell_id(Cell(3, 2)))[3], 10.f);
}

TEST(InflationDisc, RespectsBoundOriginForNewCellsOnly) {
  // bound_origin/bound_range emulate map_range: a cell that does not exist
  // yet must not be created outside the bound (it would be evicted right
  // away), but a cell that already exists inside the disc must still be
  // updated even if it happens to be outside the bound (eviction's job, not
  // inflation's).
  Grid grid(1.f, 1.f, Costs(0.f, 0.f, 0.f, 0.f));
  const Cell far_existing(3, 2); // inside the disc, outside the bound below
  grid.cell_id(far_existing);
  const Point2f origin(2.5f, 2.5f);
  const float bound_range = 0.6f; // smaller than the disc radius (1.0 m)

  naex::grid::inflate_disc_cost(grid, 3, origin, 1.0f, 10.f, &origin,
                                bound_range);

  EXPECT_FALSE(grid.has_cell(Cell(1, 2)))
      << "outside the bound and did not exist: must not be created";
  EXPECT_TRUE(grid.has_cell(far_existing));
  EXPECT_FLOAT_EQ(grid.costs(grid.cell_id(far_existing))[3], 10.f)
      << "already existed: must still be updated regardless of the bound";
}

TEST(InflationDisc, DegenerateInputsAreNoOps) {
  Grid grid(1.f, 1.f, Costs(0.f, 0.f, 0.f, 0.f));
  const size_t size_before = grid.size();
  const float kNaNf = std::numeric_limits<float>::quiet_NaN();
  naex::grid::inflate_disc_cost(grid, 3, Point2f(0.f, 0.f), -1.f, 10.f);
  naex::grid::inflate_disc_cost(grid, 3, Point2f(0.f, 0.f), 0.f, 10.f);
  naex::grid::inflate_disc_cost(grid, 3, Point2f(kNaNf, 0.f), 1.f, 10.f);
  naex::grid::inflate_disc_cost(grid, 3, Point2f(0.f, 0.f), kNaNf, 10.f);
  naex::grid::inflate_disc_cost(grid, -1, Point2f(0.f, 0.f), 1.f, 10.f);
  EXPECT_EQ(grid.size(), size_before);
}

TEST(NearestCell, EmptyPredicateAndTieBreak) {
  Grid grid = make_dense_grid(3);
  EXPECT_EQ(naex::grid::nearest_cell(grid, Point2f(0.f, 0.f),
                                     [](naex::grid::CellId) { return false; }),
            naex::grid::INVALID_CELL_ID);
  // Equidistant from the centres of (0, 0) and (1, 0); the lower CellId, i.e.
  // the cell created first, wins.
  const naex::grid::CellId v = naex::grid::nearest_cell(
      grid, Point2f(1.0f, 0.5f), [](naex::grid::CellId) { return true; });
  EXPECT_EQ(grid.cell(v).x, 0);
  EXPECT_EQ(grid.cell(v).y, 0);
}

TEST(SnapGoalCell, GoalAlreadyOnSnapCellReturnsInvalid) {
  // Default level-0 cost 5 (above max_cost 1) so only explicitly set cells
  // qualify as snap cells.
  Grid grid(1.f, 1.f, Costs(5.f, kNaN, kNaN, kNaN));
  grid.cell_costs(Cell(0, 0))[0] = 0.5f; // centre (0.5, 0.5): a snap cell
  const Point2f goal(0.5f, 0.5f);
  EXPECT_EQ(naex::grid::snap_goal_cell(grid, goal, /*radius=*/5.f, /*level=*/0,
                                       /*max_cost=*/1.f, Costs()),
            naex::grid::INVALID_CELL_ID);
}

TEST(SnapGoalCell, SnapsToNearestCellWithinRadius) {
  Grid grid(1.f, 1.f, Costs(5.f, kNaN, kNaN, kNaN));
  grid.cell_costs(Cell(2, 0))[0] = 0.5f; // centre (2.5, 0.5), dist 2 from goal
  grid.cell_costs(Cell(5, 0))[0] = 0.5f; // centre (5.5, 0.5), dist 5 from goal
  const Point2f goal(0.5f, 0.5f);
  const naex::grid::CellId v = naex::grid::snap_goal_cell(
      grid, goal, /*radius=*/6.f, /*level=*/0, /*max_cost=*/1.f, Costs());
  ASSERT_NE(v, naex::grid::INVALID_CELL_ID);
  EXPECT_EQ(grid.cell(v).x, 2);
  EXPECT_EQ(grid.cell(v).y, 0);
}

TEST(SnapGoalCell, NoSnapCellWithinRadiusReturnsInvalid) {
  Grid grid(1.f, 1.f, Costs(5.f, kNaN, kNaN, kNaN));
  grid.cell_costs(Cell(5, 0))[0] = 0.5f; // dist 5 from goal, outside radius 3
  const Point2f goal(0.5f, 0.5f);
  EXPECT_EQ(naex::grid::snap_goal_cell(grid, goal, /*radius=*/3.f, /*level=*/0,
                                       /*max_cost=*/1.f, Costs()),
            naex::grid::INVALID_CELL_ID);
}

TEST(SnapGoalCell, CellViolatingMaxCostsIsSkipped) {
  Grid grid(1.f, 1.f, Costs(5.f, 5.f, kNaN, kNaN));
  // Level-0 cost alone would qualify, but level 1 is out of max_costs bounds,
  // so this cell must not be treated as a snap cell.
  Costs &c = grid.cell_costs(Cell(2, 0));
  c[0] = 0.5f;
  c[1] = 10.f;
  const Point2f goal(0.5f, 0.5f);
  const Costs max_costs(kNaN, 1.f, kNaN, kNaN); // bound only layer 1
  EXPECT_EQ(naex::grid::snap_goal_cell(grid, goal, /*radius=*/6.f, /*level=*/0,
                                       /*max_cost=*/1.f, max_costs),
            naex::grid::INVALID_CELL_ID);
}

TEST(SnapGoalCell, InvalidLevelOrRadiusReturnsInvalid) {
  Grid grid(1.f, 1.f, Costs(5.f, kNaN, kNaN, kNaN));
  grid.cell_costs(Cell(2, 0))[0] = 0.5f;
  const Point2f goal(0.5f, 0.5f);
  EXPECT_EQ(naex::grid::snap_goal_cell(grid, goal, 6.f, /*level=*/-1, 1.f,
                                       Costs()),
            naex::grid::INVALID_CELL_ID);
  EXPECT_EQ(naex::grid::snap_goal_cell(
                grid, goal, 6.f, /*level=*/static_cast<int>(Costs::kSize),
                1.f, Costs()),
            naex::grid::INVALID_CELL_ID);
  EXPECT_EQ(
      naex::grid::snap_goal_cell(grid, goal, /*radius=*/0.f, 0, 1.f, Costs()),
      naex::grid::INVALID_CELL_ID);
}

TEST(Planning, TwentyByTwentyWithGapExactCost) {
  // 20x20 unit cells, free cost 0, wall on column x = 10 for y in [0, 17].
  // The only way across is the gap at (10, 18) / (10, 19).
  Grid grid = make_dense_grid(20);
  for (int16_t y = 0; y <= 17; ++y) {
    grid.cell_costs(Cell(10, y))[0] = 5.f;
  }
  const Costs max_costs(1.f);
  const VertexId start = grid.cell_id(Cell(0, 0));
  const VertexId goal = grid.cell_id(Cell(19, 0));
  const ShortestPaths sp(grid, start, 8, max_costs);

  // Every free step costs cell_size * (1 + (0 + 0) / 2) * distance8, so an
  // optimal route is 19 diagonal and 17 straight steps: through (10, 18)
  // it is 10*sqrt(2) + 8 to the gap and 9*sqrt(2) + 9 back down.  Going
  // through (10, 19) instead would cost 19*sqrt(2) + 19.
  const float expected = 19.f * std::sqrt(2.f) + 17.f;
  // Dijkstra accumulates 36 float additions, so compare with a tolerance far
  // below the 2.0 cost difference to the next-best route.
  EXPECT_NEAR(sp.path_cost(goal), expected, 1e-3f);

  // The wall itself is unreachable and no optimal path enters it.
  EXPECT_FALSE(std::isfinite(sp.path_cost(grid.cell_id(Cell(10, 5)))));
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
  Grid grid = make_dense_grid(20);
  for (int16_t y = 0; y < 20; ++y) {
    grid.cell_costs(Cell(10, y))[0] = 5.f;
  }
  const VertexId start = grid.cell_id(Cell(0, 0));
  const ShortestPaths sp(grid, start, 8, Costs(1.f));
  ASSERT_FALSE(std::isfinite(sp.path_cost(grid.cell_id(Cell(19, 0)))));

  const naex::grid::CellId v1 = naex::grid::nearest_cell(
      grid, Point2f(19.5f, 0.5f),
      [&sp](naex::grid::CellId v) { return std::isfinite(sp.path_cost(v)); });
  ASSERT_NE(v1, naex::grid::INVALID_CELL_ID);
  EXPECT_EQ(grid.cell(v1).x, 9) << "nearest reachable cell to the goal";
  EXPECT_EQ(grid.cell(v1).y, 0);

  // With nothing reachable the scan must report failure rather than cell 0.
  Grid blocked = make_dense_grid(3);
  for (naex::grid::CellId v = 0; v < blocked.size(); ++v) {
    blocked.costs(v)[0] = 5.f;
  }
  const ShortestPaths none(blocked, 0, 8, Costs(1.f));
  const naex::grid::CellId unreachable = naex::grid::nearest_cell(
      blocked, Point2f(2.5f, 2.5f), [&none](naex::grid::CellId v) {
        return v != 0 && std::isfinite(none.path_cost(v));
      });
  EXPECT_EQ(unreachable, naex::grid::INVALID_CELL_ID);
}

// --- P6: input guards and the map bound ------------------------------------

namespace {

/// The per-point acceptance of Planner::receive_cloud(), without ROS: insert
/// every accepted point of @p points into @p grid on layer 0.  Mirrors the
/// loop in planner.h, so it pins the ingestion contract rather than the helper.
size_t ingest_points(Grid &grid, const std::vector<Point2f> &points,
                     const Point2f &origin, float input_range) {
  size_t skipped = 0;
  for (const Point2f &p : points) {
    if (!naex::grid::accept_input_point(grid, p, origin, input_range)) {
      ++skipped;
      continue;
    }
    grid.update_point_cost(p, 0, 1.f);
  }
  return skipped;
}

} // namespace

TEST(InputGuard, NonFinitePointsCreateNoCell) {
  // Casting NaN/Inf to int16_t is undefined behaviour and used to create a
  // phantom cell wherever the conversion happened to land (B8).
  const float kInf = std::numeric_limits<float>::infinity();
  Grid grid(0.4f, 1.f, Costs(0.f, 0.f, 0.f, 0.f));
  const std::vector<Point2f> bad = {Point2f(kNaN, 0.f),  Point2f(0.f, kNaN),
                                    Point2f(kNaN, kNaN), Point2f(kInf, 0.f),
                                    Point2f(0.f, -kInf), Point2f(-kInf, kInf)};
  EXPECT_EQ(ingest_points(grid, bad, Point2f(0.f, 0.f), 0.f), bad.size());
  EXPECT_EQ(grid.size(), 0u);
  EXPECT_TRUE(grid.empty());

  // One good point still lands, so the guard is not simply rejecting all.
  EXPECT_EQ(ingest_points(grid, {Point2f(1.f, 1.f)}, Point2f(0.f, 0.f), 0.f),
            0u);
  EXPECT_EQ(grid.size(), 1u);
}

TEST(InputGuard, OutOfInt16RangePointsCreateNoCell) {
  // At cell_size 0.4 the int16_t cell index runs out at +-13106.8 m.
  Grid grid(0.4f, 1.f, Costs(0.f, 0.f, 0.f, 0.f));
  const float limit = naex::grid::max_cell_coord(grid.cell_size());
  EXPECT_FLOAT_EQ(limit, 32767.f * 0.4f);

  const std::vector<Point2f> outside = {
      Point2f(limit, 0.f),  Point2f(-limit, 0.f),      Point2f(0.f, limit),
      Point2f(0.f, -limit), Point2f(2.f * limit, 0.f), Point2f(0.f, -1e9f)};
  EXPECT_EQ(ingest_points(grid, outside, Point2f(0.f, 0.f), 0.f),
            outside.size());
  EXPECT_EQ(grid.size(), 0u);

  // Just inside the limit is still accepted, and lands where it should.
  const float inside = std::nextafter(limit, 0.f);
  EXPECT_EQ(
      ingest_points(grid, {Point2f(inside, -inside)}, Point2f(0.f, 0.f), 0.f),
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
  EXPECT_EQ(ingest_points(grid, points, origin, range),
            points.size() - expected.size());
  ASSERT_EQ(grid.size(), expected.size());
  std::set<std::pair<int, int>> got;
  for (naex::grid::CellId v = 0; v < grid.size(); ++v) {
    got.insert({grid.cell(v).x, grid.cell(v).y});
  }
  EXPECT_EQ(got, expected);

  // A boundary point exactly at range is kept; a hair further out is not.
  EXPECT_TRUE(naex::grid::accept_input_point(
      grid, Point2f(origin.x + range, origin.y), origin, range));
  EXPECT_FALSE(naex::grid::accept_input_point(
      grid, Point2f(std::nextafter(origin.x + range, 100.f), origin.y), origin,
      range));

  // range <= 0 and NaN disable the crop; every lattice point then lands.
  for (const float off : {0.f, -1.f, kNaN}) {
    Grid all(1.f, 1.f, Costs(0.f, 0.f, 0.f, 0.f));
    EXPECT_EQ(ingest_points(all, points, origin, off), 0u) << "range " << off;
    EXPECT_EQ(all.size(), points.size()) << "range " << off;
  }
  // A broken (non-finite) origin must not silently empty the map either.
  Grid no_origin(1.f, 1.f, Costs(0.f, 0.f, 0.f, 0.f));
  EXPECT_EQ(ingest_points(no_origin, points, Point2f(kNaN, kNaN), range), 0u);
  EXPECT_EQ(no_origin.size(), points.size());
}

TEST(Eviction, EvictOutsideKeepsExactlyTheSquare) {
  // 20x20 unit cells, evict around (10, 10) with a Chebyshev radius of 3:
  // exactly the 7x7 square [7, 13]^2 survives, renumbered but in order, and
  // every cell keeps its own costs.
  Grid grid = make_dense_grid(20);
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

  const naex::grid::Eviction ev = grid.evict_outside(Cell(10, 10), 3);
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
    EXPECT_EQ(grid.cell_id(grid.cell(v)), v) << "cell " << v;
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
  EXPECT_EQ(grid.find_cell(Cell(0, 0)), naex::grid::INVALID_CELL_ID);
  EXPECT_EQ(grid.find_cell(Cell(14, 10)), naex::grid::INVALID_CELL_ID);
}

TEST(Eviction, RadiusZeroAndRadiusLargerThanTheGrid) {
  Grid grid = make_dense_grid(20);
  const uint64_t version_before = grid.version();

  // A radius larger than the grid keeps everything and renumbers nothing, so
  // every cached CellId stays valid and the version does not move.
  const naex::grid::Eviction all = grid.evict_outside(Cell(10, 10), 1000);
  EXPECT_EQ(all.before, 400u);
  EXPECT_EQ(all.after, 400u);
  EXPECT_EQ(all.removed, 0u);
  EXPECT_FALSE(all.changed());
  EXPECT_TRUE(all.old_to_new.empty());
  EXPECT_EQ(all.version, version_before);
  EXPECT_EQ(grid.version(), version_before);
  EXPECT_EQ(grid.cell_id(Cell(19, 19)), 399u);

  // Radius 0 keeps exactly the centre cell.
  const naex::grid::Eviction one = grid.evict_outside(Cell(10, 10), 0);
  EXPECT_EQ(one.after, 1u);
  EXPECT_EQ(grid.size(), 1u);
  EXPECT_EQ(grid.cell(0).x, 10);
  EXPECT_EQ(grid.cell(0).y, 10);
  EXPECT_FLOAT_EQ(grid.costs(0)[0], 0.f);
  EXPECT_GT(grid.version(), version_before);

  // A negative radius empties the grid (it is not reachable from the planner,
  // which never evicts with map_range <= 0).
  const naex::grid::Eviction none = grid.evict_outside(Cell(10, 10), -1);
  EXPECT_EQ(none.after, 0u);
  EXPECT_TRUE(grid.empty());
}

TEST(Eviction, MapRangeZeroIsANoOp) {
  // The planner's gate: map_range <= 0 or NaN must leave the grid completely
  // alone, i.e. reproduce the pre-P6 unbounded map.
  for (const float range : {0.f, -10.f, kNaN}) {
    Grid grid = make_dense_grid(20, 0.4f);
    const uint64_t version_before = grid.version();
    const naex::grid::Eviction ev =
        naex::grid::evict_outside_range(grid, Point2f(0.2f, 0.2f), range);
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
  Grid grid = make_dense_grid(20, 0.4f);
  const uint64_t version_before = grid.version();
  EXPECT_FALSE(
      naex::grid::evict_outside_range(grid, Point2f(kNaN, 0.f), 5.f).changed());
  EXPECT_FALSE(
      naex::grid::evict_outside_range(grid, Point2f(1e9f, 0.f), 5.f).changed());
  EXPECT_EQ(grid.size(), 400u);
  EXPECT_EQ(grid.version(), version_before);
}

TEST(Eviction, MapRangeInMetresBoundsTheGrid) {
  // 40x40 cells of 0.4 m centred on the origin; map_range 3 m around the robot
  // cell keeps the square of ceil(3 / 0.4) = 8 cells around it, i.e. 17x17.
  Grid grid(0.4f, 1.f, Costs(0.f, 0.f, 0.f, 0.f));
  for (int16_t x = -20; x < 20; ++x) {
    for (int16_t y = -20; y < 20; ++y) {
      grid.cell_id(Cell(x, y));
    }
  }
  ASSERT_EQ(grid.size(), 1600u);
  EXPECT_EQ(naex::grid::cell_radius(grid, 3.f), 8);

  const Point2f robot(0.1f, 0.1f); // cell (0, 0)
  const naex::grid::Eviction ev =
      naex::grid::evict_outside_range(grid, robot, 3.f);
  EXPECT_EQ(ev.after, 17u * 17u);
  EXPECT_EQ(grid.size(), 17u * 17u);
  for (naex::grid::CellId v = 0; v < grid.size(); ++v) {
    EXPECT_LE(std::abs(grid.cell(v).x), 8);
    EXPECT_LE(std::abs(grid.cell(v).y), 8);
    // The retained square contains the whole map_range disc.
    EXPECT_EQ(grid.cell_id(grid.cell(v)), v);
  }
  for (int16_t x = -7; x <= 7; ++x) {
    EXPECT_NE(grid.find_cell(Cell(x, 7)), naex::grid::INVALID_CELL_ID)
        << "cell (" << x << ", 7) is within 3 m and must survive";
  }

  // An eviction that removes nothing must not renumber: repeat it.
  const uint64_t version = grid.version();
  const naex::grid::Eviction again =
      naex::grid::evict_outside_range(grid, robot, 3.f);
  EXPECT_FALSE(again.changed());
  EXPECT_EQ(grid.version(), version);
}

TEST(Eviction, PlanningOnACompactedGridIsExact) {
  // The 20x20 gap fixture of Planning.TwentyByTwentyWithGapExactCost, but
  // reached by compacting a 30x30 grid down to the 20x20 core: the plan after
  // the eviction must be bit for bit the plan on the grid built that way.
  Grid grid = make_dense_grid(30);
  // Full-height wall on column x = 10 except the gap at y = 18, 19, so the
  // route is unambiguous both before and after the eviction.
  for (int16_t y = 0; y < 30; ++y) {
    if (y == 18 || y == 19) {
      continue;
    }
    grid.cell_costs(Cell(10, y))[0] = 5.f;
  }
  ASSERT_EQ(grid.size(), 900u);

  const naex::grid::Eviction ev =
      grid.erase_cells([&grid](naex::grid::CellId v) {
        return grid.cell(v).x < 20 && grid.cell(v).y < 20;
      });
  ASSERT_EQ(ev.after, 400u);
  ASSERT_EQ(ev.removed, 500u) << "every CellId is renumbered";
  ASSERT_EQ(grid.size(), 400u);

  const Costs max_costs(1.f);
  const VertexId start = grid.cell_id(Cell(0, 0));
  const VertexId goal = grid.cell_id(Cell(19, 0));
  const ShortestPaths sp(grid, start, 8, max_costs);
  const float expected = 19.f * std::sqrt(2.f) + 17.f;
  EXPECT_NEAR(sp.path_cost(goal), expected, 1e-3f);
  EXPECT_FALSE(std::isfinite(sp.path_cost(grid.cell_id(Cell(10, 5)))));

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
    EXPECT_FLOAT_EQ(grid.cell_costs(Cell(10, y))[0], 5.f) << "y = " << y;
  }
  EXPECT_EQ(grid.find_cell(Cell(20, 0)), naex::grid::INVALID_CELL_ID);
}

TEST(Eviction, EraseCellsOnAnEmptyGrid) {
  Grid grid(0.4f);
  const naex::grid::Eviction ev = grid.evict_outside(Cell(0, 0), 3);
  EXPECT_EQ(ev.before, 0u);
  EXPECT_EQ(ev.after, 0u);
  EXPECT_FALSE(ev.changed());
  EXPECT_TRUE(grid.empty());
}

TEST(Grid, VersionBumpsOnStructuralChangesOnly) {
  Grid grid(1.f, 1.f, Costs(0.f, 0.f, 0.f, 0.f));
  const uint64_t v0 = grid.version();
  grid.cell_id(Cell(0, 0));
  const uint64_t v1 = grid.version();
  EXPECT_GT(v1, v0);
  // An existing cell is not created again.
  grid.cell_id(Cell(0, 0));
  EXPECT_EQ(grid.version(), v1);
  // Reads and cost updates leave the numbering alone.
  grid.update_cell_cost(Cell(0, 0), 0, 3.f);
  EXPECT_EQ(grid.version(), v1);
  EXPECT_EQ(grid.size(), 1u);
  // ... but a cost update that creates a cell does bump it.
  grid.update_cell_cost(Cell(5, 5), 0, 3.f);
  EXPECT_GT(grid.version(), v1);
  const uint64_t v2 = grid.version();
  grid.clear();
  EXPECT_GT(grid.version(), v2);
}

// --- P2: the flat neighbour table ------------------------------------------
// The table replaces the per-edge hash lookups of the Dijkstra inner loop, so
// a stale or mis-wired entry produces a plausible but wrong path.  The tests
// below check it against the hash lookup it replaced, on every path that can
// change a CellId (create_cell, clear, erase_cells), and check the search
// itself against a verbatim copy of the pre-P2 hash-based graph.

namespace {

/// Brute-force recomputation of one neighbour row: exactly the lookup
/// Graph::target() used to do per examined edge.
naex::grid::CellId reference_neighbor(const Grid &grid, naex::grid::CellId v,
                                      int i) {
  const Cell n = naex::grid::neighbor8(grid.cell(v), i);
  return grid.has_cell(n) ? grid.cell_id(n) : naex::grid::INVALID_CELL_ID;
}

/// Every entry of the table equals the hash lookup, and every present entry is
/// symmetric: if b is neighbour i of a then a is neighbour (i + 4) % 8 of b.
void expect_neighbor_table_consistent(const Grid &grid) {
  for (naex::grid::CellId v = 0; v < grid.size(); ++v) {
    for (int i = 0; i < 8; ++i) {
      const naex::grid::CellId n = grid.neighbor_id(v, i);
      EXPECT_EQ(n, reference_neighbor(grid, v, i))
          << "cell " << v << " direction " << i;
      if (n == naex::grid::INVALID_CELL_ID) {
        continue;
      }
      ASSERT_LT(n, grid.size()) << "cell " << v << " direction " << i;
      EXPECT_EQ(grid.neighbor_id(n, (i + 4) % 8), v)
          << "back link of cell " << v << " direction " << i;
    }
  }
}

/// A grid of `count` cells drawn from a `side` x `side` area in a shuffled,
/// seeded order, so that neighbours are created before *and* after each other.
Grid make_shuffled_grid(int16_t side, size_t count, unsigned seed) {
  std::vector<Cell> cells;
  for (int16_t x = 0; x < side; ++x) {
    for (int16_t y = 0; y < side; ++y) {
      cells.push_back(Cell(x, y));
    }
  }
  std::mt19937 rng(seed);
  std::shuffle(cells.begin(), cells.end(), rng);
  cells.resize(std::min(count, cells.size()));
  Grid grid(1.f, 1.f, Costs(0.f, 0.f, 0.f, 0.f));
  for (const Cell &c : cells) {
    grid.cell_id(c);
  }
  return grid;
}

} // namespace

TEST(Grid, NeighborTableAntipodal) {
  // The property the incremental back-link patch relies on.
  for (int i = 0; i < 8; ++i) {
    const Cell c(3, -5);
    const Cell back =
        naex::grid::neighbor8(naex::grid::neighbor8(c, i), (i + 4) % 8);
    EXPECT_EQ(back.x, c.x) << "index " << i;
    EXPECT_EQ(back.y, c.y) << "index " << i;
  }
}

TEST(Grid, NeighborTableMatchesReferenceLookup) {
  // 200 of 400 cells in a shuffled order: every cell has holes around it and
  // both creation orders (neighbour first, neighbour last) occur.
  const Grid grid = make_shuffled_grid(20, 200, 12345u);
  ASSERT_EQ(grid.size(), 200u);
  expect_neighbor_table_consistent(grid);
  // Some entries must actually be missing, or the test proves nothing.
  size_t missing = 0;
  for (naex::grid::CellId v = 0; v < grid.size(); ++v) {
    for (int i = 0; i < 8; ++i) {
      missing += grid.neighbor_id(v, i) == naex::grid::INVALID_CELL_ID;
    }
  }
  EXPECT_GT(missing, 0u);
}

TEST(Grid, NeighborTableGrowsWithEveryCreateOrder) {
  // Incremental maintenance after every single insertion, for the two extreme
  // orders: strictly increasing (neighbours always exist already) and strictly
  // decreasing (every link has to be patched from the other side).
  for (int reverse = 0; reverse < 2; ++reverse) {
    Grid grid(1.f, 1.f, Costs(0.f, 0.f, 0.f, 0.f));
    for (int16_t k = 0; k < 6; ++k) {
      for (int16_t l = 0; l < 6; ++l) {
        const int16_t x = reverse ? static_cast<int16_t>(5 - k) : k;
        const int16_t y = reverse ? static_cast<int16_t>(5 - l) : l;
        grid.cell_id(Cell(x, y));
        expect_neighbor_table_consistent(grid);
      }
    }
    EXPECT_EQ(grid.size(), 36u);
  }
}

TEST(Grid, NeighborTableAfterClear) {
  Grid grid = make_shuffled_grid(10, 60, 7u);
  expect_neighbor_table_consistent(grid);
  grid.clear();
  EXPECT_EQ(grid.size(), 0u);
  // Refilling with a different set must not resurrect a single old link.
  for (int16_t x = 0; x < 4; ++x) {
    for (int16_t y = 0; y < 4; ++y) {
      grid.cell_id(Cell(static_cast<int16_t>(x + 50), y));
    }
  }
  EXPECT_EQ(grid.size(), 16u);
  expect_neighbor_table_consistent(grid);
}

TEST(Grid, NeighborTableAfterEvictionOnRandomGrids) {
  // erase_cells() renumbers the survivors, so the table has to be remapped
  // through Eviction::old_to_new.  Compare against the brute-force lookup on
  // the compacted grid, for a range of seeds and eviction squares.
  for (unsigned seed = 0; seed < 8; ++seed) {
    Grid grid = make_shuffled_grid(20, 250, seed);
    expect_neighbor_table_consistent(grid);
    std::mt19937 rng(seed + 1000u);
    const int16_t cx = static_cast<int16_t>(rng() % 20);
    const int16_t cy = static_cast<int16_t>(rng() % 20);
    const int32_t r = static_cast<int32_t>(rng() % 8);
    const naex::grid::Eviction ev = grid.evict_outside(Cell(cx, cy), r);
    ASSERT_EQ(grid.size(), ev.after);
    expect_neighbor_table_consistent(grid);
    // An eviction that removed nothing must leave the table untouched, and
    // one that removed everything must leave no table at all.
    if (!ev.changed()) {
      EXPECT_TRUE(ev.old_to_new.empty());
    }
    // The grid must still be usable: create a cell next to a survivor.
    if (grid.size() > 0) {
      const Cell c = naex::grid::neighbor8(grid.cell(0), 0);
      grid.cell_id(c);
      expect_neighbor_table_consistent(grid);
    }
  }
}

TEST(Grid, NeighborTableAfterEvictionKeepsEveryLink) {
  // The 20x20 dense fixture cropped to a 7x7 square: every interior link of
  // the square must survive the renumbering, and the boundary must lose
  // exactly the links that pointed outside.
  Grid grid = make_dense_grid(20);
  grid.evict_outside(Cell(10, 10), 3);
  ASSERT_EQ(grid.size(), 49u);
  expect_neighbor_table_consistent(grid);
  size_t links = 0;
  for (naex::grid::CellId v = 0; v < grid.size(); ++v) {
    for (int i = 0; i < 8; ++i) {
      links += grid.neighbor_id(v, i) != naex::grid::INVALID_CELL_ID;
    }
  }
  // 7x7 square: 2*(2*7*6) straight + 2*(2*6*6) diagonal directed links.
  EXPECT_EQ(links, static_cast<size_t>(2 * 2 * 7 * 6 + 2 * 2 * 6 * 6));
}

// The pre-P2 graph, kept verbatim so the searches can be compared edge for
// edge: target() by two hash lookups with the self-edge fallback, cost() by
// recomputing Costs::total() and the bounds check on both endpoints.
namespace ref {

using naex::grid::Cost;
using naex::grid::Costs;
using naex::grid::EdgeId;
using naex::grid::EdgeIter;
using naex::grid::Grid;
using naex::grid::VertexId;
using naex::grid::VertexIter;

class RefGraph {
public:
  static constexpr Cost INF = std::numeric_limits<Cost>::infinity();

  RefGraph(const Grid &grid, uint8_t neighborhood, const Costs &max_costs)
      : grid_(grid), neighborhood_(neighborhood), max_costs_(max_costs) {}
  VertexId num_vertices() const { return grid_.size(); }
  std::pair<VertexIter, VertexIter> vertices() const {
    return {VertexIter(0), VertexIter(num_vertices())};
  }
  std::pair<EdgeIter, EdgeIter> out_edges(const VertexId &u) const {
    return {EdgeIter(neighborhood_ * u), EdgeIter(neighborhood_ * (u + 1))};
  }
  VertexId source(const EdgeId &e) const { return e / neighborhood_; }
  VertexId target_index(const EdgeId &e) const { return e % neighborhood_; }
  VertexId target(const EdgeId &e) const {
    auto s = source(e);
    auto cell = grid_.cell(s);
    auto i = target_index(e);
    if (neighborhood_ == 8) {
      cell = naex::grid::neighbor8(cell, i);
    } else if (neighborhood_ == 4) {
      cell = naex::grid::neighbor4(cell, i);
    }
    if (grid_.has_cell(cell)) {
      return grid_.cell_id(cell);
    }
    return s;
  }
  bool costs_in_bounds(const Costs &costs) const {
    for (size_t i = 0; i < Costs::kSize; ++i) {
      if (!std::isfinite(max_costs_[i])) {
        continue;
      }
      if (!(costs[i] <= max_costs_[i])) {
        return false;
      }
    }
    return true;
  }
  Cost cost(const EdgeId &e) const {
    const auto &c0 = grid_.costs(source(e));
    if (!costs_in_bounds(c0)) {
      return INF;
    }
    const auto &c1 = grid_.costs(target(e));
    if (!costs_in_bounds(c1)) {
      return INF;
    }
    auto cost = 1 + (c0.total() + c1.total()) / 2;
    cost *= grid_.cell_size();
    if (neighborhood_ == 8) {
      cost *= naex::grid::distance8(target_index(e));
    }
    return cost;
  }

private:
  const Grid &grid_;
  const uint8_t neighborhood_;
  const Costs max_costs_;
};

class RefEdgeCosts {
public:
  RefEdgeCosts(const RefGraph &graph) : graph_(graph) {}
  Cost operator[](const EdgeId &e) const { return graph_.cost(e); }

private:
  const RefGraph &graph_;
};

inline std::pair<VertexIter, VertexIter> vertices(const RefGraph &g) {
  return g.vertices();
}
inline VertexId source(EdgeId e, const RefGraph &g) { return g.source(e); }
inline VertexId target(EdgeId e, const RefGraph &g) { return g.target(e); }
inline std::pair<EdgeIter, EdgeIter> out_edges(VertexId u, const RefGraph &g) {
  return g.out_edges(u);
}
inline Cost get(const RefEdgeCosts &map, const EdgeId &key) { return map[key]; }

} // namespace ref

namespace boost {
template <> struct graph_traits<ref::RefGraph> {
  typedef naex::grid::VertexId vertex_descriptor;
  typedef naex::grid::VertexId vertices_size_type;
  typedef naex::grid::EdgeId edge_descriptor;
  typedef naex::grid::EdgeId edges_size_type;
  typedef directed_tag directed_category;
  typedef disallow_parallel_edge_tag edge_parallel_category;
  typedef bidirectional_traversal_tag traversal_category;
  typedef naex::grid::VertexIter vertex_iterator;
  typedef naex::grid::EdgeIter out_edge_iterator;
};
template <> class property_traits<ref::RefEdgeCosts> {
public:
  typedef naex::grid::EdgeId key_type;
  typedef naex::grid::Cost value_type;
  typedef naex::grid::Cost reference;
  typedef readable_property_map_tag category;
};
} // namespace boost

namespace ref {

/// The pre-P2 ShortestPaths: fresh buffers, hash-based graph.
struct RefShortestPaths {
  RefShortestPaths(const Grid &grid, VertexId start, uint8_t neighborhood,
                   const Costs &max_costs)
      : predecessor(grid.size(), std::numeric_limits<VertexId>::max()),
        path_costs(grid.size(), std::numeric_limits<Cost>::infinity()) {
    const RefGraph graph(grid, neighborhood, max_costs);
    const RefEdgeCosts edge_costs(graph);
    boost::typed_identity_property_map<VertexId> index_map;
    boost::dijkstra_shortest_paths_no_color_map(
        graph, start, predecessor.data(), path_costs.data(), edge_costs,
        index_map, std::less<Cost>(), boost::closed_plus<Cost>(),
        std::numeric_limits<Cost>::infinity(), Cost(0.),
        boost::dijkstra_visitor<boost::null_visitor>());
  }
  std::vector<VertexId> predecessor;
  std::vector<Cost> path_costs;
};

} // namespace ref

TEST(ShortestPaths, MatchesThePreP2HashGraphOnRandomGrids) {
  // 20 random sparse grids with random costs, searched from a random start
  // with both neighbourhoods.  Costs are multiples of 0.25 so that ties are
  // exact and both searches break them the same way.
  for (unsigned seed = 0; seed < 20; ++seed) {
    std::mt19937 rng(seed);
    Grid grid(0.4f, 1.f, Costs(0.f, 0.f, 0.f, 0.f));
    for (int16_t x = 0; x < 16; ++x) {
      for (int16_t y = 0; y < 16; ++y) {
        if (rng() % 4 == 0) {
          continue; // hole: no cell at all
        }
        const naex::grid::CellId v = grid.cell_id(Cell(x, y));
        grid.costs(v)[0] = 0.25f * static_cast<float>(rng() % 9);
        grid.costs(v)[1] = (rng() % 8 == 0) ? kNaN : 0.f;
      }
    }
    ASSERT_GT(grid.size(), 0u);
    const VertexId start = static_cast<VertexId>(rng() % grid.size());
    // Half the seeds bound layer 0, so some cells are untraversable.
    const Costs max_costs = (seed % 2) ? Costs(1.f) : Costs();
    for (const uint8_t nb : {uint8_t(4), uint8_t(8)}) {
      const ref::RefShortestPaths expected(grid, start, nb, max_costs);
      const ShortestPaths actual(grid, start, nb, max_costs);
      ASSERT_EQ(actual.path_costs().size(), expected.path_costs.size());
      for (VertexId v = 0; v < grid.size(); ++v) {
        const Cost a = actual.path_cost(v);
        const Cost b = expected.path_costs[v];
        ASSERT_EQ(std::isfinite(a), std::isfinite(b))
            << "seed " << seed << " nb " << int(nb) << " vertex " << v;
        if (std::isfinite(b)) {
          EXPECT_FLOAT_EQ(a, b)
              << "seed " << seed << " nb " << int(nb) << " vertex " << v;
        }
        EXPECT_EQ(actual.predecessor(v), expected.predecessor[v])
            << "seed " << seed << " nb " << int(nb) << " vertex " << v;
      }
    }
  }
}

TEST(ShortestPaths, MatchesThePreP2HashGraphAfterEviction) {
  // Same comparison on a compacted grid, i.e. against a remapped table.
  Grid grid = make_shuffled_grid(20, 300, 99u);
  std::mt19937 rng(5u);
  for (naex::grid::CellId v = 0; v < grid.size(); ++v) {
    grid.costs(v)[0] = 0.25f * static_cast<float>(rng() % 5);
  }
  grid.evict_outside(Cell(9, 9), 5);
  ASSERT_GT(grid.size(), 0u);
  expect_neighbor_table_consistent(grid);
  const ref::RefShortestPaths expected(grid, 0, 8, Costs(0.75f));
  const ShortestPaths actual(grid, 0, 8, Costs(0.75f));
  for (VertexId v = 0; v < grid.size(); ++v) {
    ASSERT_EQ(std::isfinite(actual.path_cost(v)),
              std::isfinite(expected.path_costs[v]))
        << "vertex " << v;
    if (std::isfinite(expected.path_costs[v])) {
      EXPECT_FLOAT_EQ(actual.path_cost(v), expected.path_costs[v])
          << "vertex " << v;
    }
    EXPECT_EQ(actual.predecessor(v), expected.predecessor[v]) << "vertex " << v;
  }
}

TEST(Graph, TargetFallsBackToSourceAtTheBorder) {
  // A single isolated cell has no neighbour at all: target() still reports the
  // source, and the edge is inert because its cost is INF (pre-P2 it was a
  // finite self-edge, which Dijkstra could not relax either).
  Grid grid(1.f, 1.f, Costs(0.f, 0.f, 0.f, 0.f));
  const VertexId v = grid.cell_id(Cell(4, 4));
  const Graph graph(grid, Costs());
  for (naex::grid::EdgeId e = 0; e < 8; ++e) {
    EXPECT_EQ(graph.target(e), v) << "edge " << e;
    EXPECT_FALSE(std::isfinite(graph.cost(e))) << "edge " << e;
  }
  // A search from it terminates and reaches nothing else.
  const ShortestPaths sp(grid, v, 8, Costs());
  EXPECT_FLOAT_EQ(sp.path_cost(v), 0.f);
  EXPECT_EQ(sp.predecessor(v), v);
}

TEST(Graph, CostUsesTheTableAndTheCachedTotals) {
  // Two adjacent cells: the straight edge costs cell_size, the diagonal one
  // cell_size * sqrt(2), and a cost on either endpoint enters as half.
  Grid grid(0.4f, 1.f, Costs(0.f, 0.f, 0.f, 0.f));
  const VertexId a = grid.cell_id(Cell(0, 0));
  const VertexId b = grid.cell_id(Cell(1, 0));
  const VertexId c = grid.cell_id(Cell(1, 1));
  grid.costs(b)[0] = 2.f;
  const Graph graph(grid, Costs());
  // Edge a -> b is direction 0 (+x), a -> c is direction 1 (+x+y).
  EXPECT_FLOAT_EQ(graph.cost(8 * a + 0), (1.f + 1.f) * 0.4f);
  EXPECT_FLOAT_EQ(graph.cost(8 * a + 1), 1.f * 0.4f * std::sqrt(2.f));
  EXPECT_EQ(graph.target(8 * a + 0), b);
  EXPECT_EQ(graph.target(8 * a + 1), c);
  // Direction 4 (-x) does not exist: inert edge.
  EXPECT_EQ(graph.target(8 * a + 4), a);
  EXPECT_FALSE(std::isfinite(graph.cost(8 * a + 4)));
}

TEST(Planning, NeighborhoodFourMatchesEight) {
  // The 20x20 gap fixture, 4-connected: the shared 8-slot table is read at the
  // even indices only, so the cost must be the integral Manhattan optimum.
  Grid grid = make_dense_grid(20);
  for (int16_t y = 0; y <= 17; ++y) {
    grid.cell_costs(Cell(10, y))[0] = 5.f;
  }
  const Costs max_costs(1.f);
  const VertexId start = grid.cell_id(Cell(0, 0));
  const VertexId goal = grid.cell_id(Cell(19, 0));
  const ShortestPaths sp(grid, start, 4, max_costs);
  // Around the wall through the gap at (10, 18): 19 in x and 2 * 18 in y.
  EXPECT_NEAR(sp.path_cost(goal), 19.f + 36.f, 1e-3f);
  EXPECT_FALSE(std::isfinite(sp.path_cost(grid.cell_id(Cell(10, 5)))));
  // The 8-connected search on the same grid is the one the exact-cost guard
  // pins; it must stay strictly cheaper.
  const ShortestPaths sp8(grid, start, 8, max_costs);
  EXPECT_LT(sp8.path_cost(goal), sp.path_cost(goal));
}

TEST(Grid, ReserveDoesNotChangeContent) {
  // P10: reserve() only grows capacity; the table and the ids stay put.
  Grid grid = make_shuffled_grid(10, 60, 3u);
  const size_t n = grid.size();
  const std::vector<naex::grid::CellId> before = [&] {
    std::vector<naex::grid::CellId> v;
    for (naex::grid::CellId i = 0; i < n; ++i) {
      for (int k = 0; k < 8; ++k) {
        v.push_back(grid.neighbor_id(i, k));
      }
    }
    return v;
  }();
  grid.reserve(100000);
  EXPECT_EQ(grid.size(), n);
  grid.reserve(1); // smaller: ignored
  EXPECT_EQ(grid.size(), n);
  for (naex::grid::CellId i = 0; i < n; ++i) {
    for (int k = 0; k < 8; ++k) {
      EXPECT_EQ(grid.neighbor_id(i, k), before[8 * i + k]);
    }
  }
  expect_neighbor_table_consistent(grid);
}

TEST(Grid, CellHashIsPerfect) {
  // P10: the packed hash must be injective over the int16 pair domain.
  const naex::grid::CellHasher hash;
  std::set<std::size_t> seen;
  const int16_t values[] = {-32768, -32767, -1, 0, 1, 32766, 32767};
  for (const int16_t x : values) {
    for (const int16_t y : values) {
      EXPECT_TRUE(seen.insert(hash(Cell(x, y))).second)
          << "collision at (" << x << ", " << y << ")";
    }
  }
  const size_t n = sizeof(values) / sizeof(values[0]);
  EXPECT_EQ(seen.size(), n * n);
}

// ---------------------------------------------------------------------------
// A* (astar branch), ported onto the flat neighbour table.
// ---------------------------------------------------------------------------

TEST(Graph, NeighborDegreeCountsExistingNeighborsOnly) {
  // The frontier detection counts neighbours through the table instead of
  // walking out_edges of a filtered graph; the count must be the number of
  // existing neighbours that pass the predicate.
  const Grid grid = make_dense_grid(5);
  const auto all = [](naex::grid::CellId) { return true; };
  EXPECT_EQ(naex::grid::neighbor_degree(grid, grid.cell_id(Cell(2, 2)), 8, all),
            8);
  EXPECT_EQ(naex::grid::neighbor_degree(grid, grid.cell_id(Cell(2, 2)), 4, all),
            4);
  // Corner: 3 of 8, and 2 of 4.
  EXPECT_EQ(naex::grid::neighbor_degree(grid, grid.cell_id(Cell(0, 0)), 8, all),
            3);
  EXPECT_EQ(naex::grid::neighbor_degree(grid, grid.cell_id(Cell(0, 0)), 4, all),
            2);
  // A predicate that drops everything leaves degree 0, which is what makes an
  // out-of-range cell look like a frontier.
  const auto none = [](naex::grid::CellId) { return false; };
  EXPECT_EQ(
      naex::grid::neighbor_degree(grid, grid.cell_id(Cell(2, 2)), 8, none), 0);
}

TEST(AStar, MatchesDijkstraOnTheGapFixture) {
  // Same fixture as Planning.TwentyByTwentyWithGapExactCost: A* must find the
  // same optimal cost and the same 37-vertex route, because the Euclidean
  // heuristic is consistent for these edge costs.
  Grid grid = make_dense_grid(20);
  for (int16_t y = 0; y <= 17; ++y) {
    grid.cell_costs(Cell(10, y))[0] = 5.f;
  }
  const Costs max_costs(1.f);
  const VertexId start = grid.cell_id(Cell(0, 0));
  const VertexId goal = grid.cell_id(Cell(19, 0));
  const naex::Vec3 goal_point(grid.point(goal).x, grid.point(goal).y, 0.f);

  ShortestPaths sp;
  const bool found =
      sp.compute_astar(rclcpp::get_logger("test_grid"), grid, start, goal_point,
                       true, 1000.f, 8, max_costs);
  EXPECT_TRUE(found);
  EXPECT_TRUE(sp.found_goal());
  EXPECT_TRUE(sp.is_astar());

  const float expected = 19.f * std::sqrt(2.f) + 17.f;
  EXPECT_NEAR(sp.path_cost(goal), expected, 1e-3f);

  const ShortestPaths dijkstra(grid, start, 8, max_costs);
  EXPECT_NEAR(sp.path_cost(goal), dijkstra.path_cost(goal), 1e-4f);

  size_t vertices = 1;
  VertexId v = goal;
  while (v != start && vertices <= grid.size()) {
    EXPECT_FLOAT_EQ(grid.costs(v)[0], 0.f) << "path entered a blocked cell";
    v = sp.predecessor(v);
    ++vertices;
  }
  ASSERT_EQ(v, start);
  EXPECT_EQ(vertices, 37u);

  // The wall is out of bounds, so no edge into it is ever relaxed.  Boost's
  // A* leaves an unreached vertex at numeric_limits<Cost>::max() rather than
  // at infinity (its default distance_inf), which is why every caller tests
  // "> kUnreachableCost" and not "!isfinite".
  EXPECT_GT(sp.path_cost(grid.cell_id(Cell(10, 5))),
            naex::grid::ShortestPaths::kUnreachableCost);
  EXPECT_GT(sp.f_value(grid.cell_id(Cell(10, 5))),
            naex::grid::ShortestPaths::kUnreachableCost);
  EXPECT_LT(sp.path_cost(goal), naex::grid::ShortestPaths::kUnreachableCost);

  // The predecessor path length in metres is what the frontier decision uses.
  // It skips the last edge (the one leaving the goal cell), as it always has.
  EXPECT_NEAR(sp.cheapest_path_euclidean_dist(grid, start, goal),
              19.f * std::sqrt(2.f) + 17.f - std::sqrt(2.f), 1e-3f);
}

TEST(AStar, RangeCropBoundsTheExpansion) {
  // astar_max_range drops cells farther than the radius from the start cell;
  // they are neither expanded nor reachable, whatever the costs say.
  const Grid grid = make_dense_grid(20);
  const Costs max_costs(1.f);
  const VertexId start = grid.cell_id(Cell(0, 0));
  const naex::Vec3 goal_point(19.f, 0.f, 0.f);

  ShortestPaths sp;
  sp.compute_astar(rclcpp::get_logger("test_grid"), grid, start, goal_point,
                   true, 5.f, 8, max_costs);
  EXPECT_FALSE(sp.found_goal()) << "the goal is 19 m away, the crop is 5 m";
  // Cell size is 1 m and the start cell centre is the origin of the crop.
  EXPECT_EQ(sp.visited()[grid.cell_id(Cell(3, 3))], 1);
  EXPECT_EQ(sp.visited()[grid.cell_id(Cell(10, 10))], 0);
  // Cropped-out vertices are not even initialized by boost, so they keep the
  // infinity the buffers were filled with.
  EXPECT_FALSE(std::isfinite(sp.path_cost(grid.cell_id(Cell(10, 10)))));
  EXPECT_GT(sp.path_cost(grid.cell_id(Cell(10, 10))),
            naex::grid::ShortestPaths::kUnreachableCost);
  // Without the crop the same goal is reached.
  ShortestPaths full;
  EXPECT_TRUE(full.compute_astar(rclcpp::get_logger("test_grid"), grid, start,
                                 goal_point, true, 1000.f, 8, max_costs));
}

TEST(AStar, ReusesItsBuffersAcrossRuns) {
  // The buffers are members (P2); a second run must not see the first one's
  // state, whichever search ran before.
  Grid grid = make_dense_grid(10);
  const Costs max_costs(1.f);
  const VertexId start = grid.cell_id(Cell(0, 0));
  const VertexId goal = grid.cell_id(Cell(9, 9));
  const naex::Vec3 goal_point(9.f, 9.f, 0.f);

  ShortestPaths sp;
  sp.compute_astar(rclcpp::get_logger("test_grid"), grid, start, goal_point,
                   true, 1000.f, 8, max_costs);
  const Cost first = sp.path_cost(goal);
  // Dijkstra on the same grid, then A* again.
  sp.compute(grid, start, 8, max_costs);
  EXPECT_FALSE(sp.is_astar());
  EXPECT_NEAR(sp.path_cost(goal), first, 1e-4f);
  EXPECT_EQ(sp.visited()[goal], 1);
  sp.compute_astar(rclcpp::get_logger("test_grid"), grid, start, goal_point,
                   true, 1000.f, 8, max_costs);
  EXPECT_TRUE(sp.found_goal());
  EXPECT_NEAR(sp.path_cost(goal), first, 1e-4f);
}

// --- trace_path_vertices (shared by grid_planner and mule_planner) --------

TEST(TracePathVertices, WalksBackFromV1ToV0) {
  // predecessor[v] == v's predecessor; 0 is its own predecessor (the root).
  const std::vector<VertexId> predecessor = {0, 0, 1, 2};
  const auto path = trace_path_vertices(0, 3, predecessor);
  const std::vector<VertexId> expected = {0, 1, 2, 3};
  EXPECT_EQ(path, expected);
}

TEST(TracePathVertices, CyclicPredecessorMapTerminates) {
  // 0 is the (claimed) root, but 1 and 2 form a cycle unreachable from it --
  // a bug in whatever built the map.  Tracing back from 3 must still
  // terminate instead of looping forever.
  const std::vector<VertexId> predecessor = {0, 2, 1, 2};
  std::vector<VertexId> path;
  trace_path_vertices(0, 3, predecessor, path);
  EXPECT_FALSE(path.empty());
  EXPECT_LE(path.size(), 2 * predecessor.size());
}
