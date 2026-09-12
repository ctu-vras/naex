// Unit tests for the grid planner data structures.
#include "naex/grid/graph.h"
#include "naex/grid/grid.h"
#include "naex/grid/search.h"

#include <gtest/gtest.h>

#include <cmath>
#include <cstdlib>
#include <limits>
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
