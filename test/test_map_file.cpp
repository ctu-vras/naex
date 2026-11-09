// Unit tests for saving and reloading the cost grid.
#include "naex/grid/grid.h"
#include "naex/grid/map_file.h"

#include <gtest/gtest.h>

#include <cmath>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <limits>
#include <stdexcept>
#include <string>
#include <unistd.h>

using naex::grid::Cell;
using naex::grid::CellId;
using naex::grid::copy_cells;
using naex::grid::Cost;
using naex::grid::Costs;
using naex::grid::describe_config_mismatch;
using naex::grid::Grid;
using naex::grid::INVALID_CELL_ID;
using naex::grid::MapCells;
using naex::grid::MapConfig;
using naex::grid::MapPose;
using naex::grid::MapSnapshot;
using naex::grid::merge_cells;
using naex::grid::read_map_file;
using naex::grid::relative_rigid2;
using naex::grid::resample_cells;
using naex::grid::Rigid2;
using naex::grid::write_map_file;

namespace fs = std::filesystem;

namespace {

constexpr Cost kNaN = std::numeric_limits<Cost>::quiet_NaN();
constexpr float kCellSize = 0.2f;

MapConfig make_config() {
  MapConfig config;
  config.cell_size = kCellSize;
  config.earth_frame = "FP_ECEF";
  config.cost_fields = {"cost", "traversability"};
  config.cloud_levels = {0, 1};
  config.cloud_weights = {0.5, 1.0};
  config.default_costs = Costs(7.5f, 0.4f);
  return config;
}

MapSnapshot make_snapshot() {
  MapSnapshot map;
  map.config = make_config();
  map.map_frame = "FP_ENU0";
  map.earth_from_map = {4.0e6, 1.0e6, 4.8e6, 0.1, 0.2, 0.3, 0.9};
  map.cells.cells = {Cell(1, 2), Cell(-3, 4), Cell(32000, -32000)};
  map.cells.costs = {Costs(1.f, 2.f), Costs(kNaN, 3.f, 4.f, 5.f), Costs(0.5f)};
  return map;
}

/// A file path in a directory of its own, removed with the test.
class MapFile : public ::testing::Test {
protected:
  void SetUp() override {
    dir_ = fs::temp_directory_path() /
           ("naex_map_file_" + std::to_string(::getpid()) + "_" +
            ::testing::UnitTest::GetInstance()->current_test_info()->name());
    fs::remove_all(dir_);
    path_ = (dir_ / "sub" / "map.bin").string();
  }
  void TearDown() override { fs::remove_all(dir_); }

  fs::path dir_;
  std::string path_;
};

void expect_same_cost(Cost a, Cost b) {
  if (std::isnan(a)) {
    EXPECT_TRUE(std::isnan(b));
  } else {
    EXPECT_EQ(a, b);
  }
}

/// w x h block of cells whose first cost encodes its index.
MapCells make_block(int w, int h, int x0 = 0, int y0 = 0) {
  MapCells block;
  for (int x = 0; x < w; ++x) {
    for (int y = 0; y < h; ++y) {
      block.cells.emplace_back(static_cast<int16_t>(x0 + x),
                               static_cast<int16_t>(y0 + y));
      block.costs.emplace_back(static_cast<Cost>(x * 1000 + y));
    }
  }
  return block;
}

Grid to_grid(const MapCells &cells) {
  Grid grid(kCellSize);
  merge_cells(grid, cells);
  return grid;
}

} // namespace

TEST_F(MapFile, RoundTripsEveryField) {
  const MapSnapshot in = make_snapshot();
  write_map_file(path_, in);
  EXPECT_FALSE(fs::exists(path_ + ".tmp"));

  const MapSnapshot out = read_map_file(path_);
  EXPECT_TRUE(describe_config_mismatch(in.config, out.config).empty());
  EXPECT_EQ(in.map_frame, out.map_frame);
  EXPECT_EQ(in.earth_from_map, out.earth_from_map);
  ASSERT_EQ(in.cells.cells.size(), out.cells.cells.size());
  for (size_t i = 0; i < in.cells.cells.size(); ++i) {
    EXPECT_EQ(in.cells.cells[i], out.cells.cells[i]);
    for (size_t l = 0; l < Costs::kSize; ++l) {
      expect_same_cost(in.cells.costs[i][l], out.cells.costs[i][l]);
    }
  }
}

TEST_F(MapFile, OverwriteKeepsOnlyTheLatest) {
  MapSnapshot map = make_snapshot();
  write_map_file(path_, map);
  map.cells.cells.resize(1);
  map.cells.costs.resize(1);
  write_map_file(path_, map);
  EXPECT_EQ(read_map_file(path_).cells.cells.size(), 1u);
}

TEST_F(MapFile, FailedWriteLeavesThePreviousMap) {
  MapSnapshot map = make_snapshot();
  write_map_file(path_, map);
  // A directory in the way of the temp file makes the write fail.
  fs::create_directory(path_ + ".tmp");
  EXPECT_THROW(write_map_file(path_, map), std::runtime_error);
  fs::remove(path_ + ".tmp");
  EXPECT_EQ(read_map_file(path_).cells.cells.size(), 3u);
}

TEST_F(MapFile, RejectsDamagedFiles) {
  write_map_file(path_, make_snapshot());
  std::ifstream in(path_, std::ios::binary);
  const std::string good((std::istreambuf_iterator<char>(in)),
                         std::istreambuf_iterator<char>());
  const auto rejected = [&](const std::string &contents) {
    std::ofstream(path_, std::ios::binary | std::ios::trunc) << contents;
    EXPECT_THROW(read_map_file(path_), std::runtime_error);
  };
  rejected("");
  rejected(good.substr(0, 4));
  rejected("X" + good.substr(1));
  // Truncated at every length, so no read can run past the end.
  for (size_t n = 8; n < good.size(); n += 7) {
    rejected(good.substr(0, n));
  }
  rejected(good + "x");
}

TEST_F(MapFile, MissingFileThrows) {
  EXPECT_THROW(read_map_file(path_), std::runtime_error);
}

TEST(MapConfig, MismatchIsDetectedPerField) {
  const MapConfig base = make_config();
  EXPECT_TRUE(describe_config_mismatch(base, base).empty());

  MapConfig other = base;
  other.cell_size = 0.6f;
  EXPECT_FALSE(describe_config_mismatch(base, other).empty());
  other = base;
  other.earth_frame = "earth";
  EXPECT_FALSE(describe_config_mismatch(base, other).empty());
  other = base;
  other.cost_fields = {"traversability"};
  EXPECT_FALSE(describe_config_mismatch(base, other).empty());
  other = base;
  other.cloud_levels = {1, 0};
  EXPECT_FALSE(describe_config_mismatch(base, other).empty());
  other = base;
  other.cloud_weights = {0.5, 2.0};
  EXPECT_FALSE(describe_config_mismatch(base, other).empty());
  other = base;
  other.default_costs = Costs(7.5f, 0.5f);
  EXPECT_FALSE(describe_config_mismatch(base, other).empty());
}

TEST(RelativeRigid2, RecoversAShiftBetweenFramesFarFromTheOrigin) {
  // Same orientation, the new origin 12.3 m further along x of the frame.
  const MapPose old_pose = {4.0e6, 1.0e6, 4.8e6, 0.0, 0.0, 0.0, 1.0};
  const MapPose new_pose = {4.0e6 + 12.3, 1.0e6, 4.8e6, 0.0, 0.0, 0.0, 1.0};
  const Rigid2 t = relative_rigid2(old_pose, new_pose);
  EXPECT_NEAR(t.c, 1.0, 1e-12);
  EXPECT_NEAR(t.s, 0.0, 1e-12);
  EXPECT_NEAR(t.tx, -12.3, 1e-6);
  EXPECT_NEAR(t.ty, 0.0, 1e-6);
}

TEST(RelativeRigid2, RecoversAYaw) {
  // The old frame is rotated +90 deg about z relative to the new one.
  const double h = std::sqrt(0.5);
  const MapPose old_pose = {1.0, 2.0, 3.0, 0.0, 0.0, h, h};
  const MapPose new_pose = {1.0, 2.0, 3.0, 0.0, 0.0, 0.0, 1.0};
  const Rigid2 t = relative_rigid2(old_pose, new_pose);
  EXPECT_NEAR(t.c, 0.0, 1e-12);
  EXPECT_NEAR(t.s, 1.0, 1e-12);
}

TEST(ResampleCells, IdentityIsExact) {
  const MapCells old = make_block(20, 30, -5, -7);
  const MapCells out = resample_cells(old, kCellSize, Rigid2());
  ASSERT_EQ(out.cells.size(), old.cells.size());
  const Grid grid = to_grid(out);
  for (size_t i = 0; i < old.cells.size(); ++i) {
    const CellId v = grid.find_cell(old.cells[i]);
    ASSERT_NE(v, INVALID_CELL_ID);
    EXPECT_EQ(grid.costs(v)[0], old.costs[i][0]);
  }
}

TEST(ResampleCells, WholeCellShiftIsExact) {
  const MapCells old = make_block(10, 10);
  Rigid2 t;
  t.tx = 3 * kCellSize;
  t.ty = -2 * kCellSize;
  const MapCells out = resample_cells(old, kCellSize, t);
  ASSERT_EQ(out.cells.size(), old.cells.size());
  const Grid grid = to_grid(out);
  for (size_t i = 0; i < old.cells.size(); ++i) {
    const CellId v =
        grid.find_cell(Cell(old.cells[i].x + 3, old.cells[i].y - 2));
    ASSERT_NE(v, INVALID_CELL_ID);
    EXPECT_EQ(grid.costs(v)[0], old.costs[i][0]);
  }
}

TEST(ResampleCells, FractionalShiftMovesCostsByLessThanACell) {
  const MapCells old = make_block(10, 10);
  Rigid2 t;
  t.tx = 0.3 * kCellSize;
  const MapCells out = resample_cells(old, kCellSize, t);
  // 0.3 of a cell keeps each target centre in the same-index old cell.
  const Grid grid = to_grid(out);
  EXPECT_EQ(grid.costs(grid.find_cell(Cell(4, 4)))[0], 4004.f);
}

TEST(ResampleCells, AnyRotationLeavesNoHoles) {
  const MapCells old = make_block(200, 200, -100, -100);
  for (const double yaw : {2e-4, 0.1, 0.7, 1.5}) {
    Rigid2 t;
    t.c = std::cos(yaw);
    t.s = std::sin(yaw);
    t.tx = 0.37;
    t.ty = -0.11;
    const Grid grid = to_grid(resample_cells(old, kCellSize, t));
    // Every target whose centre is well inside the rotated block must exist.
    for (int x = -50; x <= 50; ++x) {
      for (int y = -50; y <= 50; ++y) {
        EXPECT_NE(grid.find_cell(Cell(x, y)), INVALID_CELL_ID)
            << "yaw " << yaw << " x " << x << " y " << y;
      }
    }
  }
}

TEST(ResampleCells, DropsCellsOutsideTheAddressableRange) {
  const MapCells old = make_block(4, 1, 32760);
  Rigid2 t;
  t.tx = 5 * kCellSize;
  const MapCells out = resample_cells(old, kCellSize, t);
  for (const Cell &c : out.cells) {
    EXPECT_LE(c.x, 32766);
  }
  EXPECT_LT(out.cells.size(), old.cells.size());
}

TEST(MergeCells, ObservedCellsWin) {
  Grid grid(kCellSize);
  grid.cell_costs(Cell(0, 0)) = Costs(42.f);
  const CellId live = grid.find_cell(Cell(0, 0));

  MapCells loaded;
  loaded.cells = {Cell(0, 0), Cell(1, 0)};
  loaded.costs = {Costs(1.f), Costs(2.f)};

  EXPECT_EQ(merge_cells(grid, loaded), 1u);
  EXPECT_EQ(grid.find_cell(Cell(0, 0)), live);
  EXPECT_EQ(grid.costs(live)[0], 42.f);
  EXPECT_EQ(grid.costs(grid.find_cell(Cell(1, 0)))[0], 2.f);
}

TEST(MergeCells, WiresTheNeighbourTable) {
  Grid grid(kCellSize);
  const CellId a = grid.cell_id(Cell(0, 0));

  MapCells loaded;
  loaded.cells = {Cell(1, 0)};
  loaded.costs = {Costs(2.f)};
  merge_cells(grid, loaded);

  const CellId b = grid.find_cell(Cell(1, 0));
  EXPECT_EQ(grid.neighbor_id(a, 0), b);
  EXPECT_EQ(grid.neighbor_id(b, 4), a);
}

TEST(CopyCells, FollowsCellIdOrder) {
  Grid grid(kCellSize);
  grid.cell_costs(Cell(5, 5)) = Costs(1.f);
  grid.cell_costs(Cell(-2, 3)) = Costs(2.f);
  const MapCells copy = copy_cells(grid);
  ASSERT_EQ(copy.cells.size(), 2u);
  EXPECT_EQ(copy.cells[0], Cell(5, 5));
  EXPECT_EQ(copy.costs[1][0], 2.f);
}
