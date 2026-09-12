#pragma once

#include <Eigen/Dense>
#include <cstdint>
#include <limits>

namespace naex {

// Basic floating-point and index types
typedef float Elem;
typedef Elem Value;
// typedef uint32_t Index;
typedef int Index;
// typedef size_t Index;

// Arrays and matrices
typedef Eigen::Matrix<Value, 3, 1, Eigen::DontAlign> Vec3;
typedef Eigen::Map<Vec3> Vec3Map;
typedef Eigen::Map<const Vec3> ConstVec3Map;
static_assert(sizeof(Vec3) == 3 * sizeof(Value));

typedef Eigen::Matrix<Value, 4, 1, Eigen::DontAlign> Vec4;
static_assert(sizeof(Vec4) == 4 * sizeof(Value));

typedef Eigen::Matrix<Value, 3, 3, Eigen::DontAlign> Mat3;

typedef Eigen::Quaternion<Value, Eigen::DontAlign> Quat;

// Vertex and edge indices
// TODO: Rename both to Index (to be used elsewhere too).
typedef Index Vertex;
typedef Index Edge;
// Edge cost or length
typedef Elem Cost;

enum Flags {
  // A static point, not dynamic or empty, necessary for being traversable.
  STATIC = 1 << 1,
  // Approximately horizontal orientation based on normal direction,
  // necessary condition for being traversable.
  HORIZONTAL = 1 << 2,
  // A point at the edge, i.e. a frontier.
  EDGE = 1 << 4,
  // Traversable based on terrain roughness and obstacles in neighborhood.
  TRAVERSABLE = 1 << 5
};

const Vertex INVALID_VERTEX = std::numeric_limits<Vertex>::max();

class Point {
public:
  Point() {}

  Value position_[3] = {std::numeric_limits<Value>::quiet_NaN(),
                        std::numeric_limits<Value>::quiet_NaN(),
                        std::numeric_limits<Value>::quiet_NaN()};
  // Geometric features
  // TODO: More compact normal representation? Maybe just for sharing,
  // impacts on memory is small compared to neighbors.
  // TODO: Switch to compact (u)int8 types where possible.
  Value normal_[3] = {std::numeric_limits<Value>::quiet_NaN(),
                      std::numeric_limits<Value>::quiet_NaN(),
                      std::numeric_limits<Value>::quiet_NaN()};
  //    int8 normal_[3];
  // Normal scale is common to all points.
  // Number of points used in normal computation.
  uint8_t normal_support_{0};
  // Roughness features (in neighborhood radius).
  // from ball neighborhood
  Value ground_diff_std_{std::numeric_limits<Value>::quiet_NaN()};
  // circle in ground plane
  Value min_ground_diff_{std::numeric_limits<Value>::quiet_NaN()};
  Value max_ground_diff_{std::numeric_limits<Value>::quiet_NaN()};
  Value mean_abs_ground_diff_{std::numeric_limits<Value>::quiet_NaN()};
  Value coverage_{0};
  Value self_coverage_{0};
  // Distance to nearest obstacle (non horizontal point).
  Value dist_to_obstacle_{std::numeric_limits<Value>::quiet_NaN()};
  // Point flags accoring to Flags.
  uint8_t flags_{0};
  // Number of occurences of empty/occupied state.
  uint8_t num_empty_{0};
  uint8_t num_occupied_{0};
  Value dist_to_plane_{std::numeric_limits<Value>::quiet_NaN()};
  // Number of obstacle points in clearance cylinder.
  uint8_t num_obstacle_pts_{0};
  // Number of obstacles nearby.
  uint8_t num_obstacle_neighbors_{0};
  // Number of edge points nearby.
  uint8_t num_edge_neighbors_{0};
  // Planning costs and rewards
  Value path_cost_{std::numeric_limits<Value>::quiet_NaN()};
  Value reward_{std::numeric_limits<Value>::quiet_NaN()};
  Value relative_cost_{std::numeric_limits<Value>::quiet_NaN()};
};

class Neighborhood {
public:
  Neighborhood() {}

  // TODO: Make K_NEIGHBORS a parameter.
  static constexpr Index K_NEIGHBORS = 48;
  Value position_[3] = {std::numeric_limits<Value>::quiet_NaN(),
                        std::numeric_limits<Value>::quiet_NaN(),
                        std::numeric_limits<Value>::quiet_NaN()};
  // NN Graph
  Index neighbors_[K_NEIGHBORS] = {0};
  // Treat zero distance and cost as invalid.
  Value distances_[K_NEIGHBORS] = {0};
  Value costs_[K_NEIGHBORS] = {0};
};

} // namespace naex
