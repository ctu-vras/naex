#pragma once
// Incrementally built point map with a k-NN traversability graph.
//
// The templates and the hot graph accessors stay in the header; the remaining
// Map member functions are defined under src/map/ (library naex_core).
#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <geometry_msgs/msg/transform.hpp>
#include <limits>
#include <memory>
#include <mutex>
#include <naex/flann.h>
#include <naex/geom.h>
#include <naex/iterators.h>
#include <naex/nearest_neighbors.h>
#include <naex/timer.h>
#include <naex/types.h>
#include <rclcpp/clock.hpp>
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <unordered_set>
#include <utility>
#include <vector>

namespace naex {

inline rclcpp::Logger map_logger() { return rclcpp::get_logger("naex.map"); }

/**
 * Incrementally built point map with a k-NN traversability graph.
 *
 * The map owns a point cloud (@ref cloud_), a parallel neighborhood graph
 * (@ref graph_) and a FLANN index over the point positions. Point features,
 * labels and edge costs are recomputed lazily for the dirty indices.
 */
class Map {
public:
  typedef std::recursive_mutex Mutex;
  typedef std::lock_guard<Mutex> Lock;

  static const size_t DEFAULT_CAPACITY = 10000000;

  Map();

  /// FLANN view of the point positions in [start, end).
  FlannMat position_matrix(Index start = 0, Index end = 0);

  /// FLANN view of the neighbor indices of the whole graph.
  flann::Matrix<int> neighbor_matrix();

  Value *position(size_t i) { return &cloud_[i].position_[0]; }

  const Value *position(size_t i) const { return &cloud_[i].position_[0]; }

  Value *normal(size_t i) { return &cloud_[i].normal_[0]; }

  const Value *normal(size_t i) const { return &cloud_[i].normal_[0]; }

  /// True if the point has been seen through often enough to be removed.
  bool point_empty(const Point &p) const;

  /// True if point @p i is within @p radius of any of the given positions.
  bool point_near(Index i, const std::vector<Value> &points,
                  Value radius) const;

  inline bool valid_cost(const Value cost) const { return cost > 0; }

  /// Traversal cost of the edge, infinity if the edge cannot be traversed.
  Cost compute_edge_cost(const Edge &e);

  inline Cost edge_cost(const Edge &e) const {
    const auto v0 = source(e);
    const auto v1_index = target_index(e);
    const auto &stored = graph_[v0].costs_[v1_index];
    return valid_cost(stored) ? stored : std::numeric_limits<Value>::infinity();
  }

  /// Rebuild the FLANN index from the current points.
  void update_index();

  std::vector<Index> collect_points_to_update();

  template <typename It> void update_neighborhood(It begin, It end) {
    // NB: It should be a stable iteration order.
    Timer t;

    std::vector<Neighborhood> dirty_cloud;

    for (It it = begin; it != end; ++it) {
      dirty_cloud.push_back(graph_[*it]);
    }

    if (dirty_cloud.empty()) {
      RCLCPP_DEBUG(map_logger(),
                   "Graph up to date, no points to update (%.3f s).",
                   t.seconds_elapsed());
      return;
    }

    flann::Matrix<Value> positions(dirty_cloud[0].position_, dirty_cloud.size(),
                                   3, sizeof(Neighborhood));
    flann::Matrix<Index> neighbors(
        dirty_cloud[0].neighbors_, dirty_cloud.size(),
        Neighborhood::K_NEIGHBORS, sizeof(Neighborhood));
    flann::Matrix<Value> distances(
        dirty_cloud[0].distances_, dirty_cloud.size(),
        Neighborhood::K_NEIGHBORS, sizeof(Neighborhood));

    t.reset();
    flann::SearchParams params;
    params.checks = 64;
    params.cores = 0;
    params.max_neighbors = Neighborhood::K_NEIGHBORS;
    params.sorted = true;
    {
      Lock cloud_lock(cloud_mutex_);
      Lock index_lock(index_mutex_);
      index_->knnSearch(positions, neighbors, distances,
                        Neighborhood::K_NEIGHBORS, params);
    }
    // Propagate NN info into the main graph.
    auto point_it = dirty_cloud.begin();
    for (It it = begin; it != end; ++it, ++point_it) {
      std::copy((*point_it).neighbors_,
                (*point_it).neighbors_ + Neighborhood::K_NEIGHBORS,
                graph_[*it].neighbors_);
      std::transform((*point_it).distances_,
                     (*point_it).distances_ + Neighborhood::K_NEIGHBORS,
                     graph_[*it].distances_,
                     [](const Value x) -> Value { return std::sqrt(x); });
      // Invalidate computed edge costs to enforce recomputation.
      std::fill(graph_[*it].costs_,
                graph_[*it].costs_ + Neighborhood::K_NEIGHBORS,
                std::numeric_limits<Value>::quiet_NaN());
    }

    RCLCPP_DEBUG(map_logger(),
                 "Neighborhood updated at %lu / %lu pts (%.3f s).",
                 dirty_cloud.size(), cloud_.size(), t.seconds_elapsed());
  }

  template <typename It> void compute_edge_costs(It begin, It end) {
    Timer t;
    size_t n = 0;
    for (It it = begin; it != end; ++it) {
      ++n;
      const auto v0 = *it;
      Index e0 = v0 * Neighborhood::K_NEIGHBORS;
      for (Index i = 0, e = e0; i < Neighborhood::K_NEIGHBORS; ++i, ++e) {
        graph_[v0].costs_[i] = compute_edge_cost(e);
      }
    }
    RCLCPP_INFO(map_logger(), "Edge costs computed for %lu vertices (%.3f s).",
                n, t.seconds_elapsed());
  }

  // Boost graph interface, see naex/graph.h for the adapter.

  inline Vertex num_vertices() const { return Vertex(size()); }
  inline Edge num_edges() const {
    // TODO: Compute true number based on valid neighbors.
    return num_vertices() * Neighborhood::K_NEIGHBORS;
  }
  inline std::pair<VertexIter, VertexIter> vertices() const {
    return {VertexIter(0), VertexIter(num_vertices())};
  }
  inline std::pair<EdgeIter, EdgeIter> out_edges(const Vertex &u) const {
    // Skip the first neighbor - the vertex itself.
    return {EdgeIter(u * Neighborhood::K_NEIGHBORS + 1),
            EdgeIter((u + 1) * Neighborhood::K_NEIGHBORS)};
  }
  inline Edge out_degree(const Vertex &u) const {
    (void)u;
    // TODO: Compute true number based on valid neighbors.
    return Neighborhood::K_NEIGHBORS - 1;
  }
  inline Vertex source(const Edge &e) const {
    return e / Neighborhood::K_NEIGHBORS;
  }
  inline Vertex target_index(const Edge &e) const {
    return e % Neighborhood::K_NEIGHBORS;
  }
  inline Vertex target(const Edge &e) const {
    return graph_[source(e)].neighbors_[target_index(e)];
  }

  /// Update neighborhood, features, labels and edge costs of dirty points.
  void update_dirty();

  void clear_updated();

  void clear_dirty();

  template <typename It> void compute_features(It begin, It end) {
    Timer t;
    const auto semicircle_centroid_offset =
        Value(4.0 * neighborhood_radius_ / (3.0 * M_PI));
    Index n = 0;
    Index n_edge = 0;

    for (It it = begin; it != end; ++it, ++n) {
      const auto v0 = *it;

      // Disregard empty / dynamic points.
      if (!(cloud_[v0].flags_ & STATIC)) {
        continue;
      }

      // Estimate normals (local surface orientation).
      Vec3 mean = Vec3::Zero();
      Mat3 cov = Mat3::Zero();
      cloud_[v0].normal_support_ = 0;
      // First neighbor is the point itself.
      for (Index j = 0; j < Neighborhood::K_NEIGHBORS; ++j) {
        if (!valid_neighbor(graph_[v0].neighbors_[j],
                            graph_[v0].distances_[j])) {
          continue;
        }
        Index v1 = graph_[v0].neighbors_[j];

        // Disregard empty points.
        if (!(cloud_[v1].flags_ & STATIC)) {
          continue;
        }
        if (graph_[v0].distances_[j] <= clearance_radius_) {
          mean += ConstVec3Map(cloud_[v1].position_);
          Vec3 pc = (ConstVec3Map(cloud_[v1].position_) -
                     ConstVec3Map(cloud_[v0].position_));
          cov += pc * pc.transpose();
          ++cloud_[v0].normal_support_;
        }
      }
      mean /= cloud_[v0].normal_support_;
      cov /= cloud_[v0].normal_support_;
      Eigen::SelfAdjointEigenSolver<Mat3> solver(cov);
      Vec3Map normal(cloud_[v0].normal_);
      normal = solver.eigenvectors().col(0);

      // Compute other properties dependent on normal.
      cloud_[v0].ground_diff_std_ = std::sqrt(solver.eigenvalues()(0));

      // Inject support label here where we have mean at hand.
      const auto centroid_offset =
          (mean - ConstVec3Map(cloud_[v0].position_)).norm();
      if (centroid_offset >
          edge_min_centroid_offset_ * semicircle_centroid_offset) {
        cloud_[v0].flags_ |= EDGE;
        ++n_edge;
      } else {
        cloud_[v0].flags_ &= ~EDGE;
      }
    }
    RCLCPP_DEBUG(map_logger(), "%lu / %lu edge points (%.4f s).",
                 size_t(n_edge), size_t(n), t.seconds_elapsed());
  }

  template <typename It> void compute_labels(It begin, It end) {
    Timer t;
    Index n = 0;
    Index n_horizontal = 0;
    Index n_traversable = 0;
    Index n_empty = 0;
    Index n_actor = 0;

    const auto max_slope = std::max(max_pitch_, max_roll_);
    const auto min_z = std::cos(max_slope);

    for (It it = begin; it != end; ++it, ++n) {
      const auto v0 = *it;

      // Disregard empty points. Don't recompute anything for these.
      if (!(cloud_[v0].flags_ & STATIC)) {
        ++n_empty;
        continue;
      }

      // Clear flags we may set later.
      cloud_[v0].flags_ &= ~(HORIZONTAL | TRAVERSABLE);

      // Actor flag is temporary and orthogonal to others.
      if (cloud_[v0].flags_ & ACTOR) {
        ++n_actor;
      }

      // TODO: Check sign once normals keep consistent orientation.
      if (std::abs(cloud_[v0].normal_[2]) >= min_z) {
        // Approx. horizontal based on normal.
        cloud_[v0].flags_ |= HORIZONTAL;
        ++n_horizontal;
      }

      // Count edge neighbors (must run in the second pass).
      cloud_[v0].num_edge_neighbors_ = 0;
      cloud_[v0].num_obstacle_neighbors_ = 0;
      // Compute ground features (need second loop through neighbors).
      cloud_[v0].min_ground_diff_ = std::numeric_limits<Elem>::infinity();
      cloud_[v0].max_ground_diff_ = -std::numeric_limits<Elem>::infinity();
      cloud_[v0].mean_abs_ground_diff_ = 0.;
      // Compute clearance.
      cloud_[v0].dist_to_obstacle_ = std::numeric_limits<Value>::infinity();
      cloud_[v0].num_obstacle_pts_ = 0;
      Index n_cylinder_pts = 0;

      for (Vertex j = 0; j < Neighborhood::K_NEIGHBORS; ++j) {
        const auto v1 = graph_[v0].neighbors_[j];
        // Disregard invalid neighbors.
        if (!valid_neighbor(v1, graph_[v0].distances_[j])) {
          continue;
        }
        // Disregard distant neighbors.
        if (graph_[v0].distances_[j] > neighborhood_radius_) {
          continue;
        }

        // Disregard empty points.
        if (!(cloud_[v1].flags_ & STATIC)) {
          continue;
        }

        if (cloud_[v1].flags_ & EDGE) {
          ++cloud_[v0].num_edge_neighbors_;
        }

        if (!(cloud_[v1].flags_ & HORIZONTAL)) {
          ++cloud_[v0].num_obstacle_neighbors_;
        }

        // Avoid driving near obstacles.
        // TODO: Use clearance as hard constraint and distance to obstacles as
        // costs. Hard constraint can be removed with min_dist_to_obstacle_.
        if (!(cloud_[v1].flags_ & HORIZONTAL)) {
          if (graph_[v0].distances_[j] <= min_dist_to_obstacle_) {
            cloud_[v0].flags_ &= ~TRAVERSABLE;
          }
          cloud_[v0].dist_to_obstacle_ =
              std::min(graph_[v0].distances_[j], cloud_[v0].dist_to_obstacle_);
        }

        Vec3Map p0(cloud_[v0].position_);
        Vec3Map p1(cloud_[v1].position_);
        Vec3Map n0(cloud_[v0].normal_);

        Value height_diff = n0.dot(p1 - p0);
        Vec3 ground_pt = p1 - height_diff * n0;
        Value ground_dist = (ground_pt - p0).norm();

        if (ground_dist <= clearance_radius_) {
          cloud_[v0].min_ground_diff_ =
              std::min(height_diff, cloud_[v0].min_ground_diff_);
          cloud_[v0].max_ground_diff_ =
              std::max(height_diff, cloud_[v0].max_ground_diff_);
          cloud_[v0].mean_abs_ground_diff_ += std::abs(height_diff);
          ++n_cylinder_pts;
          if (height_diff >= clearance_low_ && height_diff <= clearance_high_) {
            ++cloud_[v0].num_obstacle_pts_;
          }
        }
      }
      cloud_[v0].mean_abs_ground_diff_ /= n_cylinder_pts;
      if ((cloud_[v0].flags_ & HORIZONTAL) &&
          cloud_[v0].num_obstacle_pts_ < min_points_obstacle_ &&
          cloud_[v0].min_ground_diff_ >= min_ground_diff_ &&
          cloud_[v0].max_ground_diff_ <= max_ground_diff_ &&
          cloud_[v0].ground_diff_std_ <= max_ground_diff_std_ &&
          cloud_[v0].mean_abs_ground_diff_ <= max_mean_abs_ground_diff_) {
        cloud_[v0].flags_ |= TRAVERSABLE;
        ++n_traversable;
      }
    }
    RCLCPP_DEBUG(map_logger(),
                 "%lu labels updated: %lu horizontal, %lu traversable, "
                 "%lu empty, %lu actor, (%.3f s).",
                 size_t(n), size_t(n_horizontal), size_t(n_traversable),
                 size_t(n_empty), size_t(n_actor), t.seconds_elapsed());
  }

  /** Resize point buffers if necessary, update wrappers and index. */
  void reserve(size_t n);

  std::vector<Index> nearby_indices(Value *origin, Value radius);

  /// Update occupancy from an organized cloud using its own neighborhood.
  void update_occupancy_organized(
      const sensor_msgs::msg::PointCloud2 &cloud,
      const geometry_msgs::msg::Transform &cloud_to_map_tf);

  /// Update occupancy from an organized cloud using a fitted sensor model.
  void update_occupancy_projection(
      const sensor_msgs::msg::PointCloud2 &cloud,
      const geometry_msgs::msg::Transform &cloud_to_map_tf);

  void update_occupancy_unorganized(const flann::Matrix<Elem> &points,
                                    const flann::Matrix<Elem> &origin);

  void initialize(const flann::Matrix<Elem> &points,
                  const flann::Matrix<Elem> &origin);

  void merge(const flann::Matrix<Elem> &points,
             const flann::Matrix<Elem> &origin);

  /// Describe the Point layout as point cloud fields.
  void initialize_cloud(sensor_msgs::msg::PointCloud2 &cloud) const;

  void create_cloud_msg(sensor_msgs::msg::PointCloud2 &cloud);

  template <typename C>
  void create_cloud_msg(const C &indices,
                        sensor_msgs::msg::PointCloud2 &cloud) {
    initialize_cloud(cloud);
    sensor_msgs::PointCloud2Modifier modifier(cloud);
    modifier.resize(indices.size());
    auto out = cloud.data.data();
    Lock cloud_lock(cloud_mutex_);
    for (auto it = indices.begin(); it != indices.end();
         ++it, out += cloud.point_step) {
      const auto from = reinterpret_cast<const uint8_t *>(&cloud_[*it]);
      std::copy(from, from + cloud.point_step, out);
    }
  }

  size_t capacity() const {
    Lock cloud_lock(cloud_mutex_);
    return cloud_.capacity();
  }

  size_t size() const {
    Lock cloud_lock(cloud_mutex_);
    return cloud_.size();
  }

  bool empty() const {
    Lock cloud_lock(cloud_mutex_);
    return cloud_.empty();
  }

  // Lock mutexes in this sequence
  // cloud_mutex_, index_mutex_, dirty_mutex_.
  // to avoid deadlocks.

  mutable Mutex cloud_mutex_;
  std::vector<Point> cloud_{};
  std::vector<Neighborhood> graph_{};

  mutable Mutex index_mutex_;
  std::shared_ptr<FlannIndex> index_;

  mutable Mutex updated_mutex_;
  std::vector<Index> updated_indices_{};

  mutable Mutex dirty_mutex_;
  std::unordered_set<Index> dirty_indices_{};

  // Steady clock used only for log throttling (the map has no node).
  mutable rclcpp::Clock clock_{RCL_STEADY_TIME};

  // Map parameters
  float points_min_dist_{0.2};
  // Occupancy
  float min_empty_cos_{0.259};
  int min_num_empty_{2};
  float min_empty_ratio_{1.0};
  int max_occ_counter_{7};

  // Graph
  float neighborhood_radius_{0.6};
  // Traversability
  float edge_min_centroid_offset_{0.5};
  float min_ground_diff_{-0.1};
  float max_ground_diff_{0.3};
  float clearance_low_{0.15};
  float clearance_high_{0.8};
  float clearance_radius_{neighborhood_radius_};
  float min_points_obstacle_{3};
  float max_ground_diff_std_{0.1};
  float max_mean_abs_ground_diff_{0.1};
  float min_dist_to_obstacle_{clearance_radius_};
  /// Max traversable slope, in radians; radians<double>() so that the value
  /// is the same double expression (30 / 180 * pi) it has always been.
  float max_pitch_{float(radians(30.))};
  float max_roll_{float(radians(30.))};
  float inclination_penalty_{1.0};
};

/**
 * Boost graph view of the map.
 *
 * https://www.boost.org/doc/libs/1_75_0/libs/graph/doc/adjacency_list.html
 */
class Graph {
public:
  explicit Graph(const Map &map) : map_(map) {}
  inline Vertex num_vertices() const { return map_.num_vertices(); }
  /** Returns the number of edges in the graph g. */
  inline Edge num_edges() const { return map_.num_edges(); }
  inline std::pair<VertexIter, VertexIter> vertices() const {
    return map_.vertices();
  }
  inline std::pair<EdgeIter, EdgeIter> out_edges(const Vertex &u) const {
    return map_.out_edges(u);
  }
  inline Edge out_degree(const Vertex &u) const { return map_.out_degree(u); }
  inline Vertex source(const Edge &e) const { return map_.source(e); }
  inline Vertex target_index(const Edge &e) const {
    return map_.target_index(e);
  }
  inline Vertex target(const Edge &e) const { return map_.target(e); }
  const Map &map_;
};

/// Readable property map of edge costs.
class EdgeCosts {
public:
  explicit EdgeCosts(const Map &map) : map_(map) {}
  inline Cost operator[](const Edge &e) const { return map_.edge_cost(e); }

private:
  const Map &map_;
};

} // namespace naex
