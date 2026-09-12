#include "naex/spherical_projection.h"

#include "naex/geom.h"
#include "naex/timer.h"
#include "naex/types.h"
#include <algorithm>
#include <cassert>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <random>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <sstream>
#include <utility>
#include <vector>

namespace naex {

bool SphericalProjection::check(const sensor_msgs::msg::PointCloud2 &cloud) {
  assert(height_ == cloud.height);
  assert(width_ == cloud.width);
  if (cloud.height != height_ || cloud.width != width_) {
    RCLCPP_WARN(rclcpp::get_logger("naex"),
                "Cloud size (%i, %i) inconsistent with model size (%i, %i).",
                cloud.height, cloud.width, height_, width_);
  }
  sensor_msgs::PointCloud2ConstIterator<float> x_it(cloud, "x");
  double residual_sum = 0.;
  Index n = 0;

  for (uint32_t r = 0; r < height_; ++r) {
    for (uint32_t c = 0; c < width_; ++c, ++x_it) {
      if (!std::isfinite(x_it[0]) || !std::isfinite(x_it[1]) ||
          !std::isfinite(x_it[2]))
        continue;
      Value r_model, c_model;
      project(x_it[0], x_it[1], x_it[2], r_model, c_model);

      if (r == std::round(r_model) && c == std::round(c_model)) {
        continue;
      }

      Vec3 pt(x_it[0], x_it[1], x_it[2]);
      pt.normalize();
      Vec3 pt_model(0., 0., 0.);
      unproject(Value(r), Value(c), pt_model(0), pt_model(1), pt_model(2));
      Value residual = std::acos(pt.dot(pt_model));
      if (std::isfinite(residual)) {
        residual_sum += residual;
        ++n;
      }
    }
  }
  if (n == 0) {
    RCLCPP_WARN(rclcpp::get_logger("naex"),
                "No valid points to check the model against.");
    return false;
  }
  double mean_residual = residual_sum / n;
  if (mean_residual >
      std::min(std::abs(azimuth_step_), std::abs(elevation_step_)) / 2.) {
    RCLCPP_WARN(rclcpp::get_logger("naex"), "Mean angular error: %.3f [deg].",
                degrees(mean_residual));
  } else {
    RCLCPP_DEBUG(rclcpp::get_logger("naex"), "Mean angular error: %.3f [deg].",
                 degrees(mean_residual));
  }
  return true;
}

void SphericalProjection::print_model_summary() {
  std::stringstream az_ss, el_ss;
  const uint32_t r_step = std::max<uint32_t>(1, height_ / 8);
  const uint32_t c_step = std::max<uint32_t>(1, width_ / 8);
  for (uint32_t r = 0; r < height_; r += r_step) {
    if (r > 0) {
      az_ss << std::endl;
      el_ss << std::endl;
    }
    for (uint32_t c = 0; c < width_; c += c_step) {
      Vec3 pt_model(0.f, 0.f, 0.f);
      unproject(Value(r), Value(c), pt_model(0), pt_model(1), pt_model(2));
      Value az, el, radius;
      cartesian_to_spherical(pt_model(0), pt_model(1), pt_model(2), az, el,
                             radius);
      if (c > 0) {
        az_ss << " ";
        el_ss << " ";
      }
      az_ss << degrees(az);
      el_ss << degrees(el);
    }
  }
  RCLCPP_INFO(rclcpp::get_logger("naex"), "Azimuth model sample:\n%s",
              az_ss.str().c_str());
  RCLCPP_INFO(rclcpp::get_logger("naex"), "Elevation model sample:\n%s",
              el_ss.str().c_str());
}

bool SphericalProjection::fit_robust(
    const sensor_msgs::msg::PointCloud2 &cloud) {
  Timer t;
  assert(cloud.height >= 1);
  assert(cloud.width >= 1);

  const Index n_points = cloud.height * cloud.width;
  sensor_msgs::PointCloud2ConstIterator<float> x_begin(cloud, "x");

  // Collect valid indices.
  std::vector<Index> valid;
  valid.reserve(n_points);
  auto x = x_begin;
  for (Index i = 0; i < n_points; ++i, ++x) {
    if (!std::isfinite(x[0]) || !std::isfinite(x[1]) || !std::isfinite(x[2]))
      continue;
    valid.push_back(i);
  }

  // Shuffle valid indices.
  std::random_device rd;
  std::mt19937 g(rd());
  std::shuffle(valid.begin(), valid.end(), g);

  // Generate models from pairs of points.
  // Model consists of (start, step).
  typedef std::pair<Value, Value> Model;
  const auto comp = [](const Model &a, const Model &b) {
    return a.second < b.second;
  };
  std::vector<Model> az_models, el_models;
  az_models.reserve(n_points);
  el_models.reserve(n_points);
  for (size_t i = 0; i + 1 < valid.size(); ++i) {
    auto x0 = x_begin + valid[i];
    auto x1 = x_begin + valid[i + 1];

    Index c0 = valid[i] % cloud.width;
    Index c1 = valid[i + 1] % cloud.width;
    if (c0 != c1) {
      Value az0 = azimuth(x0[0], x0[1]);
      Value az1 = azimuth(x1[0], x1[1]);
      auto az_step = (az1 - az0) / (int(c1) - int(c0));
      az_models.push_back({az0 - c0 * az_step, az_step});
    }

    Index r0 = valid[i] / cloud.width;
    Index r1 = valid[i + 1] / cloud.width;
    if (r0 != r1) {
      Value el0 = elevation(x0[0], x0[1], x0[2]);
      Value el1 = elevation(x1[0], x1[1], x1[2]);
      auto el_step = (el1 - el0) / (int(r1) - int(r0));
      el_models.push_back({el0 - r0 * el_step, el_step});
    }

    if (az_models.size() >= 25 && el_models.size() >= 25) {
      break;
    }
  }

  if (az_models.empty() || el_models.empty()) {
    return false;
  }

  // Get median step models.
  std::sort(az_models.begin(), az_models.end(), comp);
  azimuth_start_ = az_models[az_models.size() / 2].first;
  azimuth_step_ = az_models[az_models.size() / 2].second;

  std::sort(el_models.begin(), el_models.end(), comp);
  elevation_start_ = el_models[el_models.size() / 2].first;
  elevation_step_ = el_models[el_models.size() / 2].second;

  height_ = cloud.height;
  width_ = cloud.width;

  RCLCPP_DEBUG(rclcpp::get_logger("naex"),
               "Robust fit [deg]: "
               "azimuth [%.1f, %.1f], step %.3f (from %lu models), "
               "elevation [%.1f, %.1f], step %.3f (from %lu models) "
               "(%.6f s).",
               degrees(azimuth_start_),
               degrees(azimuth_start_ + (width_ - 1) * azimuth_step_),
               degrees(azimuth_step_), az_models.size(),
               degrees(elevation_start_),
               degrees(elevation_start_ + (height_ - 1) * elevation_step_),
               degrees(elevation_step_), el_models.size(), t.seconds_elapsed());

  return true;
}

bool SphericalProjection::fit(const sensor_msgs::msg::PointCloud2 &cloud) {
  return fit_robust(cloud);
}

} // namespace naex
