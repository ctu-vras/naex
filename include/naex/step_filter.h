#pragma once

#include <naex/filter.h>
#include <naex/timer.h>
#include <algorithm>
#include <cassert>
#include <cstdint>
#include <limits>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

namespace naex {

inline void step_subsample(const sensor_msgs::msg::PointCloud2 &input,
                           const uint32_t max_rows, const uint32_t max_cols,
                           sensor_msgs::msg::PointCloud2 &output) {
  Timer t;
  output.header = input.header;
  uint32_t row_step = (input.height - 1) / max_rows + 1;
  uint32_t col_step = (input.width - 1) / max_cols + 1;
  output.height = input.height / row_step;
  output.width = input.width / col_step;
  output.fields = input.fields;
  output.is_bigendian = input.is_bigendian;
  output.point_step = input.point_step;
  output.row_step = output.width * input.point_step;
  output.is_dense = input.is_dense;

  output.data.resize(size_t(output.height) * output.width * output.point_step);
  const uint8_t *in_ptr = input.data.data();
  uint8_t *out_ptr = output.data.data();
  for (uint32_t r = 0; r < output.height; ++r) {
    for (uint32_t c = 0; c < output.width; ++c) {
      std::copy(in_ptr + (r * row_step * input.row_step +
                          (c * col_step) * input.point_step),
                in_ptr + (r * row_step * input.row_step +
                          (c * col_step + 1) * input.point_step),
                out_ptr + (r * output.row_step + c * output.point_step));
    }
  }
  RCLCPP_DEBUG(rclcpp::get_logger("naex.filter"),
               "Step-subsample %u-by-%u cloud to %u-by-%u (%.6f s).",
               input.height, input.width, output.height, output.width,
               t.seconds_elapsed());
}

/// Subsample an organized cloud to at most max_rows x max_cols.
class StepFilter : public Filter<sensor_msgs::msg::PointCloud2> {
public:
  StepFilter(uint32_t max_rows, uint32_t max_cols)
      : Filter<sensor_msgs::msg::PointCloud2>(), max_rows_(max_rows),
        max_cols_(max_cols) {
    assert(max_rows > 0);
    assert(max_cols > 0);
  }
  virtual ~StepFilter() = default;

  void filter(const sensor_msgs::msg::PointCloud2 &input,
              sensor_msgs::msg::PointCloud2 &output) override {
    step_subsample(input, max_rows_, max_cols_, output);
  }

protected:
  uint32_t max_rows_{std::numeric_limits<uint32_t>::max()};
  uint32_t max_cols_{std::numeric_limits<uint32_t>::max()};
};

} // namespace naex
