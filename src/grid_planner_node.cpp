#include <naex/grid/planner.h>
#include <rclcpp/rclcpp.hpp>

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  auto node = rclcpp::Node::make_shared("grid_planner", rclcpp::NodeOptions());
  naex::grid::Planner planner(node);

  rclcpp::spin(node);
  rclcpp::shutdown();

  return 0;
}
