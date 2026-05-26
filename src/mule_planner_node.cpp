#include <naex/grid/mule_planner.h>
#include <rclcpp/rclcpp.hpp>

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  auto node = rclcpp::Node::make_shared("mule_planner", rclcpp::NodeOptions());
  naex::grid::MulePlanner planner(node);

  rclcpp::spin(node);
  rclcpp::shutdown();

  return 0;
}
