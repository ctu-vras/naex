#include <naex/grid/planner.h>
#include <rclcpp/rclcpp.hpp>

#include <exception>
#include <memory>

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  auto node = rclcpp::Node::make_shared("grid_planner", rclcpp::NodeOptions());
  std::unique_ptr<naex::grid::Planner> planner;
  try {
    planner = std::make_unique<naex::grid::Planner>(node);
  } catch (const std::exception &ex) {
    RCLCPP_FATAL(node->get_logger(), "Could not initialize the planner: %s",
                 ex.what());
    rclcpp::shutdown();
    return 1;
  }

  rclcpp::spin(node);
  rclcpp::shutdown();

  return 0;
}
