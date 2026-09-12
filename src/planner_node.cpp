#include <naex/planner.h>
#include <rclcpp/rclcpp.hpp>

#include <exception>
#include <memory>

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  auto node = rclcpp::Node::make_shared("planner", rclcpp::NodeOptions());
  std::unique_ptr<naex::Planner> planner;
  try {
    planner = std::make_unique<naex::Planner>(node);
  } catch (const std::exception &ex) {
    RCLCPP_FATAL(node->get_logger(), "Could not initialize the planner: %s",
                 ex.what());
    rclcpp::shutdown();
    return 1;
  }

  // Reproduce the ROS 1 MultiThreadedSpinner(8); the planner uses a reentrant
  // callback group and guards its data with its own mutexes.
  rclcpp::executors::MultiThreadedExecutor executor(
      rclcpp::ExecutorOptions(), 8);
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();

  return 0;
}
