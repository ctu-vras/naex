#!/usr/bin/env python3
"""Request a plan from the planner.

Calls the nav_msgs/GetPlan service `get_plan`. Both start and goal positions
may be NaN (the default): a NaN start uses the robot's current position, and
a NaN goal invokes the exploration / go-home behaviour, but only with the
legacy point-map `planner` node -- the `grid_planner` rejects a NaN goal.
"""

from geometry_msgs.msg import PoseStamped
from nav_msgs.srv import GetPlan
from rcl_interfaces.msg import ParameterDescriptor
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

NAN = float("nan")


def as_position(value):
    """Convert a parameter value into a list of three floats."""
    if isinstance(value, str):
        value = [float(x) for x in value.split(",")]
    else:
        value = [float(x) for x in value]
    assert len(value) == 3
    return value


class GetPlanClient(Node):
    def __init__(self):
        super().__init__("get_plan")

        # Both a string ('x,y,z') and a double array are accepted, as in ROS 1.
        dynamic = ParameterDescriptor(dynamic_typing=True)
        self.start_pos = as_position(
            self.declare_parameter("start", [NAN, NAN, NAN], dynamic).value
        )
        self.get_logger().info("Start: [%.2f, %.2f, %.2f]" % tuple(self.start_pos))

        self.goal_pos = as_position(
            self.declare_parameter("goal", [NAN, NAN, NAN], dynamic).value
        )
        self.get_logger().info("Goal: [%.2f, %.2f, %.2f]" % tuple(self.goal_pos))

        self.tolerance = self.declare_parameter("tolerance", 32.0).value
        self.verbose = self.declare_parameter("verbose", False).value

        self.client = self.create_client(GetPlan, "get_plan")

    def request_plan(self):
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Waiting for service get_plan...")

        req = GetPlan.Request()
        start = PoseStamped()
        (start.pose.position.x, start.pose.position.y, start.pose.position.z) = (
            self.start_pos
        )
        goal = PoseStamped()
        (goal.pose.position.x, goal.pose.position.y, goal.pose.position.z) = (
            self.goal_pos
        )
        req.start = start
        req.goal = goal
        req.tolerance = float(self.tolerance)

        future = self.client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        res = future.result()
        if res is None:
            self.get_logger().error("Service call failed: %s" % future.exception())
            return

        if self.verbose:
            self.get_logger().info(str(res.plan))
        else:
            positions = [
                "[%.2f, %.2f, %.2f]"
                % (p.pose.position.x, p.pose.position.y, p.pose.position.z)
                for p in res.plan.poses
            ]
            self.get_logger().info("\n".join(positions))


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = GetPlanClient()
        node.request_plan()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
