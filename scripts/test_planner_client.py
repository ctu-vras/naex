#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.srv import GetPlan
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
import time
import math


class TestPlannerClient(Node):
    def __init__(self):
        super().__init__('test_planner_client')
        
        # Declare parameters
        self.declare_parameter('start_x', -8.0)
        self.declare_parameter('start_y', 0.0)
        self.declare_parameter('start_yaw', 0.0)
        self.declare_parameter('goal_x', 8.0)
        self.declare_parameter('goal_y', 0.0)
        self.declare_parameter('goal_yaw', 0.0)
        self.declare_parameter('test_delay', 3.0)
        self.declare_parameter('obstacle_size', 6.0)
        self.declare_parameter('obstacle_center_x', 0.0)
        self.declare_parameter('obstacle_center_y', 0.0)

        # Get parameters
        self.start_x = self.get_parameter('start_x').get_parameter_value().double_value
        self.start_y = self.get_parameter('start_y').get_parameter_value().double_value
        self.start_yaw = self.get_parameter('start_yaw').get_parameter_value().double_value
        self.goal_x = self.get_parameter('goal_x').get_parameter_value().double_value
        self.goal_y = self.get_parameter('goal_y').get_parameter_value().double_value
        self.goal_yaw = self.get_parameter('goal_yaw').get_parameter_value().double_value
        self.test_delay = self.get_parameter('test_delay').get_parameter_value().double_value
        self.obstacle_size = self.get_parameter('obstacle_size').get_parameter_value().double_value
        self.obstacle_center_x = self.get_parameter('obstacle_center_x').get_parameter_value().double_value
        self.obstacle_center_y = self.get_parameter('obstacle_center_y').get_parameter_value().double_value
        
        # Create publisher for visualizing the planned path
        self.path_pub = self.create_publisher(Path, 'planned_path', 10)

        # Create service client
        self.get_plan_client = self.create_client(GetPlan, 'get_plan')
        
        # Wait for service and then run test
        self.timer = self.create_timer(1.0, self.check_service_and_test)
        self.test_executed = False
        
        self.get_logger().info('Test planner client initialized')
        self.get_logger().info(f'Start: ({self.start_x}, {self.start_y}, {self.start_yaw})')
        self.get_logger().info(f'Goal: ({self.goal_x}, {self.goal_y}, {self.goal_yaw})')
        self.get_logger().info(f'Waiting {self.test_delay}s before testing...')
        
    def check_service_and_test(self):
        """Check if service is available and run test if not already executed"""
        if not self.test_executed:
            if self.get_plan_client.service_is_ready():
                self.get_logger().info('GetPlan service is ready, starting test in a moment...')
                # Cancel the timer and start the test after delay
                self.timer.cancel()
                self.create_timer(self.test_delay, self.run_periodic_test)
            else:
                self.get_logger().info('Waiting for GetPlan service...')
                
    def create_pose_stamped(self, x, y, yaw=0.0):
        """Create a PoseStamped message"""
        pose = PoseStamped()
        pose.header.frame_id = 'local_utm'
        pose.header.stamp = self.get_clock().now().to_msg()
        
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = 0.0
        
        # Convert yaw to quaternion
        half_yaw = yaw / 2.0
        pose.pose.orientation.x = 0.0
        pose.pose.orientation.y = 0.0
        pose.pose.orientation.z = math.sin(half_yaw)
        pose.pose.orientation.w = math.cos(half_yaw)
        
        return pose

    def handle_plan_response(self, future):
        try:
            response = future.result()
            if response is not None:
                self.process_response(response)
            else:
                self.get_logger().error('Service call failed or timed out')
        except Exception as e:
            self.get_logger().error(f'Service call exception: {str(e)}')

    def process_response(self, response):
        """Process the GetPlan service response"""
        if len(response.plan.poses) == 0:
            self.get_logger().error('No path found!')
            return
            
        path_length = len(response.plan.poses)
        self.get_logger().info(f'Path found with {path_length} waypoints')
        
        # Log some waypoints
        if path_length > 0:
            start_pose = response.plan.poses[0]
            end_pose = response.plan.poses[-1]
            
            self.get_logger().info(f'Path start: ({start_pose.pose.position.x:.2f}, {start_pose.pose.position.y:.2f})')
            self.get_logger().info(f'Path end: ({end_pose.pose.position.x:.2f}, {end_pose.pose.position.y:.2f})')
            
        # Calculate path metrics
        total_distance = self.calculate_path_distance(response.plan)
        self.get_logger().info(f'Total path distance: {total_distance:.2f} meters')
        
        # Check if path avoids the obstacle (should not pass through center region)
        obstacle_avoidance = self.check_obstacle_avoidance(response.plan)
        if obstacle_avoidance:
            self.get_logger().info('Path successfully avoids obstacle')
        else:
            self.get_logger().warn('Path may pass through obstacle!')
            
        # Publish path for visualization
        self.path_pub.publish(response.plan)
        self.get_logger().info('Path published for RViz visualization')
        
    def run_periodic_test(self):
        """Run periodic tests with slight variations"""
        # Slightly vary the goal position to test different scenarios
        import random
        goal_x_var = self.goal_x + random.uniform(-2.0, 2.0)
        goal_y_var = self.goal_y + random.uniform(-5.0, 5.0)
        
        request = GetPlan.Request()
        request.start = self.create_pose_stamped(self.start_x, self.start_y, 0.0)
        request.goal = self.create_pose_stamped(goal_x_var, goal_y_var, 0.0)
        request.tolerance = 0.5
        
        self.get_logger().info(f'Testing path to varied goal: ({goal_x_var:.1f}, {goal_y_var:.1f})')
        
        try:
            future = self.get_plan_client.call_async(request)
            future.add_done_callback(self.handle_plan_response)
            
        except Exception as e:
            self.get_logger().error(f'Periodic test failed: {str(e)}')
            
    def calculate_path_distance(self, path):
        """Calculate total distance along the path"""
        total_distance = 0.0
        
        for i in range(1, len(path.poses)):
            prev_pose = path.poses[i-1].pose.position
            curr_pose = path.poses[i].pose.position
            
            dx = curr_pose.x - prev_pose.x
            dy = curr_pose.y - prev_pose.y
            distance = math.sqrt(dx*dx + dy*dy)
            total_distance += distance
            
        return total_distance
        
    def check_obstacle_avoidance(self, path):
        """Check if path avoids the central square obstacle area"""
        
        # Calculate square obstacle bounds
        obstacle_half_size = self.obstacle_size / 2.0
        obstacle_x_min = self.obstacle_center_x - obstacle_half_size
        obstacle_x_max = self.obstacle_center_x + obstacle_half_size
        obstacle_y_min = self.obstacle_center_y - obstacle_half_size
        obstacle_y_max = self.obstacle_center_y + obstacle_half_size
        
        for pose in path.poses:
            x = pose.pose.position.x
            y = pose.pose.position.y
            
            # Check if point is within square obstacle bounds
            if (obstacle_x_min <= x <= obstacle_x_max and 
                obstacle_y_min <= y <= obstacle_y_max):
                self.get_logger().warn(f'Path point ({x:.2f}, {y:.2f}) is inside square obstacle bounds')
                return False
                
        return True


def main(args=None):
    rclpy.init(args=args)
    node = TestPlannerClient()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()