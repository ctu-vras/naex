#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import tf2_ros
from geometry_msgs.msg import TransformStamped
import math


class MockTfPublisher(Node):
    def __init__(self):
        super().__init__('mock_tf_publisher')
        
        # Declare parameters
        self.declare_parameter('robot_x', -8.0)
        self.declare_parameter('robot_y', 0.0)
        self.declare_parameter('robot_z', 0.0)
        self.declare_parameter('robot_yaw', 0.0)
        
        # Get parameters
        self.robot_x = self.get_parameter('robot_x').get_parameter_value().double_value
        self.robot_y = self.get_parameter('robot_y').get_parameter_value().double_value
        self.robot_z = self.get_parameter('robot_z').get_parameter_value().double_value
        self.robot_yaw = math.radians(self.get_parameter('robot_yaw').get_parameter_value().double_value)
        
        # Create TF broadcaster
        self.tf_broadcaster = tf2_ros.StaticTransformBroadcaster(self)
        
        # Create and publish static transforms
        self.publish_static_transforms()
        
        self.get_logger().info(f'Mock TF publisher initialized')
        self.get_logger().info(f'Robot position: ({self.robot_x}, {self.robot_y}, {self.robot_z})')
        self.get_logger().info(f'Robot yaw: {self.robot_yaw} rad ({math.degrees(self.robot_yaw)} deg)')
        
    def publish_static_transforms(self):
        """Publish static transform from local_utm to base_link"""
        
        # Create transform from local_utm to base_link
        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = 'local_utm'
        transform.child_frame_id = 'base_link'
        
        # Set translation
        transform.transform.translation.x = self.robot_x
        transform.transform.translation.y = self.robot_y
        transform.transform.translation.z = self.robot_z
        
        # Set rotation (quaternion from yaw)
        # q = [0, 0, sin(yaw/2), cos(yaw/2)]
        half_yaw = self.robot_yaw / 2.0
        transform.transform.rotation.x = 0.0
        transform.transform.rotation.y = 0.0
        transform.transform.rotation.z = math.sin(half_yaw)
        transform.transform.rotation.w = math.cos(half_yaw)
        
        # Publish the transform
        transforms = [transform]
        self.tf_broadcaster.sendTransform(transforms)
        
        self.get_logger().info('Published static transform: local_utm -> base_link')


def main(args=None):
    rclpy.init(args=args)
    node = MockTfPublisher()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()