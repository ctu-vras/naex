#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
import struct


class MockMapPublisher(Node):
    def __init__(self):
        super().__init__('mock_map_publisher')
        
        # Declare parameters
        self.declare_parameter('map_size', 20.0)
        self.declare_parameter('map_cost', 1.0)
        self.declare_parameter('obstacle_size', 6.0)
        self.declare_parameter('obstacle_center_x', 0.0)
        self.declare_parameter('obstacle_center_y', 0.0)
        self.declare_parameter('grid_resolution', 0.1)
        self.declare_parameter('obstacle_cost', 10.0)
        self.declare_parameter('publish_rate', 1.0)
        
        # Get parameters
        self.map_size = self.get_parameter('map_size').get_parameter_value().double_value
        self.map_cost = self.get_parameter('map_cost').get_parameter_value().double_value
        self.obstacle_size = self.get_parameter('obstacle_size').get_parameter_value().double_value
        self.obstacle_center_x = self.get_parameter('obstacle_center_x').get_parameter_value().double_value
        self.obstacle_center_y = self.get_parameter('obstacle_center_y').get_parameter_value().double_value
        self.grid_resolution = self.get_parameter('grid_resolution').get_parameter_value().double_value
        self.obstacle_cost = self.get_parameter('obstacle_cost').get_parameter_value().double_value
        self.publish_rate = self.get_parameter('publish_rate').get_parameter_value().double_value
        
        # Create publisher (single map publisher)
        self.osm_grid_pub = self.create_publisher(PointCloud2, 'osm_grid', 10)
        
        # Create timer
        self.timer = self.create_timer(1.0 / self.publish_rate, self.publish_map)
        
        self.get_logger().info(f'Mock map publisher initialized')
        self.get_logger().info(f'Map: {self.map_size}x{self.map_size}m with cost {self.map_cost}')
        self.get_logger().info(f'Obstacle: {self.obstacle_size}x{self.obstacle_size}m at ' +
                               f'({self.obstacle_center_x}, {self.obstacle_center_y}) ' + 
                               f'with cost {self.obstacle_cost}')
        
    def create_map_cloud(self, frame_id='local_utm'):
        """Create a point cloud representing the complete map with traversable area and obstacle"""
        
        # Calculate map bounds
        map_half_size = self.map_size / 2.0
        map_x_min = -map_half_size
        map_x_max = map_half_size
        map_y_min = -map_half_size
        map_y_max = map_half_size
        
        # Calculate obstacle bounds
        obstacle_half_size = self.obstacle_size / 2.0
        obstacle_x_min = self.obstacle_center_x - obstacle_half_size
        obstacle_x_max = self.obstacle_center_x + obstacle_half_size
        obstacle_y_min = self.obstacle_center_y - obstacle_half_size
        obstacle_y_max = self.obstacle_center_y + obstacle_half_size
        
        points = []
        
        # Generate grid points for the entire map
        num_points_x = int(self.map_size / self.grid_resolution) + 1
        num_points_y = int(self.map_size / self.grid_resolution) + 1
        
        for i in range(num_points_x):
            for j in range(num_points_y):
                x = map_x_min + i * self.grid_resolution
                y = map_y_min + j * self.grid_resolution
                z = 0.0
                
                # Check if point is within map bounds
                if (map_x_min <= x <= map_x_max and map_y_min <= y <= map_y_max):
                    # Check if point is inside obstacle
                    if (obstacle_x_min <= x <= obstacle_x_max and 
                        obstacle_y_min <= y <= obstacle_y_max):
                        # Point is in obstacle - use obstacle cost
                        points.append([x, y, z, self.obstacle_cost])
                    else:
                        # Point is in traversable area - use map cost
                        points.append([x, y, z, self.map_cost])
        
        if not points:
            self.get_logger().warn('No map points generated')
            return None
        
        # Define point cloud fields (always include cost field)
        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='cost', offset=12, datatype=PointField.FLOAT32, count=1),
        ]
        
        # Create point cloud message
        cloud_msg = PointCloud2()
        cloud_msg.header = Header()
        cloud_msg.header.frame_id = frame_id
        cloud_msg.header.stamp = self.get_clock().now().to_msg()
        
        cloud_msg.height = 1
        cloud_msg.width = len(points)
        cloud_msg.fields = fields
        cloud_msg.is_bigendian = False
        cloud_msg.point_step = 16
        cloud_msg.row_step = cloud_msg.point_step * cloud_msg.width
        
        # Pack point data (always include cost field)
        cloud_data = []
        for point in points:
            cloud_data.extend(struct.pack('ffff', point[0], point[1], point[2], point[3]))
                
        cloud_msg.data = bytes(cloud_data)
        cloud_msg.is_dense = True
        
        return cloud_msg
        
    def publish_map(self):
        """Publish complete map point cloud"""
        
        # Create and publish unified map with traversable area and obstacle
        map_cloud = self.create_map_cloud('local_utm')
        if map_cloud:
            self.osm_grid_pub.publish(map_cloud)
            
            # Count traversable vs obstacle points for logging
            traversable_points = sum(1 for i in range(0, len(map_cloud.data), 16) 
                                   if struct.unpack('f', map_cloud.data[i+12:i+16])[0] == self.map_cost)
            obstacle_points = map_cloud.width - traversable_points
            
            self.get_logger().info(
                f'Published map: {map_cloud.width} total points '
                f'({traversable_points} traversable, {obstacle_points} obstacle)'
            )
        else:
            self.get_logger().warn('Failed to create map cloud')


def main(args=None):
    rclpy.init(args=args)
    node = MockMapPublisher()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()