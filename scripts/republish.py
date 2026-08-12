#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.time import Time
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
import tf2_ros
from tf2_geometry_msgs import do_transform_pose


class PathToBaseLink(Node):
    def __init__(self):
        super().__init__('path_to_base_link')
        self.target_frame = 'base_link'

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.pub = self.create_publisher(Path, '/path_base_link', 10)
        self.sub = self.create_subscription(Path, '/path', self.path_cb, 10)

        self.get_logger().info('Subscribed to /path, publishing /path_base_link')

    def path_cb(self, msg: Path):
        if not msg.poses:
            self.pub.publish(msg)
            return

        try:
            # Single lookup for the whole path (source frame -> base_link)
            transform = self.tf_buffer.lookup_transform(
                self.target_frame,
                msg.header.frame_id,
                Time(),  # latest available transform
                timeout=Duration(seconds=0.5),
            )
        except Exception as e:
            self.get_logger().warn(f'TF lookup {msg.header.frame_id} -> {self.target_frame} failed: {e}')
            return

        out = Path()
        out.header.stamp = msg.header.stamp
        out.header.frame_id = self.target_frame

        for pose in msg.poses:
            ps = PoseStamped()
            ps.header.stamp = pose.header.stamp
            ps.header.frame_id = self.target_frame
            ps.pose = do_transform_pose(pose.pose, transform)
            out.poses.append(ps)

        self.pub.publish(out)


def main():
    rclpy.init()
    node = PathToBaseLink()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()