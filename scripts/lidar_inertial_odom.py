#!/usr/bin/env python3
"""Lidar-inertial odometry node.

Note: the projective ICP is not implemented yet (`projective_icp` is a stub),
so the node is a work in progress, ported as-is from ROS 1.
"""
from threading import RLock

from geometry_msgs.msg import Pose, Transform, TransformStamped
from nav_msgs.msg import Odometry
import numpy as np
import rclpy
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from ros2_numpy import msgify, numpify
from sensor_msgs.msg import Imu, PointCloud2
import tf2_ros
# Optional dependency, kept from the original implementation; the learned
# correspondence weights are meant to be evaluated with it.
import torch  # noqa: F401


def cross_product_matrix(v, dtype=np.float32):
    assert len(v) == 3
    mat = np.array([[0., -v[2], v[1]],
                    [v[2], 0., -v[0]],
                    [-v[1], v[0], 0.]], dtype=dtype)
    return mat


def projective_icp(x, normal_x, y, normal_y, tf=np.eye(4)):
    pass


def preprocess_cloud(cloud, logger=None):
    x = np.stack([cloud[f] for f in ('x', 'y', 'z')])
    if logger is not None:
        logger.info('Cloud shape: %s' % (x.shape,))
    normal_x = np.cross(x[1:, :-1] - x[:-1, :-1], x[:-1, 1:] - x[:-1, :-1])
    normal_x /= np.linalg.norm(normal_x, axis=-1, keepdims=True)
    return x, normal_x


class LidarInertialOdom(Node):
    """Lidar-inertial odometry node.

    Subscribes IMU and PointCloud2 messages and publishes Odometry and TFMessage
    messages.

    IMU measurement (assumes valid orientation) is used to estimate initial
    cloud orientation, constant (previous) velocity gives the translation
    estimate. Projective point-to-plane ICP is used to align incoming cloud
    to a previous cloud, a keyframe, or a map, starting from the initial
    transform estimate.

    Basic principle is similar to [1]. The correspondence weights are learned
    instead of being given by Huber weighting.

    [1] Behley, J. & Stachniss, C.
        Efficient Surfel-Based SLAM using 3D Laser Range Data in Urban Environments
        Robotics: Science and System XIV, 2018
    """

    def __init__(self):
        super().__init__('lidar_inertial_odom')

        self.odom_frame = self.declare_parameter('odom_frame', 'odom').value
        self.robot_frame = self.declare_parameter('robot_frame', 'base_link').value
        self.imu_frame = self.declare_parameter('imu_frame', 'imu').value
        self.lidar_frame = self.declare_parameter('lidar_frame', 'lidar').value

        reliable = QoSProfile(depth=5, reliability=ReliabilityPolicy.RELIABLE)
        best_effort = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT)

        self.tf_pub = tf2_ros.TransformBroadcaster(self)
        self.tf = tf2_ros.Buffer(cache_time=Duration(seconds=5.0))
        self.tf_sub = tf2_ros.TransformListener(self.tf, self, spin_thread=True)

        self.imu_to_robot = None
        self.lidar_to_robot = None
        self.wait_for_transforms()

        self.feature_cloud_pub = self.create_publisher(PointCloud2, 'feature_cloud',
                                                       best_effort)
        self.odom_pub = self.create_publisher(Odometry, 'odom', reliable)

        self.imu_lock = RLock()
        self.imu_tf = tf2_ros.Buffer(cache_time=Duration(seconds=5.0))
        self.prev_imu_msg = None
        self.imu_sub = self.create_subscription(Imu, 'imu', self.imu_data_received,
                                                best_effort)

        self.cloud_lock = RLock()
        self.clouds = []
        self.prev_cloud_msg = None
        self.prev_x = None
        self.prev_normal_x = None
        self.prev_lidar_vel = np.zeros((3, 1))
        self.prev_robot_to_odom = np.eye(4)
        self.cloud_sub = self.create_subscription(PointCloud2, 'cloud',
                                                  self.cloud_received, best_effort)

    def wait_for_transforms(self):
        start = self.get_clock().now()
        last = Time()
        timeout = Duration(seconds=3.0)
        waiting = 0.0
        while True:
            try:
                self.imu_to_robot = self.tf.lookup_transform(
                    self.robot_frame, self.imu_frame, last, timeout)
            except tf2_ros.TransformException as ex:
                self.get_logger().info('Could not transform %s to %s: %s.'
                                       % (self.imu_frame, self.robot_frame, ex))
            try:
                self.lidar_to_robot = self.tf.lookup_transform(
                    self.robot_frame, self.lidar_frame, last, timeout)
            except tf2_ros.TransformException as ex:
                self.get_logger().info('Could not transform %s to %s: %s.'
                                       % (self.lidar_frame, self.robot_frame, ex))
            waiting = (self.get_clock().now() - start).nanoseconds / 1e9
            if self.imu_to_robot and self.lidar_to_robot:
                break
            if waiting > 3.:
                self.get_logger().warning('Waiting for transforms (%.1f s)...' % waiting)
        self.get_logger().info('Got sensor transforms (after %.1f s).' % waiting)

    def imu_data_received(self, msg):
        assert isinstance(msg, Imu)
        # TODO: Integrate angular velocities if orientation is not provided.
        assert msg.orientation_covariance[0] != -1.
        self.get_logger().info('IMU data received.')

        if (self.prev_imu_msg is not None
                and Time.from_msg(self.prev_imu_msg.header.stamp)
                >= Time.from_msg(msg.header.stamp)):
            self.get_logger().warning('Skipping old IMU message.')
        self.prev_imu_msg = msg

        # Insert IMU orientation in the buffer.
        tf = TransformStamped()
        tf.header.frame_id = 'parent'
        tf.header.stamp = msg.header.stamp
        tf.child_frame_id = msg.header.frame_id
        tf.transform.rotation = msg.orientation
        # We assume transformation between base_link, lidar, and imu are all static,
        # so that only relative orientation between IMU measurements can be used.
        self.imu_tf.set_transform(tf, 'lidar_inertial_odom')

    def cloud_received(self, msg):
        assert isinstance(msg, PointCloud2)
        self.get_logger().info('Cloud received with %i points.'
                               % (msg.height * msg.width))

        with self.imu_lock:
            imu_frame = self.prev_imu_msg.header.frame_id

        with self.cloud_lock:
            if (self.prev_cloud_msg is not None
                    and Time.from_msg(self.prev_cloud_msg.header.stamp)
                    >= Time.from_msg(msg.header.stamp)):
                self.get_logger().warning('Skipping old point cloud message.')

            # Preprocess incoming cloud - estimate normals using forward
            # differences.
            cloud = numpify(msg)
            x, normal_x = preprocess_cloud(cloud, self.get_logger())
            # TODO: Disregard invalid points and normals.
            cloud['nx'] = normal_x[..., 0]
            cloud['ny'] = normal_x[..., 1]
            cloud['nz'] = normal_x[..., 2]
            cloud_msg = msgify(PointCloud2, cloud)
            self.feature_cloud_pub.publish(cloud_msg)

            # TODO: Apply IMU orientation (and constant velocity) in robot frame.
            # Get relative IMU orientation.
            timeout = Duration(seconds=1.0)
            try:
                tf = self.imu_tf.lookup_transform_full(
                    imu_frame, Time.from_msg(self.prev_cloud_msg.header.stamp),
                    imu_frame, Time.from_msg(msg.header.stamp), 'parent', timeout)
            except tf2_ros.TransformException as ex:
                self.get_logger().warning(
                    'Could not get relative cloud orientation from IMU: %s.' % ex)
                return
            imu_step = numpify(tf.transform)

            # Compute predictive step from above and constant velocity for translation.
            dt = ((Time.from_msg(msg.header.stamp)
                   - Time.from_msg(self.prev_cloud_msg.header.stamp)).nanoseconds / 1e9)
            self.get_logger().info('Odometry time step: %.2f s' % dt)
            lidar_step_est = np.eye(4)
            lidar_step_est[:3, :3] = imu_step[:3, :3]
            lidar_step_est[:3, 3:] = dt * self.prev_lidar_vel

            # Iterate projective point-to-plane ICP and compute odometry.
            lidar_step = projective_icp(x, normal_x, self.prev_x, self.prev_normal_x,
                                        lidar_step_est)
            self.get_logger().info('Lidar step:\n%s' % (lidar_step,))
            robot_to_odom = self.prev_robot_to_odom.dot(self.lidar_to_robot).dot(lidar_step)
            self.get_logger().info('Odometry:\n%s' % (robot_to_odom,))

            # Publish tf and odometry messages.
            tf_msg = TransformStamped()
            tf_msg.header.frame_id = self.odom_frame
            tf_msg.header.stamp = msg.header.stamp
            tf_msg.child_frame_id = self.robot_frame
            tf_msg.transform = msgify(Transform, robot_to_odom)
            self.tf_pub.sendTransform(tf_msg)

            odom_msg = Odometry()
            odom_msg.header.frame_id = self.odom_frame
            odom_msg.header.stamp = msg.header.stamp
            odom_msg.child_frame_id = self.robot_frame
            odom_msg.pose.pose = msgify(Pose, robot_to_odom)
            self.odom_pub.publish(odom_msg)

            # Remember current estimates for next step.
            self.prev_lidar_vel = lidar_step[:3, 3:] / dt
            # TODO: Normalize transform time to time.
            self.prev_robot_to_odom = robot_to_odom


def main(args=None):
    rclpy.init(args=args)
    node = LidarInertialOdom()
    executor = MultiThreadedExecutor()
    try:
        rclpy.spin(node, executor=executor)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
