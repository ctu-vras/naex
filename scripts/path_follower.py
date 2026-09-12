#!/usr/bin/env python3
"""Simple path follower.

It always acts on the last received plan which can be hold onto for a
specified duration. An empty plan means no action (stopping the robot).

The main control loop is (almost) stateless:
(1) The closest point on the path is found.
(2) A look-ahead point at a given distance is used as current goal.
(3) Bounded proportional control navigates robot towards the goal.

It supports 2D and 3D modes.
In 2D mode, the path and all poses are converted into 2D, meaning that z =
roll = pitch = 0 for (1)-(3).

Point cloud inputs can be used to check for needed clearance.
"""

from threading import RLock
from timeit import default_timer as timer
import traceback

from geometry_msgs.msg import (
    Point,
    Pose,
    PoseStamped,
    Transform,
    TransformStamped,
    Twist,
    Vector3,
)
from nav_msgs.msg import Path
import numpy as np
import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from ros2_numpy import msgify, numpify
from scipy.spatial import cKDTree
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import ColorRGBA
from tf_transformations import euler_from_matrix, euler_matrix
import tf2_ros
from rclpy.time import Time
from visualization_msgs.msg import Marker, MarkerArray

np.set_printoptions(precision=2)


def slots(msg):
    """Return message fields as list."""
    return [getattr(msg, var) for var in msg.get_fields_and_field_types()]


def tf_to_pose(tf):
    pose = Pose()
    pose.position.x = tf.translation.x
    pose.position.y = tf.translation.y
    pose.position.z = tf.translation.z
    pose.orientation = tf.rotation
    return pose


def tf_to_pose_stamped(tf):
    tf = TransformStamped()
    pose = PoseStamped()
    pose.header = tf.header
    pose.pose = tf_to_pose(tf.transform)
    return pose


def p2e(xh):
    x = xh[:-1, :]
    return x


def e2p(x):
    xh = np.concatenate((x, np.ones((1, x.shape[1]))))
    return xh


def points_in_2d(x, in_place=False):
    assert x.shape[0] == 3
    if not in_place:
        x = np.array(x, copy=True)
    x[2, :] = 0
    return x


def pose_in_2d(pose, in_place=False):
    assert pose.shape == (4, 4)
    if not in_place:
        pose = np.array(pose, copy=True)
    r, p, y = euler_from_matrix(pose[:3, :3])
    pose[:3, :3] = euler_matrix(0.0, 0.0, y)[:3, :3]
    pose[2, 3] = 0
    return pose


def box_param(values, name):
    """Convert a flat 6-element parameter into a 3-by-2 box array.

    ROS 2 parameters cannot be nested, so the ROS 1 [[xmin, xmax], [ymin, ymax],
    [zmin, zmax]] boxes are given as [xmin, xmax, ymin, ymax, zmin, zmax].
    """
    values = np.array(values, dtype=float)
    assert values.size == 6, "%s must have 6 elements" % name
    return values.reshape((3, 2))


class PathFollower(Node):
    def __init__(self):
        super().__init__("path_follower")

        self.map_frame = self.declare_parameter("map_frame", "map").value
        # No-wait frame
        self.odom_frame = self.declare_parameter("odom_frame", "odom").value
        self.robot_frame = self.declare_parameter("robot_frame", "base_footprint").value
        self.control_freq = self.declare_parameter("control_freq", 10.0).value  # Hz
        assert 1.0 < self.control_freq < 25.0
        self.local_goal_dims = self.declare_parameter("local_goal_dims", "xy").value
        assert self.local_goal_dims in ("xy", "xyz")
        self.goal_reached_dist = self.declare_parameter(
            "goal_reached_dist", 0.2
        ).value  # m
        self.goal_reached_angle = self.declare_parameter(
            "goal_reached_angle", 0.2
        ).value  # rad
        self.use_path_theta = self.declare_parameter("use_path_theta", "last").value
        assert self.use_path_theta in ("none", "last", "all")
        self.max_age = self.declare_parameter("max_age", 1.0).value  # s
        # Max. path distances, tail is consumed first by reached goals.
        self.max_path_dists = list(
            self.declare_parameter("max_path_dist", [0.5]).value
        )  # m
        assert len(self.max_path_dists) > 0
        self.max_path_dist = self.max_path_dists.pop()
        self.look_ahead = self.declare_parameter("look_ahead", 1.0).value  # m
        self.p_angle = self.declare_parameter("p_angle", 1.0).value
        self.p_dist = self.declare_parameter("p_dist", 1.0).value
        self.max_speed = self.declare_parameter("max_speed", 1.0).value  # m/s
        self.max_accel = self.declare_parameter("max_accel", 1.0).value  # m/s^2
        self.max_force_through_speed = self.declare_parameter(
            "max_force_through_speed", 0.25
        ).value
        self.turn_on_spot_angle = self.declare_parameter(
            "turn_on_spot_angle", np.pi / 6
        ).value
        self.max_angular_rate = self.declare_parameter(
            "max_angular_rate", 1.0
        ).value  # rad/s
        self.max_angular_accel = self.declare_parameter(
            "max_angular_accel", 2.0
        ).value  # rad/s^2
        self.max_roll = self.declare_parameter("max_roll", 0.7).value  # rad
        self.max_pitch = self.declare_parameter("max_pitch", 0.7).value  # rad
        self.keep_path = self.declare_parameter("keep_path", 30.0).value  # s
        self.increasing_waypoint_index = self.declare_parameter(
            "increasing_waypoint_index", True
        ).value
        self.estimate_path_costs = self.declare_parameter(
            "estimate_path_costs", False
        ).value
        # Keep only points inside a box for clearance check.
        self.keep_cloud_box = box_param(
            self.declare_parameter(
                "keep_cloud_box", [-4.0, 4.0, -4.0, 4.0, -4.0, 4.0]
            ).value,
            "keep_cloud_box",
        )
        self.clearance_box = box_param(
            self.declare_parameter(
                "clearance_box", [-0.6, 0.6, -0.5, 0.5, 0.0, 0.8]
            ).value,
            "clearance_box",
        )
        self.show_clearance = list(
            self.declare_parameter("show_clearance_pos", [-10, 10]).value
        )
        self.min_points_obstacle = self.declare_parameter(
            "min_points_obstacle", 1
        ).value
        self.force_through_after = self.declare_parameter(
            "force_through_after", 15.0
        ).value
        self.allow_backward = self.declare_parameter("allow_backward", True).value
        self.backtrack_after = self.declare_parameter("backtrack_after", 30.0).value

        self.path_lock = RLock()
        self.path_msg = None  # Path message
        self.path_received_time = None
        self.path_costs = None
        self.next_path_msg = None  # Subsequent path to be used after reaching goal.
        self.path = None  # n-by-3 path position array
        self.path_x_index = None  # Index of above
        # Waypoint index (into the path)
        self.waypoint_index = None
        self.path_traversed = []
        self.stuck_since = None
        self.idle_since = None
        # Acceleration limitation
        self.prev_time = None
        self.prev_speed = None
        self.prev_angular_rate = None

        self.cloud_lock = RLock()
        self.cloud_msg = None
        self.cloud = None  # n-by-3 cloud position array

        # Everything may block on TF, so run the callbacks concurrently
        # (rospy.Timer and subscriber callbacks ran in separate threads too).
        self.callback_group = ReentrantCallbackGroup()

        reliable = QoSProfile(depth=2, reliability=ReliabilityPolicy.RELIABLE)
        best_effort = QoSProfile(depth=2, reliability=ReliabilityPolicy.BEST_EFFORT)

        self.cmd_pub = self.create_publisher(Twist, "cmd_vel", reliable)

        self.tf = tf2_ros.Buffer()
        self.tf_sub = tf2_ros.TransformListener(self.tf, self, spin_thread=True)

        self.path_pub = self.create_publisher(Path, "control_path", reliable)
        self.markers_pub = self.create_publisher(MarkerArray, "~/markers", reliable)
        self.path_sub = self.create_subscription(
            Path,
            "path",
            self.path_received,
            reliable,
            callback_group=self.callback_group,
        )
        self.cloud_sub = self.create_subscription(
            PointCloud2,
            "cloud",
            self.cloud_received,
            best_effort,
            callback_group=self.callback_group,
        )
        self.timer = self.create_timer(
            1.0 / self.control_freq,
            self.control_safe,
            callback_group=self.callback_group,
        )

    def now(self):
        return self.get_clock().now()

    def age(self, stamp):
        """Age of a message stamp in seconds."""
        return (self.now() - Time.from_msg(stamp)).nanoseconds / 1e9

    def lookup_transform(
        self, target_frame, source_frame, time, no_wait_frame=None, timeout=0.0
    ):
        timeout = Duration(seconds=timeout)
        if no_wait_frame is None or no_wait_frame == target_frame:
            tf_s2t = self.tf.lookup_transform(
                target_frame, source_frame, time, timeout=timeout
            )
            return tf_s2t

        # Try to get exact transform from no-wait frame to target if available.
        # If not, use most recent transform.
        dont_wait = Duration(seconds=0.0)
        try:
            tf_n2t = self.tf.lookup_transform(
                target_frame, self.odom_frame, time, timeout=dont_wait
            )
        except tf2_ros.TransformException:
            tf_n2t = self.tf.lookup_transform(target_frame, self.odom_frame, Time())

        # Get the exact transform from source to no-wait frame.
        tf_s2n = self.tf.lookup_transform(
            self.odom_frame, source_frame, time, timeout=timeout
        )

        tf_s2t = TransformStamped()
        tf_s2t.header.frame_id = target_frame
        tf_s2t.header.stamp = time.to_msg() if isinstance(time, Time) else time
        tf_s2t.child_frame_id = source_frame
        tf_s2t.transform = msgify(
            Transform, np.matmul(numpify(tf_n2t.transform), numpify(tf_s2n.transform))
        )
        return tf_s2t

    def get_robot_pose(self, target_frame):
        tf = self.lookup_transform(
            target_frame,
            self.robot_frame,
            self.now(),
            timeout=0.5,
            no_wait_frame=self.odom_frame,
        )
        pose = tf_to_pose(tf.transform)
        return pose

    def clear_path(self):
        with self.path_lock:
            self.path_msg = None
            self.path_received_time = None
            self.path_costs = None
            self.path = None
            self.path_x_index = None
            self.waypoint_index = None

    def path_received(self, msg):
        assert isinstance(msg, Path)

        if not msg.header.frame_id:
            self.get_logger().warning(
                "Map frame %s will be used instead of empty path frame."
                % self.map_frame,
                once=True,
            )
            msg.header.frame_id = self.map_frame
        elif self.map_frame and msg.header.frame_id != self.map_frame:
            self.get_logger().warning(
                "Map frame %s will be used instead of path frame %s."
                % (self.map_frame, msg.header.frame_id),
                once=True,
            )

        # Discard old messages.
        age = self.age(msg.header.stamp)
        if age > self.max_age:
            self.get_logger().warning(
                "Discarding path %.1f s > %.1f s old." % (age, self.max_age)
            )
            return

        # Allow to stop the controller with an empty path.
        if not msg.poses:
            self.clear_path()
            self.get_logger().info("Path cleared.")
            return

        # Keep a recent path if keep_path is positive.
        with self.path_lock:
            if self.path_msg and self.keep_path > 0.0:
                age = self.age(self.path_msg.header.stamp)
                if age <= self.keep_path and self.stuck_since is None:
                    self.get_logger().info(
                        "Keeping previous path (%.1f s <= %.1f s)."
                        % (age, self.keep_path)
                    )
                    # Store as a subsequent path for later use.
                    goal = numpify(self.path_msg.poses[-1].pose.position)
                    start = numpify(msg.poses[0].pose.position)
                    if np.linalg.norm(goal - start) < 0.1:
                        self.get_logger().info("Subsequent path stored for later use.")
                        self.next_path_msg = msg
                    return

        # Update path and index.
        path = np.array([slots(p.pose.position) for p in msg.poses])
        if self.local_goal_dims == "xy":
            path = points_in_2d(path.T).T
        path_x_index = cKDTree(path)
        with self.path_lock:
            self.clear_path()
            self.path_msg = msg
            self.path_received_time = self.now()
            if self.estimate_path_costs:
                self.path_costs = self.compute_path_costs(self.path_msg.poses)
            self.next_path_msg = None
            self.path = path
            self.path_x_index = path_x_index
            self.path_pub.publish(msg)
            self.get_logger().info("Path received (%i poses)." % len(msg.poses))

    def cloud_received(self, msg):
        assert isinstance(msg, PointCloud2)

        age = self.age(msg.header.stamp)
        if age > self.max_age:
            self.get_logger().warning(
                "Discarding cloud %.1f s > %.1f s old." % (age, self.max_age)
            )
            return

        path_frame = self.map_frame

        t = timer()
        cloud = numpify(msg).ravel()
        cloud = np.stack([cloud[f] for f in ("x", "y", "z")])
        keep = (self.keep_cloud_box[:, :1] <= cloud).all(axis=0) & (
            cloud <= self.keep_cloud_box[:, 1:]
        ).all(axis=0)
        cloud = cloud[:, keep]
        if cloud.size == 0:
            self.get_logger().info("No points left.")
            return

        try:
            tf = self.lookup_transform(
                path_frame,
                msg.header.frame_id,
                Time.from_msg(msg.header.stamp),
                no_wait_frame=self.odom_frame,
                timeout=0.5,
            )
        except tf2_ros.TransformException:
            self.get_logger().error(
                "Could not transform cloud from %s to path frame %s at %.1f s."
                % (
                    msg.header.frame_id,
                    path_frame,
                    Time.from_msg(msg.header.stamp).nanoseconds / 1e9,
                )
            )
            return
        tf = numpify(tf.transform)
        cloud = np.matmul(tf, e2p(cloud))
        t = timer() - t

        with self.cloud_lock:
            self.cloud_msg = msg
            self.cloud = cloud
            self.get_logger().debug(
                "Cloud with %i points received, %i points kept (%.3f s)."
                % (msg.height * msg.width, cloud.shape[1], t)
            )

    def check_pose_clearance(self, pose):
        if self.min_points_obstacle < 1:
            return True, None
        # Convert cloud into given pose with odom/no-wait as fixed frame.
        # Check robot extents.
        with self.cloud_lock:
            cloud = self.cloud
        if cloud is None:
            self.get_logger().info("No cloud to check for obstacles.")
            return True, None
        tf = numpify(pose)
        tf[:3, :3] = tf[:3, :3].T
        tf[:3, 3:] = -np.matmul(tf[:3, :3], tf[:3, 3:])
        local_cloud = np.matmul(tf, self.cloud)
        obstacles = (local_cloud[:3, :] >= self.clearance_box[:, :1]).all(axis=0) & (
            local_cloud[:3, :] <= self.clearance_box[:, 1:]
        ).all(axis=0)
        n_obstacles = obstacles.sum()
        free = n_obstacles < self.min_points_obstacle
        pts = local_cloud[:3, obstacles] if n_obstacles else None
        return free, pts

    def path_markers(self, stamp, path_msg):
        marker = Marker()
        marker.header.frame_id = path_msg.header.frame_id
        marker.header.stamp = stamp
        marker.ns = "%s/path" % self.robot_frame
        marker.id = 0
        marker.action = Marker.MODIFY
        marker.type = Marker.LINE_STRIP
        marker.scale = Vector3(x=0.1, y=0.1, z=0.1)
        marker.color = ColorRGBA(r=0.0, g=1.0, b=0.0, a=0.5)
        marker.pose.orientation.w = 1.0

        for pose in path_msg.poses:
            marker.points.append(pose.pose.position)
            marker.colors.append(marker.color)

        return [marker]

    def clearance_markers(self, stamp, path_msg, indices):
        markers = []

        # Clean up previous clearance.
        delete_all = Marker()
        delete_all.ns = "%s/clearance" % self.robot_frame
        delete_all.action = Marker.DELETEALL
        markers.append(delete_all)

        # Marker with obstacle points.
        pts_marker = Marker()
        pts_marker.header.frame_id = path_msg.header.frame_id
        pts_marker.header.stamp = stamp
        pts_marker.ns = "%s/obstacles" % self.robot_frame
        pts_marker.id = 0
        pts_marker.action = Marker.MODIFY
        pts_marker.type = Marker.SPHERE_LIST
        pts_marker.scale = Vector3(x=0.05, y=0.05, z=0.05)
        pts_marker.color = ColorRGBA(r=1.0, g=0.0, b=0.0, a=0.5)
        pts_marker.pose.orientation.w = 1.0

        for i in indices:
            pose = path_msg.poses[i]
            # Pose clearance marker.
            marker = Marker()
            marker.header.frame_id = path_msg.header.frame_id
            marker.header.stamp = stamp
            marker.ns = "%s/clearance" % self.robot_frame
            marker.id = int(i)
            marker.action = Marker.MODIFY
            marker.type = Marker.CUBE
            pose_arr = numpify(pose.pose)
            center = self.clearance_box.mean(axis=1, keepdims=True)
            pose_arr[:3, 3:] += np.dot(pose_arr[:3, :3], center)
            marker.pose = msgify(Pose, pose_arr)
            marker.scale.x, marker.scale.y, marker.scale.z = (
                self.clearance_box[:, 1] - self.clearance_box[:, 0]
            )
            free, pts = self.check_pose_clearance(pose.pose)
            marker.color = (
                ColorRGBA(r=0.0, g=1.0, b=0.0, a=0.25)
                if free
                else ColorRGBA(r=1.0, g=0.0, b=0.0, a=0.25)
            )
            markers.append(marker)

            if pts is not None:
                for p in pts.T:
                    pts_marker.points.append(
                        Point(x=float(p[0]), y=float(p[1]), z=float(p[2]))
                    )
                    pts_marker.colors.append(ColorRGBA(r=1.0, g=0.0, b=0.0, a=0.5))

        if pts_marker.points:
            markers.append(pts_marker)

        return markers

    def waypoint_markers(self, stamp, path_msg, indices, color=None):
        if color is None:
            color = ColorRGBA(r=0.0, g=1.0, b=0.0, a=0.5)

        marker = Marker()
        marker.header.frame_id = path_msg.header.frame_id
        marker.header.stamp = stamp
        marker.ns = "%s/waypoints" % self.robot_frame
        marker.id = 0
        marker.action = Marker.MODIFY
        marker.type = Marker.SPHERE_LIST
        marker.scale = Vector3(x=0.25, y=0.25, z=0.25)
        marker.color = color
        marker.pose.orientation.w = 1.0

        for i in indices:
            pose = path_msg.poses[i]
            marker.points.append(pose.pose.position)
            marker.colors.append(marker.color)

        return [marker]

    def publish_markers(self, path_msg=None, clearance_indices=(), waypoint_indices=()):
        if not path_msg:
            with self.path_lock:
                path_msg = self.path_msg

        t = timer()
        now = self.now().to_msg()
        msg = MarkerArray()
        msg.markers.extend(self.path_markers(now, path_msg))
        msg.markers.extend(self.clearance_markers(now, path_msg, clearance_indices))
        msg.markers.extend(self.waypoint_markers(now, path_msg, waypoint_indices))

        # Send all markers.
        self.markers_pub.publish(msg)

        self.get_logger().debug("Publish clearance: %.3f s" % (timer() - t))

    def compute_path_costs(self, poses):
        """Calculate cumulative path cost for all waypoints.
        This include distance, traversability, and turning cost.
        """
        t = timer()
        costs = [0.0]

        for i in range(1, len(poses)):
            p0, p1 = poses[i - 1 : i + 1]
            assert isinstance(p0, PoseStamped)
            assert isinstance(p1, PoseStamped)
            pose_0 = numpify(p0.pose)
            pose_1 = numpify(p1.pose)

            dist = np.linalg.norm(pose_0[:3, 3] - pose_1[:3, 3])
            c01 = 0.0
            c01 += 1.06 * self.distance_cost(dist)
            c01 += 1.08 * self.distance_cost(dist) * self.pose_cost(pose_1)[0]
            _, _, yaw_0 = euler_from_matrix(pose_0)
            _, _, yaw_1 = euler_from_matrix(pose_1)
            yaw_diff = abs(yaw_1 - yaw_0)
            yaw_diff += min(yaw_diff, 2.0 * np.pi - yaw_diff)
            c01 += 0.24 * self.turning_cost(yaw_diff)
            costs.append(costs[-1] + c01)

        self.get_logger().info(
            "Path time cost: %.1f s (%.2f s)." % (costs[-1], timer() - t)
        )
        return costs

    def maybe_invoke_backtracking(self):
        if self.idle_since is None:
            self.idle_since = self.now()
            return False
        else:
            idle_duration = (self.now() - self.idle_since).nanoseconds / 1e9
            if idle_duration >= self.backtrack_after:
                path = Path()
                path.header.frame_id = self.map_frame
                path.header.stamp = self.now().to_msg()
                path.poses = [
                    PoseStamped(header=path.header, pose=pose)
                    for pose in reversed(self.path_traversed)
                ]
                self.get_logger().warning("Backtracking due to long inactivity.")
                self.path_received(path)
                return True
            else:
                return False

    def turning_cost(self, angle):
        return angle / self.max_angular_rate

    def distance_cost(self, dist):
        return dist / self.max_speed

    def pose_cost(self, pose):
        assert isinstance(pose, np.ndarray)
        assert pose.shape == (4, 4)

        roll, pitch, yaw = euler_from_matrix(pose)
        cost = np.abs(roll) / self.max_roll + np.abs(pitch) / self.max_pitch

        return cost, roll, pitch

    def limit_acceleration(self, speed, angular_rate):
        now = self.now()
        if self.prev_time is None:
            self.prev_time = now
            self.prev_speed = speed
            self.prev_angular_rate = angular_rate
            return speed, angular_rate

        dt = (now - self.prev_time).nanoseconds / 1e9
        if dt < 1e-3:
            return self.prev_speed, self.prev_angular_rate

        min_speed = self.prev_speed - self.max_accel * dt
        max_speed = self.prev_speed + self.max_accel * dt
        if speed < min_speed or speed > max_speed:
            self.get_logger().info(
                "Cannot reach desired speed %.2f m/s, out of acceleration limits "
                "[%.2f, %.2f] m/s." % (speed, min_speed, max_speed)
            )
        speed = np.clip(speed, min_speed, max_speed)

        min_angular_rate = self.prev_angular_rate - self.max_angular_accel * dt
        max_angular_rate = self.prev_angular_rate + self.max_angular_accel * dt
        if angular_rate < min_angular_rate or angular_rate > max_angular_rate:
            self.get_logger().info(
                "Cannot reach desired angular rate %.2f rad/s, out of acceleration "
                "limits [%.2f, %.2f] rad/s."
                % (angular_rate, min_angular_rate, max_angular_rate)
            )
        angular_rate = np.clip(angular_rate, min_angular_rate, max_angular_rate)

        self.prev_time = now
        self.prev_speed = speed
        self.prev_angular_rate = angular_rate

        return speed, angular_rate

    def publish_smoothed(self, linear, angular):
        # Limit linear and angular acceleration.
        linear, angular = self.limit_acceleration(linear, angular)

        msg = Twist()
        msg.angular.z = float(angular)
        msg.linear.x = float(linear)
        self.cmd_pub.publish(msg)
        self.get_logger().info(
            "Linear: %.2f m/s, angular rate: %.1f rad/s." % (linear, angular)
        )

    def control(self):
        with self.path_lock:
            pose_msg = self.get_robot_pose(self.map_frame)
            cur_pos = numpify(pose_msg.position)
            prev_pos = (
                numpify(self.path_traversed[0].position)
                if len(self.path_traversed) > 0
                else None
            )
            if prev_pos is None or np.linalg.norm(cur_pos - prev_pos) > 0.1:
                self.path_traversed.append(pose_msg)
            if len(self.path_traversed) > 3000:
                self.path_traversed = self.path_traversed[-3000:]

            if self.path_msg is None:
                self.maybe_invoke_backtracking()
                self.publish_smoothed(0.0, 0.0)
                return

            pose = numpify(pose_msg)
            self.get_logger().debug(
                "Control from robot position: [%.2f, %.2f, %.2f]"
                % (pose[0, 3], pose[1, 3], pose[2, 3])
            )

            # Get the last position on the path within look-ahead radius,
            # else extend the radius to max. path distance.
            if self.local_goal_dims == "xy":
                pose = pose_in_2d(pose)
            ind = self.path_x_index.query_ball_point(pose[:3, 3:].T, r=self.look_ahead)[
                0
            ]
            if not ind:
                self.get_logger().warning(
                    "Distance to path higher than look ahead %.1f m." % self.look_ahead
                )
                ind = self.path_x_index.query_ball_point(
                    pose[:3, 3:].T, r=self.max_path_dist
                )[0]
            if not ind:
                self.get_logger().warning(
                    "Distance to path higher than maximum %.1f m. Stopping."
                    % self.max_path_dist
                )
                self.clear_path()
                self.maybe_invoke_backtracking()
                self.publish_smoothed(0.0, 0.0)
                return

            self.idle_since = None

            # Ensure minimum look-ahead still applies for long distances
            # between poses, e.g. starting position far from the next one.
            last = len(self.path_msg.poses) - 1
            i = max(ind)
            assert self.path.shape[1] == 3
            goal = self.path[i, :].reshape([3, 1])
            look_ahead = np.linalg.norm(pose[:3, 3:] - goal)
            while i < last and look_ahead < self.look_ahead:
                i += 1
                new_goal = self.path[i, :].reshape([3, 1])
                look_ahead += np.linalg.norm(new_goal - goal)
                goal = new_goal

            if self.increasing_waypoint_index and self.waypoint_index:
                i = max(i, self.waypoint_index)
            self.waypoint_index = i

            clearance_indices = range(
                max(i + self.show_clearance[0], 0),
                min(i + self.show_clearance[1], last),
            )
            self.publish_markers(
                self.path_msg, clearance_indices=clearance_indices, waypoint_indices=[i]
            )
            if not self.check_pose_clearance(self.path_msg.poses[i].pose)[0]:
                if self.stuck_since is None:
                    self.stuck_since = self.now()
                stuck_duration = (self.now() - self.stuck_since).nanoseconds / 1e9
                if stuck_duration < self.force_through_after:
                    self.get_logger().warning(
                        "Path to goal obstructed (for %.1f s), waiting..."
                        % stuck_duration
                    )
                    self.publish_smoothed(0.0, 0.0)
                    return
                else:
                    self.get_logger().warning(
                        "Path to goal obstructed for %.1f s >= %.1f s, forcing through..."
                        % (stuck_duration, self.force_through_after)
                    )
            else:
                if self.stuck_since:
                    self.get_logger().warning("Path free again.")
                self.stuck_since = None

            # Convert the goal into robot frame.
            local_goal = p2e(np.linalg.solve(pose, e2p(goal)))
            if self.local_goal_dims == "xy":
                local_goal[2, 0] = 0.0
            dist = np.linalg.norm(local_goal)
            self.get_logger().debug(
                "Local goal: %.2f, %.2f, %.2f (%.2f m apart)"
                % tuple(local_goal.ravel().tolist() + [dist])
            )
            self.get_logger().info(
                "Local goal: %.2f, %.2f, %.2f (%.2f m apart)"
                % tuple(local_goal.ravel().tolist() + [dist]),
                throttle_duration_sec=1.0,
            )

            # TODO: Use goal theta.
            # Angular displacement from [-pi, pi)
            if (
                True
                or self.use_path_theta == "none"
                or (self.use_path_theta == "last" and i < last)
                or np.isnan(goal[2])
            ):
                angle = np.arctan2(local_goal[1, 0], local_goal[0, 0])
            else:
                goal_theta = goal[2]
                self.get_logger().info("Using path theta: %.1f." % goal_theta)

            # Clear path and stop if the goal has been reached.
            if (
                i == last
                and dist <= self.goal_reached_dist
                and abs(angle) <= self.goal_reached_angle
            ):
                est_time = self.path_costs[-1] if self.path_costs else float("nan")
                act_time = (self.now() - self.path_received_time).nanoseconds / 1e9
                self.get_logger().warning(
                    "Goal reached: %.2f m from robot (<= %.2f m). "
                    "Est. time %.1f s, actual %.1f s."
                    % (dist, self.goal_reached_dist, est_time, act_time)
                )
                self.clear_path()
                if self.next_path_msg:
                    self.get_logger().info(
                        "Using stored subsequent path (%i poses)."
                        % len(self.next_path_msg.poses)
                    )
                    self.path_received(self.next_path_msg)
                else:
                    self.publish_smoothed(0.0, 0.0)
                if self.max_path_dists:
                    self.max_path_dist = self.max_path_dists.pop()
                return

        if self.allow_backward and np.abs(angle) > np.pi / 2.0:
            angle = np.mod(angle + np.pi / 2.0, np.pi) - np.pi / 2.0
            vel_sign = -1.0
        else:
            vel_sign = 1.0

        # Angular rate
        angular_rate = self.p_angle * angle
        angular_rate = np.clip(
            angular_rate, -self.max_angular_rate, self.max_angular_rate
        )

        # Linear velocity
        speed = vel_sign * self.p_dist * dist
        speed = speed * max(0.0, 1.0 - (abs(angle) / self.turn_on_spot_angle) ** 2)
        # Limit speed for higher roll and pitch.
        pose_cost, roll, pitch = self.pose_cost(pose)
        self.get_logger().debug("Roll: %.3f, max roll: %.3f" % (roll, self.max_roll))
        self.get_logger().debug(
            "Pitch: %.3f, max pitch: %.3f" % (pitch, self.max_pitch)
        )
        speed /= 1.0 + pose_cost
        max_speed = (
            self.max_speed if self.stuck_since is None else self.max_force_through_speed
        )
        speed = np.clip(speed, -max_speed, max_speed)

        self.publish_smoothed(speed, angular_rate)

    def control_safe(self):
        t = timer()
        try:
            self.control()
        except tf2_ros.TransformException as ex:
            self.get_logger().error("Robot pose lookup failed: %s." % ex)
        except Exception as ex:
            traceback.print_exc()
            self.get_logger().error("Unknown exception during contol: %s." % ex)

        t = timer() - t
        if t >= 1.0 / self.control_freq:
            self.get_logger().warning("Control loop iteration took %.3f s." % t)
        else:
            self.get_logger().info(
                "Control loop iteration took %.3f s." % t, throttle_duration_sec=5.0
            )


def main(args=None):
    rclpy.init(args=args)
    node = PathFollower()
    executor = MultiThreadedExecutor()
    try:
        rclpy.spin(node, executor=executor)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
