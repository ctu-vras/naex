"""Odometry fusion (uncertain odometry + IMU).

TODO(ros2): neither node below has a ROS 2 release.
  * ``subt_virtual``/``odom_setter`` is a SubT-virtual-only ROS 1 package.
  * ``robot_pose_ekf`` was never ported to ROS 2; ``robot_localization``
    (``ekf_node``) is the usual replacement, but its parameters differ, so the
    substitution is intentionally NOT made here.
The actions are kept so the intended structure and parameters are preserved;
launching this file fails until the packages (or replacements) exist.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    robot = LaunchConfiguration('robot')
    use_sim_time = LaunchConfiguration('use_sim_time')

    return LaunchDescription([
        DeclareLaunchArgument('robot', default_value='X1'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),

        # TODO(ros2): no ROS 2 equivalent of subt_virtual/odom_setter.
        Node(
            package='subt_virtual',
            executable='odom_setter',
            name='odom_setter',
            output='log',
            parameters=[{
                'pose_cov_diag': [.1, .1, .1, .1, .1, .1],
                'twist_cov_diag': [.1, .1, .1, .1, .1, .1],
                'use_sim_time': use_sim_time,
            }],
            remappings=[
                ('input', 'odom'),
                ('output', 'uncertain_odom'),
            ],
        ),

        # TODO(ros2): robot_pose_ekf has no ROS 2 release; consider
        # robot_localization/ekf_node with an equivalent configuration.
        Node(
            package='robot_pose_ekf',
            executable='robot_pose_ekf',
            name='robot_pose_ekf',
            output='log',
            parameters=[{
                # According to docs base_footprint should be used but the pose
                # passthrough publishes transforms with this frame as the child
                # so we have to use the root robot frame like "X1".
                'output_frame': PythonExpression(["'", robot, "' + '/odom'"]),
                'base_footprint_frame': robot,
                'freq': 20.0,
                'sensor_timeout': 1.0,
                'odom_used': True,
                'imu_used': True,
                'vo_used': False,
                'use_sim_time': use_sim_time,
            }],
            remappings=[
                ('odom', 'uncertain_odom'),
                ('imu_data', 'imu/data'),
                ('~/odom_combined', 'imu_pose'),
            ],
        ),
    ])
