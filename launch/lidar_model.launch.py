"""Lidar model node with a throttled point cloud input."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    points = LaunchConfiguration('points')
    points_slow = LaunchConfiguration('points_slow')
    points_slow_freq = LaunchConfiguration('points_slow_freq')
    use_sim_time = LaunchConfiguration('use_sim_time')

    return LaunchDescription([
        DeclareLaunchArgument('points', default_value='points'),
        DeclareLaunchArgument('points_slow', default_value='points_slow'),
        DeclareLaunchArgument('points_slow_freq', default_value='1.0'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),

        Node(
            package='topic_tools',
            executable='throttle',
            name='points_throttle',
            output='log',
            respawn=True,
            respawn_delay=1.0,
            parameters=[{'use_sim_time': use_sim_time}],
            arguments=['messages', points, points_slow_freq, points_slow],
        ),
        Node(
            package='naex',
            executable='lidar_model',
            name='lidar_model',
            output='screen',
            respawn=True,
            respawn_delay=1.0,
            parameters=[{
                'check_model': True,
                'use_sim_time': use_sim_time,
            }],
            remappings=[('cloud', points_slow)],
        ),
    ])
