"""Static transform from base_link to base_footprint."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    base_link = LaunchConfiguration('base_link')
    base_footprint = LaunchConfiguration('base_footprint')
    z_offset = LaunchConfiguration('z_offset')
    use_sim_time = LaunchConfiguration('use_sim_time')

    return LaunchDescription([
        DeclareLaunchArgument('base_link', default_value='base_link'),
        DeclareLaunchArgument('base_footprint', default_value='base_footprint'),
        DeclareLaunchArgument('z_offset', default_value='0.0'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),

        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_footprint_pub',
            output='log',
            parameters=[{'use_sim_time': use_sim_time}],
            arguments=[
                '--x', '0.0', '--y', '0.0', '--z', z_offset,
                '--qx', '0.0', '--qy', '0.0', '--qz', '0.0', '--qw', '1.0',
                '--frame-id', base_link,
                '--child-frame-id', base_footprint,
            ],
        ),
    ])
