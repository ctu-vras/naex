"""Play Skoda factory mapping bag files through the naex mapper.

Usage in a directory with bag files:
    bags=$(ls $(pwd)/*.mcap) ros2 launch naex play_skoda.launch.py rate:=20.0
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess, GroupAction,
                            IncludeLaunchDescription, OpaqueFunction)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace


def launch_setup(context, *args, **kwargs):
    bags = LaunchConfiguration('bags').perform(context).split()
    rate = LaunchConfiguration('rate').perform(context)
    config = LaunchConfiguration('config').perform(context)
    robot = LaunchConfiguration('robot').perform(context)
    robot_type = LaunchConfiguration('robot_type').perform(context)
    robots = LaunchConfiguration('robots').perform(context)

    mapper = os.path.join(get_package_share_directory('naex'), 'launch',
                          'mapper.launch.py')

    actions = [
        # TODO(ros2): static_transform_mux (ROS 1 only) has no ROS 2 release;
        # omitted, see playback.launch.py.

        ExecuteProcess(
            name='rosbag_play',
            cmd=['ros2', 'bag', 'play', '--clock', '--rate', rate, '--delay', '1.0']
                + bags,
            output='screen'),

        GroupAction([
            PushRosNamespace(robot),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(mapper),
                launch_arguments={
                    'robot': robot,
                    'robot_type': robot_type,
                    'robots': robots,
                    'use_sim_time': 'true',
                }.items()),
        ]),

        # TODO(ros2): the ROS 1 launcher used launch/skoda.rviz, which is not in
        # this repository; provide an rviz2 config through the `config` argument.
        Node(package='rviz2', executable='rviz2', name='rviz2', output='log',
             parameters=[{'use_sim_time': True}],
             arguments=['-d', config] if config else []),
    ]
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('bags',
                              default_value=EnvironmentVariable('bags',
                                                                default_value='')),
        DeclareLaunchArgument('rate', default_value='1.0'),
        DeclareLaunchArgument('config', default_value=''),
        DeclareLaunchArgument('robot', default_value='X1'),
        DeclareLaunchArgument('robot_type', default_value='explorer_x1'),
        DeclareLaunchArgument('robots', default_value=LaunchConfiguration('robot')),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        OpaqueFunction(function=launch_setup),
    ])
