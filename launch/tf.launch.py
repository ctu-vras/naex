"""Robot TF tree helpers (base_link -> base_footprint offset per robot type)."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

# Vertical offset of base_footprint below base_link, per robot type.
Z_OFFSETS = {
    'explorer_x1': -0.132,
    'jeanine': -0.132,
    'marv': -0.15,
    'tradr': -0.07,
    'x1': -0.132,
    'dtr': -0.104,
}


def launch_setup(context, *args, **kwargs):
    robot = LaunchConfiguration('robot').perform(context)
    robot_type = LaunchConfiguration('robot_type').perform(context)
    z_offset = Z_OFFSETS.get(robot_type, 0.0)

    footprint = os.path.join(get_package_share_directory('naex'), 'launch',
                             'footprint.launch.py')
    return [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(footprint),
            launch_arguments={
                'base_link': '%s/base_link' % robot,
                'base_footprint': '%s/base_footprint' % robot,
                'z_offset': str(z_offset),
                'use_sim_time': LaunchConfiguration('use_sim_time').perform(context),
            }.items(),
        ),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('robot', default_value='X1'),
        DeclareLaunchArgument('robot_type', default_value='x1'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        OpaqueFunction(function=launch_setup),
    ])
