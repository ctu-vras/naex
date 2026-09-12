"""ICP-based SLAM (libpointmatcher dynamic mapper).

TODO(ros2): `ethzasl_icp_mapper` (libpointmatcher's `dynamic_mapper`) has no
ROS 2 release. The node action below preserves the original configuration
(including the libpointmatcher YAML files in launch/dynamic_mapper/, which are
not ROS-version specific), but launching it fails until an equivalent ROS 2
mapper is available; its parameter names will most likely differ.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):
    robot = LaunchConfiguration('robot').perform(context)
    icp = LaunchConfiguration('icp').perform(context)
    points = LaunchConfiguration('points').perform(context)
    points_min_dist = float(LaunchConfiguration('points_min_dist').perform(context))
    use_sim_time = LaunchConfiguration('use_sim_time').perform(context).lower() in (
        'true', '1', 'yes')

    config_dir = os.path.join(get_package_share_directory('naex'), 'launch',
                              'dynamic_mapper')
    icp_config = os.path.join(config_dir,
                              'icp_4dof.yaml' if icp == 'point_to_plane_4dof'
                              else 'icp.yaml')

    return [
        Node(
            package='ethzasl_icp_mapper',
            executable='dynamic_mapper',
            name='dynamic_mapper',
            output='log',
            parameters=[{
                'subscribe_scan': False,
                'inputFiltersConfig': os.path.join(config_dir, 'input_filters.yaml'),
                'icpConfig': icp_config,
                'mapPostFiltersConfig': os.path.join(config_dir,
                                                     'map_post_filters.yaml'),

                'odom_frame': '%s/odom' % robot,
                'map_frame': '%s/map' % robot,
                'useROSLogger': True,
                'minOverlap': 0.2,
                'maxOverlapToMerge': 1.0,
                'minReadingPointCount': 500,
                'minMapPointCount': 500,
                'localizing': True,
                'mapping': True,
                # Disable refreshing map-to-odom transform.
                'tfRefreshPeriod': 0.0,
                # Parameters for dynamic elements
                'priorStatic': 0.7,
                'priorDyn': 0.3,
                'maxAngle': 0.02,
                'eps_a': 0.1,  # 1 deg = 0.017 rad
                'eps_d': 0.1,
                'alpha': 0.99,
                'beta': 0.9,
                'maxDyn': 0.50,
                'maxDistNewPoint': points_min_dist,
                'sensorMaxRange': 50.0,
                'use_sim_time': use_sim_time,
            }],
            remappings=[('cloud_in', points)],
        ),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('robot', default_value='X1'),
        DeclareLaunchArgument('robot_type', default_value='explorer_x1'),
        DeclareLaunchArgument('icp', default_value='point_to_plane',
                              description='ICP mode from '
                                          '{point_to_plane, point_to_plane_4dof}.'),
        DeclareLaunchArgument('points', default_value='points_slow_filtered'),
        DeclareLaunchArgument('points_min_dist', default_value='0.125'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        OpaqueFunction(function=launch_setup),
    ])
