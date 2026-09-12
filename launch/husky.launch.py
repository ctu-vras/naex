"""Naex on the Husky robot: preprocessing, ICP SLAM, planner and follower."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

INF = float('inf')


def throttle(name, in_topic, freq, out_topic, use_sim_time):
    return Node(
        package='topic_tools', executable='throttle', name=name, output='log',
        respawn=True, respawn_delay=1.0,
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=['messages', in_topic, str(freq), out_topic],
    )


def voxel_grid(name, in_topic, out_topic, leaf_size, use_sim_time):
    # TODO(ros2): replaces the ROS 1 `nodelet standalone pcl/VoxelGrid`.
    return Node(
        package='pcl_ros', executable='filter_voxel_grid_node', name=name,
        output='screen',
        parameters=[{
            'leaf_size': leaf_size,
            'filter_field_name': '',
            'use_sim_time': use_sim_time,
        }],
        remappings=[('input', in_topic), ('output', out_topic)],
    )


def launch_setup(context, *args, **kwargs):
    cloud = LaunchConfiguration('cloud').perform(context)
    icp = LaunchConfiguration('icp').perform(context)
    cmd_vel = LaunchConfiguration('cmd_vel').perform(context)
    points_min_dist = float(LaunchConfiguration('points_min_dist').perform(context))
    use_sim_time = LaunchConfiguration('use_sim_time').perform(context).lower() in (
        'true', '1', 'yes')

    config_dir = os.path.join(get_package_share_directory('naex'), 'launch',
                              'dynamic_mapper')
    icp_config = os.path.join(config_dir,
                              'icp_4dof.yaml' if icp == 'point_to_plane_4dof'
                              else 'icp.yaml')

    return [
        throttle('os_points_throttle', cloud, 0.5, '%s_slow' % cloud, use_sim_time),
        voxel_grid('os_points_voxels', '%s_slow' % cloud, '%s_slow_filtered' % cloud,
                   points_min_dist, use_sim_time),
        throttle('scan_filtered_throttle', 'scan_filtered', 0.5, 'scan_filtered_slow',
                 use_sim_time),
        voxel_grid('scan_filtered_voxels', 'scan_filtered_slow', 'scan_filtered_voxels',
                   points_min_dist, use_sim_time),

        # TODO(ros2): ethzasl_icp_mapper (libpointmatcher dynamic_mapper) has no
        # ROS 2 release; the configuration is preserved but the node cannot run.
        Node(
            package='ethzasl_icp_mapper',
            executable='dynamic_mapper',
            name='dynamic_mapper',
            output='log',
            parameters=[{
                'subscribe_scan': False,
                'inputFiltersConfig': os.path.join(config_dir, 'input_filters.yaml'),
                'icpConfig': icp_config,
                'mapPostFiltersConfig': os.path.join(config_dir, 'map_post_filters.yaml'),
                'odom_frame': 'odom',
                'map_frame': 'map',
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
            remappings=[('cloud_in', '%s_slow_filtered' % cloud)],
        ),

        Node(
            package='naex',
            executable='planner',
            name='naex_planner',
            output='screen',
            respawn=True,
            respawn_delay=1.0,
            parameters=[{
                'position_name': 'x',
                'normal_name': 'normal_x',
                'map_frame': 'map',
                'robot_frame': 'base_link',
                'max_cloud_age': 5.0,
                'max_pitch': 0.175,
                'max_roll': 0.175,
                'neighborhood_knn': 32,
                'neighborhood_radius': 0.6,
                'normal_radius': 0.5,

                'max_nn_height_diff': 0.15,
                'clearance_radius': 0.6,
                'clearance_low': 0.1,
                'clearance_high': 0.8,
                'min_points_obstacle': 1.0,
                'max_ground_diff_std': 0.08,
                'max_ground_abs_diff_mean': 0.08,
                'edge_min_centroid_offset': 0.4,
                'min_dist_to_obstacle': 0.0,

                'viewpoints_update_freq': 1.0,
                'min_vp_distance': 1.5,
                'max_vp_distance': 5.0,
                'self_factor': 0.5,
                'path_cost_pow': 1.0,
                'planning_freq': 0.5,

                'min_num_empty': 4,
                'min_empty_ratio': 1.5,
                'max_occ_counter': 15,

                'points_min_dist': points_min_dist,
                'filter_robots': True,

                'num_input_clouds': 1,
                'input_queue_size': 15,
                'use_sim_time': use_sim_time,
            }],
            remappings=[
                ('cloud', '~/cloud'),
                ('input_cloud_0', '%s_slow' % cloud),
                ('map', 'planner_map'),
            ],
        ),

        Node(
            package='naex',
            executable='path_follower.py',
            name='path_follower',
            output='screen',
            respawn=True,
            respawn_delay=1.0,
            parameters=[{
                'map_frame': 'map',
                'odom_frame': 'odom',
                'robot_frame': 'base_link',
                'control_freq': 10.0,
                'goal_reached_dist': 0.6,
                'goal_reached_angle': INF,
                'use_path_theta': 'none',
                'max_age': 1.0,
                # Max. path distances, tail is consumed first by reached goals.
                'max_path_dist': [0.8, 2.5, 2.5, 2.5],
                'look_ahead': 0.8,
                'max_speed': 0.3,
                'max_force_through_speed': 0.05,
                'max_angular_rate': 0.5,
                'keep_path': 10.0,
                # Flat [xmin, xmax, ymin, ymax, zmin, zmax] boxes.
                'keep_cloud_box': [-3.0, 3.0, -3.0, 3.0, -2.0, 2.0],
                'clearance_box': [-0.6, 0.6, -0.45, 0.45, 0.15, 0.8],
                'show_clearance_pos': [-2, 2],
                'min_points_obstacle': 3,
                'force_through_after': 3600.0,
                'backtrack_after': 3600.0,
                'allow_backward': True,
                'use_sim_time': use_sim_time,
            }],
            remappings=[
                ('cloud', 'scan_filtered_voxels'),
                ('path', 'path'),
                ('cmd_vel', cmd_vel),
            ],
        ),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('cloud', default_value='os_cloud_node/points'),
        DeclareLaunchArgument('points_min_dist', default_value='0.15'),
        DeclareLaunchArgument('icp', default_value='point_to_plane'),
        DeclareLaunchArgument('cmd_vel', default_value='cmd_vel',
                              description='Name of the topic on which the planner '
                                          'publishes the velocity commands'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        OpaqueFunction(function=launch_setup),
    ])
