"""Naex mapper: the `planner` node configured for mapping only (planning_freq 0)."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def base_parameters(robot):
    return {
        'position_name': 'x',
        'normal_name': 'normal_x',
        'map_frame': '%s/map' % robot,
        'robot_frame': '%s/base_footprint' % robot,
        'max_cloud_age': 2.0,
        'input_range': 10.0,
        'max_pitch': 0.611,
        'max_roll': 0.524,
        'neighborhood_knn': 32,
        'neighborhood_radius': 0.6,
        'normal_radius': 0.5,

        'max_nn_height_diff': 0.15,
        'min_points_obstacle': 1.0,
        'max_ground_diff_std': 0.06,
        'max_ground_abs_diff_mean': 0.06,
        'edge_min_centroid_offset': 0.4,
        'min_dist_to_obstacle': 0.0,

        'viewpoints_update_freq': 1.0,
        'min_vp_distance': 2.0,
        'max_vp_distance': 5.0,
        'self_factor': 0.5,
        'path_cost_pow': 0.75,
        'planning_freq': 0.0,
        'random_start': False,

        'min_num_empty': 4,
        'min_empty_ratio': 2.0,
        'max_occ_counter': 7,
        'min_empty_cos': 0.216,

        'filter_robots': True,

        'num_input_clouds': 1,
        'input_queue_size': 15,
    }


def launch_setup(context, *args, **kwargs):
    robot = LaunchConfiguration('robot').perform(context)
    robot_type = LaunchConfiguration('robot_type').perform(context).lower()
    points_slow = LaunchConfiguration('points_slow').perform(context)
    points_min_dist = float(LaunchConfiguration('points_min_dist').perform(context))
    use_sim_time = LaunchConfiguration('use_sim_time').perform(context).lower() in (
        'true', '1', 'yes')

    params = base_parameters(robot)
    params['points_min_dist'] = points_min_dist
    if 'tradr' in robot_type:
        params['max_cloud_age'] = 5.0
    if 'x1' in robot_type:
        params.update({
            'clearance_radius': 0.6,
            'clearance_low': 0.1,
            'clearance_high': 0.7,
            'num_input_clouds': 5,
        })
    if 'x2' in robot_type:
        params.update({
            'clearance_radius': 0.5,
            'clearance_low': 0.1,
            'clearance_high': 0.5,
        })
    params['use_sim_time'] = use_sim_time

    remappings = [('input_cloud_0', points_slow)]
    if 'explorer_x1' in robot_type:
        remappings += [
            ('input_cloud_1', 'front_rgbd/points_slow'),
            ('input_cloud_2', 'left_rgbd/points_slow'),
            ('input_cloud_3', 'right_rgbd/points_slow'),
            ('input_cloud_4', 'rear_rgbd/points_slow'),
        ]
    remappings += [
        # Global and local maps: don't subscribe global if not needed,
        # use local map for visualization.
        ('map', 'fine_map'),
        ('local_map', 'fine_local_map'),
        # Points to update (graph, features, labels)
        ('dirty_map', '~/dirty_map'),
        # Added / removed points
        ('map_diff', '~/map_diff'),
        # Planned path
        ('path', '~/path'),
    ]

    return [
        Node(
            package='naex',
            executable='planner',
            name='naex_mapper',
            output='log',
            respawn=True,
            respawn_delay=1.0,
            parameters=[params],
            remappings=remappings,
        ),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('robot', default_value='X1'),
        DeclareLaunchArgument('robots', default_value=LaunchConfiguration('robot')),
        DeclareLaunchArgument('robot_type', default_value='x1'),
        DeclareLaunchArgument('points_slow', default_value='points_slow'),
        DeclareLaunchArgument('points_min_dist', default_value='0.05'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        OpaqueFunction(function=launch_setup),
    ])
