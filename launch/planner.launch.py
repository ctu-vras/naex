"""Naex global planner (point-map based `planner` node)."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# Pitch / roll limits, for reference:
#   5 deg 0.087   10 0.175   15 0.262   20 0.349   25 0.436
#  30 deg 0.524   35 0.611   40 0.698   45 0.785

NAN = float('nan')


def base_parameters(robot):
    return {
        'position_name': 'x',
        'normal_name': 'normal_x',
        'map_frame': 'subt',
        'robot_frame': '%s/base_footprint' % robot,
        'max_cloud_age': 2.0,
        'input_range': 15.0,
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
        'max_vp_distance': 6.0,
        'collect_rewards': True,
        'full_coverage_dist': 3.0,
        'coverage_dist_spread': 1.5,
        'self_factor': 0.5,
        'suppress_base_reward': True,
        'path_cost_pow': 0.75,
        'min_path_cost': 1.0,
        'planning_freq': 0.5,
        'random_start': False,
        'plan_from_goal_dist': 1.5,
        'bootstrap_z': NAN,

        'min_num_empty': 4,
        'min_empty_ratio': 2.0,
        'max_occ_counter': 7,
        'min_empty_cos': 0.216,

        'filter_robots': True,

        # Have some clearance defaults.
        'clearance_radius': 0.6,
        'clearance_low': 0.1,
        'clearance_high': 0.7,

        'num_input_clouds': 1,
        'input_queue_size': 15,
    }


# Customize things for particular robots.
ROBOT_PARAMETERS = {
    'dtr': {
        'max_cloud_age': 5.0,
        'max_ground_diff_std': 0.1,
        'max_ground_abs_diff_mean': 0.1,
        'clearance_radius': 0.7,
        'clearance_low': 0.15,
        'clearance_high': 0.7,
    },
    'marv': {
        'clearance_radius': 0.6,
        'clearance_low': 0.15,
        'clearance_high': 0.7,
    },
    'tradr': {
        'clearance_radius': 0.5,
        'clearance_low': 0.2,
        'clearance_high': 0.6,
        'max_cloud_age': 5.0,
    },
    'x2': {
        'clearance_radius': 0.5,
        'clearance_low': 0.1,
        'clearance_high': 0.5,
        'num_input_clouds': 2,
    },
    'jeanine': {
        'clearance_radius': 0.5,
        'clearance_low': 0.1,
        'clearance_high': 0.5,
        'num_input_clouds': 2,
    },
}

# Applies to every robot type containing 'x1'.
X1_PARAMETERS = {
    'clearance_radius': 0.6,
    'clearance_low': 0.1,
    'clearance_high': 0.7,
    'num_input_clouds': 5,
}


def launch_setup(context, *args, **kwargs):
    robot = LaunchConfiguration('robot').perform(context)
    robots = LaunchConfiguration('robots').perform(context)
    robot_type = LaunchConfiguration('robot_type').perform(context)
    points = LaunchConfiguration('points').perform(context)
    points_min_dist = float(LaunchConfiguration('points_min_dist').perform(context))
    use_sim_time = LaunchConfiguration('use_sim_time').perform(context).lower() in (
        'true', '1', 'yes')

    params = base_parameters(robot)
    params['points_min_dist'] = points_min_dist
    if 'x1' in robot_type:
        params.update(X1_PARAMETERS)
    params.update(ROBOT_PARAMETERS.get(robot_type, {}))
    # TODO(ros2): the ROS 1 launcher filled naex_planner/robot_frames/{x} using
    # subt_virtual/launch/helpers/for.launch, which has no ROS 2 equivalent.
    # The frames are passed as a plain string array instead; the ported C++ node
    # has to read robot_frames as a string array (ROS 2 has no map parameters).
    params['robot_frames'] = ['%s/base_footprint' % r.strip()
                              for r in robots.split(',')
                              if r.strip() and r.strip() != 'TEAMBASE']
    params['use_sim_time'] = use_sim_time

    remappings = [
        # Don't use map input.
        ('input_map', '~/input_map'),
        ('input_cloud_0', points),
    ]
    if robot_type == 'explorer_x1':
        remappings += [
            ('input_cloud_1', 'front_rgbd/points_slow'),
            ('input_cloud_2', 'left_rgbd/points_slow'),
            ('input_cloud_3', 'right_rgbd/points_slow'),
            ('input_cloud_4', 'rear_rgbd/points_slow'),
        ]
    elif robot_type == 'jeanine':
        remappings += [('input_cloud_1', 'rgbd_front/points_slow')]

    return [
        Node(
            package='naex',
            executable='planner',
            name='naex_planner',
            output='screen',
            respawn=True,
            respawn_delay=1.0,
            # NOTE: the ROS 1 launcher used launch-prefix="catchsegv", which no
            # longer ships with glibc; use prefix='gdb -batch -ex run --args' if
            # backtraces are needed.
            parameters=[params],
            remappings=remappings,
        ),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('robot', default_value='X1'),
        DeclareLaunchArgument('robots', default_value=LaunchConfiguration('robot')),
        DeclareLaunchArgument('robot_type', default_value='x1'),
        DeclareLaunchArgument('points', default_value='points_slow'),
        DeclareLaunchArgument('points_min_dist', default_value='0.125'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        OpaqueFunction(function=launch_setup),
    ])
