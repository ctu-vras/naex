"""Naex path follower (`path_follower.py`, documented as the `follower` node)."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

INF = float('inf')


def base_parameters(robot):
    return {
        'map_frame': 'subt',
        'odom_frame': '%s/odom' % robot,
        'robot_frame': '%s/base_footprint' % robot,
        'control_freq': 10.0,
        'goal_reached_dist': 0.3,
        'goal_reached_angle': INF,
        'use_path_theta': 'none',
        'max_age': 1.0,
        # Max. path distances, tail is consumed first by reached goals.
        # A higher value is used in the beginning to traverse across
        # unobserved area around robot.
        'max_path_dist': [0.8, 2.5, 2.5, 2.5],
        'look_ahead': 0.8,
        'max_speed': 1.0,
        'max_accel': 1.0,
        'allow_backward': False,
        'max_force_through_speed': 0.2,
        'turn_on_spot_angle': 0.524,  # 30 deg.
        'max_angular_rate': 1.5,
        'max_angular_accel': 2.0,
        'keep_path': 3.0,
        # Boxes are flat [xmin, xmax, ymin, ymax, zmin, zmax]; ROS 2 parameters
        # cannot be nested as the ROS 1 [[..], [..], [..]] lists were.
        'keep_cloud_box': [-3.0, 3.0, -3.0, 3.0, -2.0, 2.0],
        'clearance_box': [-0.6, 0.6, -0.45, 0.45, 0.2, 0.7],
        'show_clearance_pos': [-2, 2],
        'min_points_obstacle': 3,

        'force_through_after': 10.0,
        'backtrack_after': 20.0,
    }


ROBOT_PARAMETERS = {
    'dtr': {
        'clearance_box': [-0.5, 0.5, -0.4, 0.4, 0.2, 0.7],
    },
    'jeanine': {
        'max_path_dist': [0.8, 1.5, 1.5, 1.5],
        'max_speed': 1.0,
        'max_angular_rate': 1.5,
        'clearance_box': [-0.4, 0.4, -0.3, 0.3, 0.2, 0.6],
    },
    'marv': {
        'allow_backward': True,
        'clearance_box': [-0.55, 0.55, -0.4, 0.4, 0.2, 0.7],
    },
    'tradr': {
        'clearance_box': [-0.5, 0.5, -0.4, 0.4, 0.2, 0.6],
    },
    'x2': {
        'allow_backward': True,
        'clearance_box': [-0.4, 0.4, -0.3, 0.3, 0.15, 0.5],
    },
}

# Applies to every robot type containing 'x1'.
X1_PARAMETERS = {
    'allow_backward': True,
    'clearance_box': [-0.6, 0.6, -0.45, 0.45, 0.2, 0.7],
}


def launch_setup(context, *args, **kwargs):
    robot = LaunchConfiguration('robot').perform(context)
    robot_type = LaunchConfiguration('robot_type').perform(context)
    cmd_vel = LaunchConfiguration('cmd_vel').perform(context)
    path = LaunchConfiguration('path').perform(context)
    points = LaunchConfiguration('points').perform(context)
    use_sim_time = LaunchConfiguration('use_sim_time').perform(context).lower() in (
        'true', '1', 'yes')

    params = base_parameters(robot)
    if 'x1' in robot_type:
        params.update(X1_PARAMETERS)
    params.update(ROBOT_PARAMETERS.get(robot_type, {}))
    params['use_sim_time'] = use_sim_time

    return [
        Node(
            package='naex',
            executable='path_follower.py',
            name='path_follower',
            output='screen',
            respawn=True,
            respawn_delay=1.0,
            parameters=[params],
            remappings=[
                # Inputs
                ('path', path),
                ('cloud', points),
                # Outputs
                ('cmd_vel', cmd_vel),
            ],
        ),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('robot', default_value='X1'),
        DeclareLaunchArgument('robot_type', default_value='x1'),
        DeclareLaunchArgument('cmd_vel', default_value='cmd_vel',
                              description='Topic to which to publish velocity commands.'),
        DeclareLaunchArgument('path', default_value='path',
                              description='Path topic to follow.'),
        DeclareLaunchArgument('points', default_value='points_slow_filtered'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        OpaqueFunction(function=launch_setup),
    ])
