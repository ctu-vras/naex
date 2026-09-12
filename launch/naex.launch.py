"""Full naex stack for one robot: TF, preprocessing, odometry, SLAM,
planner, follower and recording, all inside the robot namespace."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import PushRosNamespace


def include(name, args):
    path = os.path.join(get_package_share_directory('naex'), 'launch', name)
    return IncludeLaunchDescription(PythonLaunchDescriptionSource(path),
                                    launch_arguments=args.items())


def launch_setup(context, *args, **kwargs):
    robot = LaunchConfiguration('robot').perform(context)
    robot_type = LaunchConfiguration('robot_type').perform(context)
    robots = LaunchConfiguration('robots').perform(context)
    points = LaunchConfiguration('points').perform(context)
    points_min_dist = LaunchConfiguration('points_min_dist').perform(context)
    cmd_vel = LaunchConfiguration('cmd_vel').perform(context)
    path = LaunchConfiguration('path').perform(context)
    use_sim_time = LaunchConfiguration('use_sim_time').perform(context)

    if not points:
        points = 'front_laser/points' if robot_type == 'jeanine' else 'points'

    common = {
        'robot': robot,
        'robot_type': robot_type,
        'robots': robots,
        'points_min_dist': points_min_dist,
        'use_sim_time': use_sim_time,
    }

    slam_points = ('points_filtered_batch' if robot_type == 'dtr'
                   else '%s_slow_filtered' % points)

    group = GroupAction([
        PushRosNamespace(robot),

        include('tf.launch.py', {'robot': robot, 'robot_type': robot_type,
                                 'use_sim_time': use_sim_time}),
        include('preproc.launch.py', dict(common, points=points)),
        include('odom.launch.py', {'robot': robot, 'use_sim_time': use_sim_time}),
        include('slam.launch.py', dict(common, points=slam_points)),
        include('planner.launch.py', dict(common, points='%s_slow' % points)),
        include('follower.launch.py', {
            'robot': robot,
            'robot_type': robot_type,
            'cmd_vel': cmd_vel,
            'path': path,
            'points': '%s_slow_filtered' % points,
            'use_sim_time': use_sim_time,
        }),
        include('record.launch.py', dict(common, mode='none')),
    ])

    return [group]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('robot', default_value='X1',
                              description='Robot name, also ROS graph and TF namespace.'),
        DeclareLaunchArgument(
            'robot_type', default_value='explorer_x1',
            description='Robot type, from {absolem, dtr, explorer_x1, jeanine, marv, x1, x2}.'),
        DeclareLaunchArgument('robots', default_value=LaunchConfiguration('robot')),
        DeclareLaunchArgument('points', default_value='',
                              description='Raw point cloud topic; defaults to '
                                          'front_laser/points for jeanine, points otherwise.'),
        DeclareLaunchArgument('points_min_dist', default_value='0.125'),
        DeclareLaunchArgument('cmd_vel', default_value='cmd_vel'),
        DeclareLaunchArgument('path', default_value='path'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        OpaqueFunction(function=launch_setup),
        # TODO(ros2): the ROS 1 launcher also included
        # $(find subt_example)/launch/teleop.launch; subt_example has no ROS 2
        # release, so the teleop is not started here.
        # The optional rpz_planning includes and the rviz node were disabled
        # (if="0") in ROS 1 and are dropped.
    ])
