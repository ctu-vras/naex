"""Play recorded bag files from SubT virtual robots.

Usage in a directory with bag files:
    bags=$(ls $(pwd)/*.mcap) ros2 launch naex playback.launch.py rate:=10.0
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.substitutions import EnvironmentVariable, LaunchConfiguration
from launch_ros.actions import Node

# Remap used topics into original namespace, to make nodes and rviz
# configs compatible with online experiment.
REMAPPED_SUFFIXES = ('points_slow_filtered/draco', 'local_map_slow/draco',
                     'updated_map/draco', 'path', 'control_path')
# Topics republished from draco back to raw.
REPUBLISHED = ('points_slow_filtered', 'local_map_slow', 'updated_map')


def launch_setup(context, *args, **kwargs):
    bags = LaunchConfiguration('bags').perform(context).split()
    rate = LaunchConfiguration('rate').perform(context)
    config = LaunchConfiguration('config').perform(context)
    robots = [r.strip() for r in LaunchConfiguration('robots').perform(context).split(',')
              if r.strip() and r.strip() != 'TEAMBASE']

    remaps = []
    for robot in ('X1', 'X2', 'X3'):
        for suffix in REMAPPED_SUFFIXES:
            remaps += ['--remap', '/robot_data/%s/%s:=/%s/%s'
                       % (robot, suffix, robot, suffix)]
    remaps += ['--remap', '/robot_data/tf:=/tf',
               '--remap', '/robot_data/tf_static:=/tf_static']

    actions = [
        # TODO(ros2): static_transform_mux (ROS 1 only) has no ROS 2 release; it
        # merged several /tf_static publishers into one. Without it, multiple
        # static transform publishers still work in ROS 2 (transient local
        # /tf_static is per publisher), so it is simply omitted.

        ExecuteProcess(
            name='rosbag_play',
            cmd=['ros2', 'bag', 'play', '--clock', '--rate', rate, '--delay', '1.0']
                + bags + remaps,
            output='screen'),
    ]

    # Draco to raw.
    for robot in robots:
        for topic in REPUBLISHED:
            actions.append(Node(
                package='point_cloud_transport', executable='republish',
                name='%s_%s_draco_to_raw' % (robot, topic.replace('/', '_')),
                output='log',
                parameters=[{'in_transport': 'draco', 'out_transport': 'raw',
                             'use_sim_time': True}],
                arguments=['draco', 'raw'],
                remappings=[('in', '%s/%s' % (robot, topic)),
                            ('out', '%s/%s' % (robot, topic))],
            ))

    # TODO(ros2): launch/legacy_rviz1/subt.rviz is a ROS 1 rviz config and
    # cannot be loaded by rviz2; recreate it and point `config` at the new file.
    actions.append(Node(package='rviz2', executable='rviz2', name='rviz2',
                        output='log',
                        parameters=[{'use_sim_time': True}],
                        arguments=['-d', config] if config else []))

    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('bags',
                              default_value=EnvironmentVariable('bags',
                                                                default_value='')),
        DeclareLaunchArgument('rate', default_value='10.0'),
        DeclareLaunchArgument('config', default_value='',
                              description='rviz2 config; the original subt.rviz is a '
                                          'ROS 1 config kept in launch/legacy_rviz1/.'),
        DeclareLaunchArgument('robots', default_value='X1,X2,X3'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        OpaqueFunction(function=launch_setup),
    ])
