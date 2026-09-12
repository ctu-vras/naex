"""Compress point clouds with draco and record input / output bags."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

DRACO_PARAMETERS = {
    'encode_speed': 7,
    'decode_speed': 7,
    'encode_method': 1,
    'deduplicate': True,
    'force_quantization': True,

    'quantization_POSITION': 13,
    'quantization_NORMAL': 4,
    'quantization_COLOR': 4,
    'quantization_TEX_COORD': 8,
    'quantization_GENERIC': 8,

    'expert_attribute_types': False,
    'expert_quantization': False,
}


def draco_republish(name, topic, use_sim_time):
    """raw -> draco republisher for one topic.

    TODO(ros2): in ROS 1 the draco settings were global parameters under
    <topic>/draco/. point_cloud_transport declares plugin parameters on the
    publishing node under <base topic>.<transport>.<name>; verify the prefix
    against your point_cloud_transport version if the settings seem ignored.
    """
    params = {'in_transport': 'raw', 'out_transport': 'draco',
              'use_sim_time': use_sim_time}
    params.update({'%s.draco.%s' % (topic, key): value
                   for key, value in DRACO_PARAMETERS.items()})
    return Node(
        package='point_cloud_transport', executable='republish', name=name,
        output='log', respawn=True, respawn_delay=1.0,
        parameters=[params],
        arguments=['raw', 'draco'],
        remappings=[('in', topic), ('out', topic)],
    )


def launch_setup(context, *args, **kwargs):
    robot = LaunchConfiguration('robot').perform(context)
    robot_type = LaunchConfiguration('robot_type').perform(context)
    mode = LaunchConfiguration('mode').perform(context)
    use_sim_time = LaunchConfiguration('use_sim_time').perform(context).lower() in (
        'true', '1', 'yes')

    actions = [
        Node(package='topic_tools', executable='throttle', name='throttle_local_map',
             output='log', respawn=True, respawn_delay=1.0,
             parameters=[{'use_sim_time': use_sim_time}],
             arguments=['messages', 'local_map', '0.1', 'local_map_slow']),
    ]

    input_topics = ['/%s/points/draco' % robot, '/tf', '/tf_static']
    output_topics = ['/%s/points_slow/draco' % robot,
                     '/%s/points_slow_filtered/draco' % robot,
                     '/%s/local_map_slow/draco' % robot,
                     '/%s/updated_map/draco' % robot,
                     '/%s/path' % robot,
                     '/%s/control_path' % robot,
                     '/%s/cmd_vel' % robot]

    # TODO(ros2): ros2 bag has no --lz4; zstd file compression is used instead.
    # The output prefix is relative to the working directory (the ROS 1 launcher
    # wrote into the naex source/share directory, which is read-only here).
    if mode in ('input', 'both'):
        actions.append(draco_republish('points_draco', 'points', use_sim_time))
        actions.append(ExecuteProcess(
            name='record_naex_input',
            cmd=['ros2', 'bag', 'record',
                 '--compression-mode', 'file', '--compression-format', 'zstd',
                 '-o', '%s_input' % robot_type] + input_topics,
            output='screen'))

    if mode in ('output', 'both'):
        for topic in ('points_slow', 'points_slow_filtered', 'local_map_slow',
                      'updated_map'):
            actions.append(draco_republish('%s_draco' % topic, topic, use_sim_time))
        actions.append(ExecuteProcess(
            name='record_naex_output',
            cmd=['ros2', 'bag', 'record',
                 '--compression-mode', 'file', '--compression-format', 'zstd',
                 '-o', '%s_output' % robot_type] + output_topics,
            output='screen'))

    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('robot', default_value='X1'),
        DeclareLaunchArgument('robot_type', default_value='x1'),
        DeclareLaunchArgument('mode', default_value='none',
                              description='Recording mode, from '
                                          '{none, input, output, both}.'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        OpaqueFunction(function=launch_setup),
    ])
