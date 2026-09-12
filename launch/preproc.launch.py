"""Slow down and filter input point clouds:
points -> points_slow -> points_slow_filtered.

Also runs adapters for TRADR and Husky:
dynamic_point_cloud = points_slow
os_cloud_node/points -> points_slow
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# Robot types that need the robot body cropped out of the cloud.
BOX_FILTER_ROBOT_TYPES = ('dtr', 'jeanine', 'marmotte', 'marv')

# TODO(ros2): the ROS 1 launcher used the cloud_proc/CropBoxImproved nodelet,
# which has no ROS 2 release. pcl_ros::CropBox (filter_crop_box_node) is used
# instead; verify that negative/keep_organized/input_frame/output_frame behave
# the same for your data before trusting the result.
CROP_BOX_PARAMETERS = {
    'dtr': {
        'input_frame': '{robot}/base_footprint',
        'min_x': -0.475, 'max_x': 0.475,
        'min_y': -0.3, 'max_y': 0.3,
        'max_z': 0.8,
        'output_frame': '{robot}/encoder_rotating_link/gpu_lidar',
    },
    'jeanine': {
        'min_x': -0.2, 'max_x': 0.2,
        'min_y': -0.2, 'max_y': 0.2,
        'min_z': -0.2, 'max_z': 0.2,
    },
    'marmotte': {
        'input_frame': '{robot}/base_footprint',
        'min_x': -0.475, 'max_x': 0.475,
        'min_y': -0.3, 'max_y': 0.3,
        'max_z': 0.8,
        'output_frame': '{robot}/sensor_rack/front_lidar',
    },
    'marv': {
        'input_frame': '{robot}/base_footprint',
        'min_x': -0.5, 'max_x': 0.5,
        'min_y': -0.4, 'max_y': 0.4,
        'max_z': 0.6,
        'output_frame': '{robot}/laser/laser',
    },
}


def throttle(name, in_topic, freq, out_topic, use_sim_time):
    return Node(
        package='topic_tools', executable='throttle', name=name, output='log',
        respawn=True, respawn_delay=1.0,
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=['messages', in_topic, str(freq), out_topic],
    )


def voxel_grid(name, in_topic, out_topic, leaf_size, use_sim_time):
    # TODO(ros2): replaces the ROS 1 `nodelet standalone pcl/VoxelGrid`;
    # run it as a component in a container if zero-copy matters.
    return Node(
        package='pcl_ros', executable='filter_voxel_grid_node', name=name, output='log',
        parameters=[{
            'leaf_size': leaf_size,
            'filter_field_name': '',
            'use_sim_time': use_sim_time,
        }],
        remappings=[('input', in_topic), ('output', out_topic)],
    )


def launch_setup(context, *args, **kwargs):
    robot = LaunchConfiguration('robot').perform(context)
    robot_type = LaunchConfiguration('robot_type').perform(context)
    points = LaunchConfiguration('points').perform(context)
    points_slow = LaunchConfiguration('points_slow').perform(context) or '%s_slow' % points
    points_slow_freq = float(LaunchConfiguration('points_slow_freq').perform(context))
    points_slow_filtered = (LaunchConfiguration('points_slow_filtered').perform(context)
                            or '%s_filtered' % points_slow)
    points_min_dist = float(LaunchConfiguration('points_min_dist').perform(context))
    use_sim_time = LaunchConfiguration('use_sim_time').perform(context).lower() in (
        'true', '1', 'yes')

    use_robot_box_filter = robot_type in BOX_FILTER_ROBOT_TYPES
    throttled = '%s_pre' % points_slow if use_robot_box_filter else points_slow

    actions = [
        # TRADR adaptor
        Node(package='topic_tools', executable='relay', name='dynamic_point_cloud_relay',
             output='log', respawn=True, respawn_delay=1.0,
             parameters=[{'use_sim_time': use_sim_time}],
             arguments=['dynamic_point_cloud', points_slow]),
        # Husky adaptor
        throttle('os_points_throttle', 'os_cloud_node/points', points_slow_freq,
                 points_slow, use_sim_time),
        throttle('points_throttle', points, points_slow_freq, throttled, use_sim_time),
    ]

    if use_robot_box_filter:
        params = {'min_z': 0.0, 'negative': True, 'keep_organized': True}
        params.update({k: (v.format(robot=robot) if isinstance(v, str) else v)
                       for k, v in CROP_BOX_PARAMETERS[robot_type].items()})
        params['use_sim_time'] = use_sim_time
        actions.append(Node(
            package='pcl_ros', executable='filter_crop_box_node',
            name='robot_box_filter', output='log',
            parameters=[params],
            remappings=[('input', throttled), ('output', points_slow)],
        ))

    actions.append(voxel_grid('points_slow_voxels', points_slow, points_slow_filtered,
                              points_min_dist, use_sim_time))

    # SubT virtual EXPLORER_X1 RGBD sensors
    if robot_type == 'explorer_x1':
        for side in ('front', 'left', 'right', 'rear'):
            actions.append(throttle('%s_rgbd_points_throttle' % side,
                                    '%s_rgbd/points' % side, 1.0,
                                    '%s_rgbd/points_slow' % side, use_sim_time))
            actions.append(voxel_grid('%s_rgbd_points_voxels' % side,
                                      '%s_rgbd/points_slow' % side,
                                      '%s_rgbd/points_slow_filtered' % side,
                                      points_min_dist, use_sim_time))

    # SubT virtual Jeanine RGBD sensor
    if robot_type == 'jeanine':
        actions.append(throttle('rgbd_front_points_throttle', 'rgbd_front/points', 1.0,
                                'rgbd_front/points_slow', use_sim_time))
        actions.append(voxel_grid('rgbd_front_points_voxels', 'rgbd_front/points_slow',
                                  'rgbd_front/points_slow_filtered', points_min_dist,
                                  use_sim_time))

    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('robot', default_value='X1'),
        DeclareLaunchArgument('robot_type', default_value='x1'),
        DeclareLaunchArgument('points', default_value='points'),
        DeclareLaunchArgument('points_slow', default_value='',
                              description='Defaults to <points>_slow.'),
        DeclareLaunchArgument('points_slow_freq', default_value='4.0'),
        DeclareLaunchArgument('points_slow_filtered', default_value='',
                              description='Defaults to <points_slow>_filtered.'),
        DeclareLaunchArgument('points_min_dist', default_value='0.125'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        OpaqueFunction(function=launch_setup),
    ])
