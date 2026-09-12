"""Naex global planner (point-map based `planner` node).

The per-robot parameter blocks of the SubT era (x1/x2/dtr/jeanine/marv/tradr)
were dropped; what is left is the single working parameter set.  Override any
of it with `parameters:=<file.yaml>` or by editing `base_parameters()`.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# Pitch / roll limits, for reference:
#   5 deg 0.087   10 0.175   15 0.262   20 0.349   25 0.436
#  30 deg 0.524   35 0.611   40 0.698   45 0.785


def base_parameters(map_frame, robot_frame):
    return {
        "position_name": "x",
        "normal_name": "normal_x",
        "map_frame": map_frame,
        "robot_frame": robot_frame,
        "max_cloud_age": 2.0,
        "input_range": 15.0,
        "max_pitch": 0.611,
        "max_roll": 0.524,
        "neighborhood_radius": 0.6,
        "min_points_obstacle": 1.0,
        "max_ground_diff_std": 0.06,
        "max_mean_abs_ground_diff": 0.06,
        "edge_min_centroid_offset": 0.4,
        "min_dist_to_obstacle": 0.0,
        "viewpoints_update_freq": 1.0,
        "max_vp_distance": 6.0,
        "full_coverage_dist": 3.0,
        "coverage_dist_spread": 1.5,
        "path_cost_pow": 0.75,
        "min_path_cost": 1.0,
        "planning_freq": 0.5,
        "random_start": False,
        "plan_from_goal_dist": 1.5,
        "min_num_empty": 4,
        "min_empty_ratio": 2.0,
        "max_occ_counter": 7,
        "min_empty_cos": 0.216,
        # Have some clearance defaults.
        "clearance_radius": 0.6,
        "clearance_low": 0.1,
        "clearance_high": 0.7,
        "num_input_clouds": 1,
        "input_queue_size": 15,
    }


def launch_setup(context, *args, **kwargs):
    map_frame = LaunchConfiguration("map_frame").perform(context)
    robot_frame = LaunchConfiguration("robot_frame").perform(context)
    points = LaunchConfiguration("points").perform(context)
    points_min_dist = float(LaunchConfiguration("points_min_dist").perform(context))
    planning_freq = float(LaunchConfiguration("planning_freq").perform(context))
    use_sim_time = LaunchConfiguration("use_sim_time").perform(context).lower() in (
        "true",
        "1",
        "yes",
    )

    params = base_parameters(map_frame, robot_frame)
    params["points_min_dist"] = points_min_dist
    # 0.0 turns the node into a mapper (what mapper.launch.py used to be).
    params["planning_freq"] = planning_freq
    params["use_sim_time"] = use_sim_time

    return [
        Node(
            package="naex",
            executable="planner",
            name="naex_planner",
            output="screen",
            respawn=True,
            respawn_delay=1.0,
            parameters=[params],
            remappings=[
                ("input_cloud_0", points),
            ],
        ),
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("map_frame", default_value="map"),
            DeclareLaunchArgument("robot_frame", default_value="base_footprint"),
            DeclareLaunchArgument("points", default_value="points_slow"),
            DeclareLaunchArgument("points_min_dist", default_value="0.125"),
            DeclareLaunchArgument(
                "planning_freq",
                default_value="0.5",
                description="Re-planning frequency; 0.0 maps only.",
            ),
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            OpaqueFunction(function=launch_setup),
        ]
    )
