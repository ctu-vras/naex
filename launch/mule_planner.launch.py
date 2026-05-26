from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    declared_args = [
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="false",
            description="simulation/bag or not",
        )
    ]

    return LaunchDescription(
        declared_args +
        [
            Node(
                package="naex",
                executable="mule_planner",
                name="mule_planner",
                output="screen",
                parameters=[
                    {
                        "astar_max_range": 50.0, # meters radius to perform search around the start vertex

                        # if start is further than this from the nearest traversable point,
                        # replanning fails and an empty path is published
                        "max_start_to_traversable_dist": 5.0,
                        # publish a replanned prefix by itself only if it is at least this long
                        "min_traversable_path_length": 3.0,
                        
                        "robot_frame": "base_link",
                        "position_field": "x",
                        "max_cloud_age": 1.0,
                        "max_ts_diff": 1.0,
                        "cell_size": 0.6,
                        "forget_factor": 1.0,
                        "cost_field": "traversability",
                        "default_costs": [0.5],
                        "neighborhood": 8,
                        "obstacle_cost_threshold": 0.7,
                        "use_sim_time": LaunchConfiguration("use_sim_time")
                    }
                ],
                remappings=[
                    #("input_cloud_0", "osm_grid"),
                    ("path", "/path"),
                    ("points", "/terrain_map"),
                ],
            )
        ]
    )
