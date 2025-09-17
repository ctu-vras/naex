from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="naex",
                executable="grid_planner",
                name="grid_planner",
                output="screen",
                parameters=[
                    {
                        "position_field": "x",
                        "map_frame": "gps_odom",
                        "robot_frame": "base_link",
                        "max_cloud_age": 5.0,
                        "input_range": 5.0,
                        "cell_size": 0.4,
                        "forget_factor": 0.1,
                        "cost_fields": ["cost", "semantic_cost", "geometric_cost"],
                        "which_cloud": [0, 1, 1],
                        "cloud_weights": [
                            1.0,
                            10.0,
                            2.0,
                        ],  # BEST RUN WAS WITH [1.0, 2.0, 10.0]
                        "max_costs": [
                            float("nan"),
                            float("nan"),
                            float("nan"),
                            float("nan"),
                        ],
                        "default_costs": [1, 0.5, 0.5, float("nan")],
                        "neighborhood": 8,
                        "min_path_cost": 1.0,
                        "planning_freq": 1.0,
                        "plan_from_goal_dist": 2.0,
                        "num_input_clouds": 2,
                        "input_queue_size": 2,
                        "start_on_request": True,
                        "stop_on_goal": True,
                        "goal_reached_dist": 0.5,
                        "mode": 2,
                        # Ad-hoc cost parameters; uncomment to enable
                        "adhoc_costs": ["sidelobes"],
                        "adhoc_layer": 3,
                        # Sidelobes strategy parameters
                        "sidelobes_offset_distance": 1.0,
                        "sidelobes_radius": 0.8,
                        "sidelobes_cost": 10.0,
                        "sidelobes_angle_offsets": [-90.0, 90.0, 180.0],
                    }
                ],
                remappings=[
                    ("input_cloud_0", "osm_grid"),
                    ("input_cloud_1", "traversability_cloud"),
                ],
            )
        ]
    )
