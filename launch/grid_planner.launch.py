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
                        # Cloud callbacks must never park the single-threaded
                        # executor on TF (P5): drop the frame after ~1 cloud
                        # period instead.  tf_timeout applies to the
                        # once-per-cycle robot pose lookup only.
                        "cloud_tf_timeout": 0.05,
                        "tf_timeout": 0.5,
                        "input_range": 5.0,
                        "cell_size": 0.4,
                        "forget_factor": 0.1,
                        "cost_fields": ["geometric_cost"],
                        "which_cloud": [0],
                        "cloud_weights": [
                            2.0,
                        ],  # BEST RUN WAS WITH [1.0, 2.0, 10.0]
                        "max_costs": [
                            float("nan"),
                        ],
                        "default_costs": [0.5],
                        "neighborhood": 8,
                        "planning_freq": 1.0,
                        "num_input_clouds": 1,
                        "input_queue_size": 2,
                        "start_on_request": True,
                        "stop_on_goal": True,
                        "goal_reached_dist": 0.5,
                        "mode": 2,
                        # Ad-hoc cost parameters; uncomment to enable
                        "adhoc_costs": ["sidelobes"],
                        #"adhoc_costs": ["nothing"],
                        "adhoc_layer": 3,
                        # Sidelobes strategy parameters
                        "sidelobes_offset_distance": 1.0,
                        "sidelobes_radius": 0.8,
                        "sidelobes_cost": 10.0,
                        "sidelobes_angle_offsets": [-90.0, 90.0, 180.0],
                    }
                ],
                remappings=[
                    #("input_cloud_0", "osm_grid"),
                    ("input_cloud_0", "geometric_traversability_cloud"),
                ],
            )
        ]
    )
