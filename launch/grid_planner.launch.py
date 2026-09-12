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
                        # Crop of the input cloud around the sensor (P6a).
                        # Bounds the per-cloud work; it does not bound the map,
                        # because cells are never removed by it.
                        "input_range": 5.0,
                        # Bound of the map itself (P6b): cells farther than
                        # map_range from the robot are dropped.  0 keeps the
                        # historical behaviour, an unbounded map that grows for
                        # the whole mission (215 k cells and 0.9 s of Dijkstra
                        # after 30 s of driving in the benchmark).
                        # Trade-off: a smaller map_range caps cells, memory and
                        # planning time (pi*r^2/cell_size^2), but the planner
                        # forgets the topology outside it, so a goal further
                        # away degrades to the nearest reachable cell and a
                        # detour around an obstacle larger than map_range can
                        # be forgotten while the robot is still driving it.
                        # Rule of thumb: several times the largest obstacle the
                        # robot must circumnavigate; 100.0 is a safe first
                        # value at cell_size 0.4, tune down toward 50.0.
                        "map_range": 0.0,
                        # Upper bound on the interval between two evictions
                        # while the robot stands still; it also evicts after
                        # 0.25 * map_range of travel.
                        "evict_period": 10.0,
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
