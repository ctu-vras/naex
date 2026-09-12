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
                        "use_astar": True,
                        "astar_max_range": 50.0,  # meters radius to perform search around the start vertex
                        "frontier_min_dist": 2.0,  # generally keep this above lookahead point distance of controller
                        "frontier_max_neighbors": 7,  # probably don't change this
                        "max_relative_dist_to_goal": 2.0,
                        # if start is further than this from the nearest traversable point,
                        # we will just plan a straight line to the goal
                        # (this should only happen when using navigate through poses)
                        # Because this only comes into play when the start is unexplored.
                        # I could see this being a problem when we start the robot and its
                        # position is naturally unexplored and at the same time there is obstacles all around
                        "max_start_to_traversable_dist": 2.0,
                        "position_field": "x",
                        "map_frame": "local_odom",
                        "robot_frame": "base_link",
                        "max_cloud_age": 5.0,
                        # Cloud callbacks must never park the single-threaded
                        # executor on TF (P5), but the timeout still has to
                        # cover one TF period: every drop seen in bag replay
                        # was an "extrapolation into the future" by 1-6 ms
                        # against a 10 Hz TF, i.e. a transform that was there
                        # one period later.  0.2 = 2 TF periods.
                        "cloud_tf_timeout": 0.2,
                        # Timeout of every TF lookup on the get_plan path,
                        # which runs on that same single thread.  All of them
                        # are "latest available", so this only ever elapses
                        # when TF is genuinely absent.
                        "request_tf_timeout": 0.5,
                        # Deprecated, unused; kept so that this file documents
                        # the migration.  Setting it (to anything but 3.0)
                        # still seeds request_tf_timeout.
                        "tf_timeout": 0.5,
                        # True subscribes to the input clouds with best-effort
                        # (sensor data) QoS; the default is reliable.
                        "sensor_data_qos": False,
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
                        # Keep it >= astar_max_range when use_astar is on.
                        "map_range": 0.0,
                        # Upper bound on the interval between two evictions
                        # while the robot stands still; it also evicts after
                        # 0.25 * map_range of travel.
                        "evict_period": 10.0,
                        "cell_size": 0.4,
                        "forget_factor": 0.1,
                        "cost_fields": ["geometric_cost"],
                        "which_cloud": [0],
                        # Cost layer each cost field is written to; defaults to
                        # the index of the field.  Must not name adhoc_layer.
                        "cloud_levels": [0],
                        # Don't forget that cloud_weights are not relative, becuase in the planner they are combined
                        # with the euclidean length of the edge in meters.
                        # E.g. if we weight the geometry cloud by 10. and we get a 0.5 geometry cost, it tranlates to
                        # a cost of 5. for an edge of the graph of size 1 meter (where the base cost is 1 per 1 meter traveled).
                        # So we are saying that the path along this edge is equal to finding a different route to the same goal point
                        # of length 5 meters and with 0 cost.
                        "cloud_weights": [
                            2.0,
                        ],  # BEST RUN WAS WITH [1.0, 2.0, 10.0]
                        "max_costs_relative": [
                            0.8,
                        ],
                        "default_costs": [  # Careful that these are never multiplied by the cloud_weights!!!
                            0.5
                        ],
                        "neighborhood": 8,
                        "min_path_cost": 1.0,
                        "planning_freq": 1.0,
                        "plan_from_goal_dist": 2.0,
                        "num_input_clouds": 1,
                        "input_queue_size": 2,
                        "start_on_request": True,
                        "stop_on_goal": True,
                        "goal_reached_dist": 0.5,
                        "mode": 2,
                        # Ad-hoc cost parameters; uncomment to enable
                        "adhoc_costs": ["sidelobes"],
                        # "adhoc_costs": ["nothing"],
                        "adhoc_layer": 3,
                        # Sidelobes strategy parameters
                        "sidelobes_offset_distance": 1.0,
                        "sidelobes_radius": 0.8,
                        "sidelobes_cost": 10.0,
                        "sidelobes_angle_offsets": [-90.0, 90.0, 180.0],
                    }
                ],
                remappings=[
                    # ("input_cloud_0", "osm_grid"),
                    ("input_cloud_0", "geometric_traversability_cloud"),
                ],
            )
        ]
    )
