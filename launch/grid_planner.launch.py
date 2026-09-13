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
                        "astar_max_range": 20.0,  # meters radius to perform search around the start vertex
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
                        "map_frame": "FP_ENU0",
                        "robot_frame": "odin1_base_link",
                        "max_cloud_age": 5.0,
                        # Cloud callbacks must never park the single-threaded
                        # executor on TF, but the timeout still has to
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
                        # True subscribes to the input clouds with best-effort
                        # (sensor data) QoS; the default is reliable.
                        "sensor_data_qos": False,
                        # Crop of the input cloud around the sensor.
                        # Bounds the per-cloud work; it does not bound the map,
                        # because cells are never removed by it.
                        "input_range": 5.0,
                        # Bound of the map itself: cells farther than
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
                        "cell_size": 0.2,
                        "forget_factor": 1.0,
                        "cost_fields": ["cost", "traversability"],
                        "which_cloud": [0, 1],
                        # Cost layer each cost field is written to; defaults to
                        # the index of the field.  Must not name adhoc_layer.
                        "cloud_levels": [0, 1],
                        # Per-cost-field threshold (parallel to cost_fields).
                        # A point is only added to the grid for a cost field if its
                        # value is strictly greater than the corresponding threshold.
                        # Use -inf (default) to keep every point. E.g. for a binary
                        # segmentation cloud, set 0.0 to only keep obstacle points (>0).
                        "min_cloud_values": [
                            float("-inf"), float("-inf"),
                        ],
                        # Per-cost-field obstacle inflation radius in meters
                        # (parallel to cost_fields). For an above-threshold
                        # (obstacle) point, its cost is also stamped onto every
                        # neighbouring cell within this radius. 0.0 disables it.
                        # Keep 0.0 for the continuous geometric layer; set a
                        # positive value only for a binary segmentation cloud.
                        "inflation_radius": [
                            0.0, 0.0,
                        ],
                        # Don't forget that cloud_weights are not relative, becuase in the planner they are combined
                        # with the euclidean length of the edge in meters.
                        # E.g. if we weight the geometry cloud by 10. and we get a 0.5 geometry cost, it tranlates to
                        # a cost of 5. for an edge of the graph of size 1 meter (where the base cost is 1 per 1 meter traveled).
                        # So we are saying that the path along this edge is equal to finding a different route to the same goal point
                        # of length 5 meters and with 0 cost.
                        "cloud_weights": [
                            5.0, 1.0,
                        ],
                        "max_costs_relative": [
                            100.0, 0.6,
                        ],
                        "default_costs": [   # Careful that these are never multiplied by the cloud_weights!!!
                            # Road layer: a cell no road point landed on (grass, or footway the camera has
                            # not seen). road_cloud's cost runs 0 (centre) .. 1 (edge), x weight 5 = 0..5, so
                            # at 5.0 grass cost the same as the outer half of the footway (2026-09-11 bags).
                            15.0, 0.4,
                        ],
                        # Used for finding the best frontier as temporary goal. The cost of the frontier is:
                        # the AStar cost of getting there + euclidean_dist_to_goal * frontier_dist_from_goal_cost
                        "frontier_dist_from_goal_cost": 1.5,
                        # Goal snapping: a goal that is not on the road moves to the nearest road cell within
                        # goal_snap_radius, so a waypoint beside the footway plans to the footway edge instead
                        # of onto the grass. A road cell is one whose road-layer cost (level 0, weighted) is
                        # <= goal_snap_max_cost: 3.5 = road_cloud cost 0.7, inside the outermost rim. Keep it
                        # below default_costs[0], so a cell the road layer never saw never qualifies, and keep
                        # the radius <= crl_commander's sequence_pass_lateral_dist, which then counts the
                        # waypoint as passed. 0 radius = off.
                        "goal_snap_radius": 4.5,
                        "goal_snap_level": 0,
                        "goal_snap_max_cost": 3.5,
                        "neighborhood": 8,
                        "planning_freq": 1.0,
                        "num_input_clouds": 2,
                        "input_queue_size": 2,
                        "start_on_request": True,
                        "stop_on_goal": True,
                        "goal_reached_dist": 0.5,
                        # The snapped goal cell (see goal_snap_* above) would otherwise be followed by the
                        # requested, off-road goal pose; the robot's elrob config did not append it either.
                        "append_goal_pose": False,
                        # Ad-hoc cost parameters; uncomment to enable
                        # "adhoc_costs": [],
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
                    ("input_cloud_0", "road_cloud"),
                    # ("input_cloud_1", "unexplored_map"),
                    ("input_cloud_1", "terrain_map"),
                    # ("input_cloud_0", "traversability_cloud"),
                    ("map_occupancy_grid", "naex/map_occupancy_grid"),
                    ("grid_planner/compute_path_to_pose", "astar/compute_path_to_pose"),
                ],
            )
        ]
    )
