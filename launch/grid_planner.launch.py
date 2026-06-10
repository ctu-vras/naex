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
                        "astar_max_range": 20.0, # meters radius to perform search around the start vertex
                        "frontier_min_dist": 2.,    # generally keep this above lookahead point distance of controller
                        "frontier_max_neighbors": 7, # probably don't change this
                        "max_relative_dist_to_goal": 2.0,

                        # if start is further than this from the nearest traversable point,
                        # we will just plan a straight line to the goal
                        # (this should only happen when using navigate through poses)
                        "max_start_to_traversable_dist": 2.0,
                        
                        "position_field": "x",
                        "map_frame": "map",
                        "robot_frame": "os_sensor",
                        "max_cloud_age": 5.0,
                        "input_range": 5.0,
                        "cell_size": 0.6,
                        "forget_factor": 1.0,
                        "cost_fields": ["traversability"],
                        "which_cloud": [0],

                        # Don't forget that cloud_weights are not relative, becuase in the planner they are combined
                        # with the euclidean length of the edge in meters.
                        # E.g. if we weight the geometry cloud by 10. and we get a 0.5 geometry cost, it tranlates to 
                        # a cost of 5. for an edge of the graph of size 1 meter (where the base cost is 1 per 1 meter traveled).
                        # So we are saying that the path along this edge is equal to finding a different route to the same goal point
                        # of length 5 meters and with 0 cost.
                        "cloud_weights": [
                            1.0,
                        ],  # BEST RUN WAS WITH [1.0, 2.0, 10.0]
                        "max_costs_relative": [
                            0.7,
                        ],
                        "default_costs": [   # Careful that these are never multiplied by the cloud_weights!!!
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
                    ("input_cloud_0", "terrain_map"),
                    ("map_occupancy_grid", "naex/map_occupancy_grid"),
                ],
            )
        ]
    )
