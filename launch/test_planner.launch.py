from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    package_dir = get_package_share_directory("grid_planner")
    rviz_config = os.path.join(package_dir, "config", "grid_planner_test.rviz")

    return LaunchDescription(
        [
            # Grid planner with test configuration
            Node(
                package="naex",
                executable="grid_planner",
                name="grid_planner",
                output="screen",
                parameters=[
                    {
                        "position_field": "x",
                        "map_frame": "local_utm",
                        "robot_frame": "base_link",
                        "max_cloud_age": 10.0,
                        "input_range": 15.0,
                        "cell_size": 0.2,
                        "forget_factor": 1.0,
                        "cost_fields": ["cost"],
                        "which_cloud": [0],
                        "cloud_weights": [1.0],
                        "max_costs": [
                            float("nan"),
                            float("nan"),
                            float("nan"),
                            float("nan"),
                        ],
                        "default_costs": [
                            float("nan"),
                            float("nan"),
                            float("nan"),
                            float("nan"),
                        ],
                        "neighborhood": 8,
                        "min_path_cost": 1.0,
                        "planning_freq": 1.0,
                        "plan_from_goal_dist": 1.0,
                        "num_input_clouds": 2,
                        "input_queue_size": 5,
                        "start_on_request": False,
                        "stop_on_goal": True,
                        "goal_reached_dist": 0.5,
                        "mode": 2,
                        "plan_to_goal": True,
                        # Ad-hoc cost parameters
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
                ],
            ),
            # Mock map publisher
            Node(
                package="naex",
                executable="mock_map_publisher.py",
                name="mock_map_publisher",
                output="screen",
                parameters=[
                    {
                        "map_size": 20.0,
                        "map_cost": 1.0,
                        "obstacle_size": 6.0,
                        "obstacle_center_x": 0.0,
                        "obstacle_center_y": 0.0,
                        "grid_resolution": 0.1,
                        "obstacle_cost": 10.0,
                        "publish_rate": 1.0,
                    }
                ],
            ),
            # Mock TF publisher
            Node(
                package="naex",
                executable="mock_tf_publisher.py",
                name="mock_tf_publisher",
                output="screen",
                parameters=[
                    {
                        "robot_x": -8.0,
                        "robot_y": 3.0,
                        "robot_z": 0.0,
                        "robot_yaw": 90.0,  # Facing north
                    }
                ],
            ),
            # Test client
            Node(
                package="naex",
                executable="test_planner_client.py",
                name="test_planner_client",
                output="screen",
                parameters=[
                    {
                        # NaN for using the current robot position as start
                        "start_x": float("nan"),
                        "start_y": float("nan"),
                        "start_yaw": 90.0,  # Facing north
                        "goal_x": 8.0,
                        "goal_y": 0.0,
                        "goal_yaw": 0.0,
                        "test_delay": 1.0,
                        "obstacle_size": 6.0,
                        "obstacle_center_x": 0.0,
                        "obstacle_center_y": 0.0,
                    }
                ],
            ),
            # RViz for visualization
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                output="screen",
                arguments=["-d", rviz_config],
            ),
        ]
    )

