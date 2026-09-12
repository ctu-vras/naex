"""Headless performance benchmark: grid_planner + the synthetic load generator.

    ros2 launch naex bench_grid_planner.launch.py field_size:=200.0 duration:=30.0

Nothing here needs a display.  The benchmark node prints a single
``BENCH_RESULT {...}`` JSON line when it is done and, if ``output`` is set,
writes the same JSON to that file.

P5 acceptance run (TF outage halfway through, with the planning timer on so
that ``path`` messages exist)::

    ros2 launch naex bench_grid_planner.launch.py \\
        field_size:=200.0 planning_freq:=1.0 tf_gap:=2.0

P6 acceptance run (the grid grows for 30 s; map_range caps it, input_range
caps the per-cloud work)::

    ros2 launch naex bench_grid_planner.launch.py \\
        field_size:=200.0 grow:=true grow_start_size:=20.0 map_range:=30.0
    ros2 launch naex bench_grid_planner.launch.py \\
        field_size:=100.0 input_range:=15.0 log_level:=debug

``bench_script`` defaults to the installed script.  Before the CMake install
rule is integrated (see scratchpad/cmake_additions_perf.md) it can be pointed
at the source tree:

    ros2 launch ./launch/bench_grid_planner.launch.py \\
        bench_script:=$PWD/scripts/bench_grid_planner.py
"""

import os
import sys

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    ExecuteProcess,
    RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

_ARGS = {
    # Benchmark load.
    "field_size": "200.0",
    "point_spacing": "0.4",
    "num_obstacles": "40",
    "obstacle_size": "6.0",
    "cloud_rate": "2.0",
    "grow": "false",
    "grow_start_size": "20.0",
    "request_rate": "1.0",
    # 0/0 means "opposite corner of the field".  Note that a lattice-aligned
    # synthetic field whose point_spacing equals cell_size leaves whole cell
    # rows empty (floor() of an exact multiple lands one cell low for some
    # indices), so the far corner is usually in a different connected
    # component than the robot; pick a nearby goal to exercise the A* "goal
    # reached" branch.
    "goal_x": "0.0",
    "goal_y": "0.0",
    "duration": "30.0",
    "warmup": "3.0",
    "subscribe_map": "true",
    # P5 acceptance: stop broadcasting map -> base_link for tf_gap seconds,
    # starting at tf_gap_at of the run.  Needs planning_freq > 0, otherwise no
    # path message is published and the executor stall is invisible.
    "tf_gap": "0.0",
    "tf_gap_at": "0.5",
    # "debug" also emits the per-cloud "perf cloud[i]" line (TF + point loop).
    "log_level": "info",
    "output": "none",
    # Planner.
    "cell_size": "0.4",
    "neighborhood": "8",
    # A* instead of the full Dijkstra, and the radius it crops the search to.
    # max_costs stays NaN here, i.e. every cell is traversable and the search
    # is guided by the accumulated cost only.
    "use_astar": "false",
    "astar_max_range": "50.0",
    # P6.  Both default to 0 (disabled) so that every configuration recorded
    # before P6 reproduces: input_range crops the cloud around the sensor,
    # map_range bounds the grid itself.  The "grow:=true" configuration is the
    # one map_range is meant for.
    "input_range": "0.0",
    "map_range": "0.0",
    "evict_period": "10.0",
    # 0 keeps every measured cycle request-driven; > 0 also runs the planning
    # timer, which is what publishes the "path" topic the tf_gap run measures.
    "planning_freq": "0.0",
    # Baseline values on purpose: the production launch file sets 0.5 / 0.05.
    "tf_timeout": "3.0",
    "cloud_tf_timeout": "0.05",
}


def _default_bench_script():
    """lib/naex/bench_grid_planner.py next to the installed share directory."""
    try:
        share = get_package_share_directory("naex")
    except Exception:  # package not found: fall back to the source tree
        return os.path.join(
            os.path.dirname(os.path.abspath(__file__)),
            "..",
            "scripts",
            "bench_grid_planner.py",
        )
    prefix = os.path.dirname(os.path.dirname(share))
    return os.path.join(prefix, "lib", "naex", "bench_grid_planner.py")


def generate_launch_description():
    args = [
        DeclareLaunchArgument(name, default_value=default)
        for name, default in _ARGS.items()
    ]
    args.append(
        DeclareLaunchArgument("bench_script", default_value=_default_bench_script())
    )

    cfg = {name: LaunchConfiguration(name) for name in _ARGS}

    planner = Node(
        package="naex",
        executable="grid_planner",
        name="grid_planner",
        output="screen",
        ros_arguments=["--log-level", ["grid_planner:=", cfg["log_level"]]],
        parameters=[
            {
                "position_field": "x",
                "map_frame": "map",
                "robot_frame": "base_link",
                "max_cloud_age": 5.0,
                "cell_size": 0.4,
                "forget_factor": 0.1,
                "cost_fields": ["geometric_cost"],
                "which_cloud": [0],
                "cloud_weights": [2.0],
                "max_costs": [float("nan")],
                "default_costs": [0.5],
                "neighborhood": 8,
                # Re-planning is off by default (planning_freq launch arg):
                # every measured cycle is then driven by a get_plan request, so
                # client latency and the node-side summary line describe the
                # same cycle.
                "num_input_clouds": 1,
                "input_queue_size": 2,
                "start_on_request": False,
                "stop_on_goal": False,
                "goal_reached_dist": 0.5,
                "mode": 2,
                "adhoc_costs": ["sidelobes"],
                "adhoc_layer": 3,
                "sidelobes_offset_distance": 1.0,
                "sidelobes_radius": 0.8,
                "sidelobes_cost": 10.0,
                "sidelobes_angle_offsets": [-90.0, 90.0, 180.0],
            },
            {
                "cell_size": ParameterValue(cfg["cell_size"], value_type=float),
                "neighborhood": ParameterValue(
                    cfg["neighborhood"], value_type=int
                ),
                "planning_freq": ParameterValue(
                    cfg["planning_freq"], value_type=float
                ),
                "tf_timeout": ParameterValue(
                    cfg["tf_timeout"], value_type=float
                ),
                "cloud_tf_timeout": ParameterValue(
                    cfg["cloud_tf_timeout"], value_type=float
                ),
                "input_range": ParameterValue(
                    cfg["input_range"], value_type=float
                ),
                "map_range": ParameterValue(cfg["map_range"], value_type=float),
                "evict_period": ParameterValue(
                    cfg["evict_period"], value_type=float
                ),
                "use_astar": ParameterValue(cfg["use_astar"], value_type=bool),
                "astar_max_range": ParameterValue(
                    cfg["astar_max_range"], value_type=float
                ),
            },
        ],
    )

    bench = ExecuteProcess(
        cmd=[
            sys.executable,
            LaunchConfiguration("bench_script"),
            "--ros-args",
            "-p",
            ["field_size:=", cfg["field_size"]],
            "-p",
            ["point_spacing:=", cfg["point_spacing"]],
            "-p",
            ["num_obstacles:=", cfg["num_obstacles"]],
            "-p",
            ["obstacle_size:=", cfg["obstacle_size"]],
            "-p",
            ["cloud_rate:=", cfg["cloud_rate"]],
            "-p",
            ["grow:=", cfg["grow"]],
            "-p",
            ["grow_start_size:=", cfg["grow_start_size"]],
            "-p",
            ["request_rate:=", cfg["request_rate"]],
            "-p",
            ["goal_x:=", cfg["goal_x"]],
            "-p",
            ["goal_y:=", cfg["goal_y"]],
            "-p",
            ["duration:=", cfg["duration"]],
            "-p",
            ["warmup:=", cfg["warmup"]],
            "-p",
            ["subscribe_map:=", cfg["subscribe_map"]],
            "-p",
            ["tf_gap:=", cfg["tf_gap"]],
            "-p",
            ["tf_gap_at:=", cfg["tf_gap_at"]],
            "-p",
            ["output:=", cfg["output"]],
        ],
        output="screen",
    )

    # Shut the whole launch down (and with it grid_planner) as soon as the
    # benchmark has printed its result, so the run is unattended.
    stop = RegisterEventHandler(
        OnProcessExit(
            target_action=bench,
            on_exit=[EmitEvent(event=Shutdown(reason="benchmark finished"))],
        )
    )

    return LaunchDescription(args + [planner, bench, stop])
