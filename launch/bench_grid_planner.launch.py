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
caps the per-cloud work).  ``input_range`` defaults to ``map_range``, so the
run below crops the input to 30 m as well; pass ``input_range:=0.0``
explicitly to reproduce the uncropped (and much more expensive) pre-2026-09-12
behaviour::

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
    # 0/0 means "opposite corner of the field".  The synthetic field used to
    # be lattice-aligned, which left whole cell rows empty when point_spacing
    # equalled cell_size (floor() of an exact multiple lands one cell low for
    # some indices) and put the far corner in a different connected component
    # than the robot; bench_grid_planner.py now offsets the points by half a
    # spacing, so the far corner is reachable and the default goal exercises a
    # full-length path.
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
    # P6.  map_range bounds the grid itself and defaults to 0 (disabled), so
    # every configuration recorded before P6 reproduces.  input_range crops the
    # cloud around the sensor and *defaults to map_range* (see the
    # DeclareLaunchArgument below): 0 while the map is unbounded, so the
    # 200 m static run is unchanged, but 30 m in the documented
    # "map_range:=30.0" growth run, which is the setting production uses.
    # Ingesting past map_range only re-creates the cells the next eviction
    # drops -- the 2026-09-12 profile measured ~100 k such cells per cycle and
    # twice the CPU for a map of the same size -- and the node clamps the crop
    # it applies (with one warning) if a configuration still asks for it.
    "map_range": "0.0",
    # Placeholder: the real default is a substitution, see below.  The entry
    # is kept so that "cfg" below still has an input_range key.
    "input_range": "",
    "evict_period": "10.0",
    # 0 keeps every measured cycle request-driven; > 0 also runs the planning
    # timer, which is what publishes the "path" topic the tf_gap run measures.
    "planning_freq": "0.0",
    # tf_timeout is deprecated and unused; kept so that the benchmark still
    # accepts it on the command line.  cloud_tf_timeout has to cover one TF
    # period (the replayed drops were 1-6 ms of extrapolation into the future
    # against a 10 Hz TF), request_tf_timeout bounds the get_plan path.
    "tf_timeout": "3.0",
    "cloud_tf_timeout": "0.2",
    "request_tf_timeout": "0.5",
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
        if name != "input_range"
    ]
    # Declared after map_range (and hence not from the loop above) because its
    # default *is* map_range: a bounded map must not ingest past its own bound.
    args.append(
        DeclareLaunchArgument(
            "input_range", default_value=LaunchConfiguration("map_range")
        )
    )
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
                "request_tf_timeout": ParameterValue(
                    cfg["request_tf_timeout"], value_type=float
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
