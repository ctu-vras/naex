#!/usr/bin/env python3
"""Reproducible load generator and latency recorder for the grid planner.

Publishes a synthetic traversability cloud on ``input_cloud_0``, broadcasts the
map -> robot transform, and calls ``get_plan`` repeatedly with a far goal.  It
records the client-side round-trip latency of every request and, when it can
see ``/rosout``, the ``perf plan:`` summary line that
``naex::grid::Planner::logPlanSummary()`` emits once per planning cycle.

Everything is headless; no display, no rviz, no bag files are needed.

Typical use (see launch/bench_grid_planner.launch.py)::

    ros2 run naex bench_grid_planner.py --ros-args \
        -p field_size:=200.0 -p duration:=30.0

Parameters
----------
field_size        Edge length of the square traversability field, in metres.
point_spacing     Distance between synthetic points, in metres.
cost_field        Name of the cost field in the published cloud.
free_cost         Cost value of a free point.
obstacle_cost     Cost value of a point inside an obstacle patch.
num_obstacles     Number of random square obstacle patches.
obstacle_size     Edge length of one obstacle patch, in metres.
seed              RNG seed; the same seed gives the same field every run.
cloud_rate        Cloud publishing rate, in Hz.
grow              If true, the field grows linearly from ``grow_start_size``
                  to ``field_size`` over ``duration`` (emulates P6's unbounded
                  grid growth).
grow_start_size   Initial edge length in growth mode, in metres.
request_rate      Upper bound on the get_plan request rate, in Hz.  Requests
                  are sequential, so the achieved rate is at most this.
duration          Benchmark duration, in seconds.  0 means run forever.
warmup            Seconds of requests discarded before recording, so that the
                  first (grid-filling) cycles do not pollute the statistics.
map_frame         Frame the planner plans in.
robot_frame       Frame the cloud is published in and the robot sits in.
robot_x, robot_y  Robot position in the map frame.
goal_x, goal_y    Goal position in the map frame.
subscribe_map     If true, subscribe to the planner's rviz-only ``map`` cloud
                  so that its publish cost (P4) is actually paid.  Set false to
                  measure the planner with no map consumer attached.
tf_gap            If > 0, stop broadcasting ``map -> robot_frame`` for that many
                  seconds, starting at ``tf_gap_at`` of the run (P5 acceptance
                  test).  The report then carries the worst gap between
                  consecutive ``path`` messages and the client latency observed
                  inside the window.
tf_gap_at         Fraction of ``duration`` at which the TF outage starts.
output            Path to write the results to as JSON; "" or "none" disables.

The ``path`` topic is always subscribed: with ``planning_freq > 0`` on the
planner it is the only observable that shows the executor being parked, because
a stalled executor cannot run the planning timer either.
"""

import json
import math
import statistics
import sys
import threading
import time

import numpy as np
import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Path
from nav_msgs.srv import GetPlan
from rcl_interfaces.msg import Log
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
from tf2_ros import TransformBroadcaster


def _quantiles(values, qs):
    """Nearest-rank quantiles; robust for tiny samples, unlike statistics."""
    if not values:
        return {q: float("nan") for q in qs}
    ordered = sorted(values)
    out = {}
    for q in qs:
        idx = min(len(ordered) - 1, max(0, int(math.ceil(q * len(ordered))) - 1))
        out[q] = ordered[idx]
    return out


class BenchGridPlanner(Node):
    def __init__(self):
        super().__init__("bench_grid_planner")

        self.field_size = self._p("field_size", 200.0)
        self.point_spacing = self._p("point_spacing", 0.4)
        self.cost_field = self._p("cost_field", "geometric_cost")
        self.free_cost = self._p("free_cost", 0.1)
        self.obstacle_cost = self._p("obstacle_cost", 5.0)
        self.num_obstacles = self._p("num_obstacles", 40)
        self.obstacle_size = self._p("obstacle_size", 6.0)
        self.seed = self._p("seed", 42)
        self.cloud_rate = self._p("cloud_rate", 2.0)
        self.grow = self._p("grow", False)
        self.grow_start_size = self._p("grow_start_size", 20.0)
        self.request_rate = self._p("request_rate", 1.0)
        self.duration = self._p("duration", 30.0)
        self.warmup = self._p("warmup", 3.0)
        self.map_frame = self._p("map_frame", "map")
        self.robot_frame = self._p("robot_frame", "base_link")
        self.robot_x = self._p("robot_x", 0.0)
        self.robot_y = self._p("robot_y", 0.0)
        self.goal_x = self._p("goal_x", 0.0)
        self.goal_y = self._p("goal_y", 0.0)
        self.subscribe_map = self._p("subscribe_map", True)
        self.tf_gap = self._p("tf_gap", 0.0)
        self.tf_gap_at = self._p("tf_gap_at", 0.5)
        self.output = self._p("output", "")

        if self.goal_x == 0.0 and self.goal_y == 0.0:
            # Default goal: opposite corner of the field, just inside it.
            half = self.field_size / 2.0 - 2.0 * self.point_spacing
            self.goal_x = self.robot_x + half
            self.goal_y = self.robot_y + half

        self.group = ReentrantCallbackGroup()
        self.cloud_pub = self.create_publisher(PointCloud2, "input_cloud_0", 2)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.client = self.create_client(
            GetPlan, "get_plan", callback_group=self.group
        )
        self.create_subscription(
            Log, "/rosout", self.on_log, 100, callback_group=self.group
        )
        self.map_msgs = 0
        self.map_bytes = 0
        self.path_times = []
        self.create_subscription(
            Path, "path", self.on_path, 2, callback_group=self.group
        )
        if self.subscribe_map:
            self.create_subscription(
                PointCloud2, "map", self.on_map, 1, callback_group=self.group
            )

        self.rng = np.random.default_rng(int(self.seed))
        self.obstacles = self._make_obstacles()
        self.cloud_cache = {}
        self.start_time = time.monotonic()
        self.published_points = 0
        self.perf_lines = []
        self.cloud_lines = []

        self.create_timer(
            1.0 / max(self.cloud_rate, 0.01),
            self.publish_cloud,
            callback_group=self.group,
        )
        self.create_timer(0.05, self.publish_tf, callback_group=self.group)

        self.get_logger().info(
            "bench: field %.1f m at %.2f m, %d obstacles, goal (%.1f, %.1f), "
            "%.1f s"
            % (
                self.field_size,
                self.point_spacing,
                int(self.num_obstacles),
                self.goal_x,
                self.goal_y,
                self.duration,
            )
        )

    def _p(self, name, default):
        self.declare_parameter(name, default)
        return self.get_parameter(name).value

    # --- synthetic field ---------------------------------------------------

    def _make_obstacles(self):
        half = self.field_size / 2.0
        n = int(self.num_obstacles)
        if n <= 0:
            return np.zeros((0, 2))
        return self.rng.uniform(-half, half, size=(n, 2))

    def _current_size(self):
        if not self.grow or self.duration <= 0.0:
            return self.field_size
        frac = min(1.0, (time.monotonic() - self.start_time) / self.duration)
        return self.grow_start_size + frac * (self.field_size - self.grow_start_size)

    def _build_points(self, size):
        """Structured x/y/z/cost array for a square field of the given size."""
        key = round(size, 2)
        if key in self.cloud_cache:
            return self.cloud_cache[key]
        half = size / 2.0
        n = max(2, int(size / self.point_spacing) + 1)
        axis = np.linspace(-half, half, n, dtype=np.float32)
        xs, ys = np.meshgrid(axis, axis, indexing="ij")
        xs = xs.ravel()
        ys = ys.ravel()
        cost = np.full(xs.shape, float(self.free_cost), dtype=np.float32)
        r = self.obstacle_size / 2.0
        for ox, oy in self.obstacles:
            if abs(ox) > half + r or abs(oy) > half + r:
                continue
            inside = (np.abs(xs - ox) <= r) & (np.abs(ys - oy) <= r)
            cost[inside] = float(self.obstacle_cost)
        # Keep the robot cell and the goal cell free so a plan always exists.
        for px, py in ((0.0, 0.0), (self.goal_x - self.robot_x, self.goal_y - self.robot_y)):
            near = (np.abs(xs - px) <= 1.0) & (np.abs(ys - py) <= 1.0)
            cost[near] = float(self.free_cost)

        data = np.zeros(
            xs.shape[0],
            dtype=[("x", "<f4"), ("y", "<f4"), ("z", "<f4"), ("c", "<f4")],
        )
        data["x"] = xs
        data["y"] = ys
        data["z"] = 0.0
        data["c"] = cost
        raw = data.tobytes()
        # Cache at most a handful of sizes so growth mode stays bounded.
        if len(self.cloud_cache) > 64:
            self.cloud_cache.clear()
        self.cloud_cache[key] = (raw, xs.shape[0])
        return self.cloud_cache[key]

    def publish_cloud(self):
        raw, count = self._build_points(self._current_size())
        msg = PointCloud2()
        msg.header.frame_id = self.robot_frame
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.height = 1
        msg.width = count
        msg.fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(
                name=self.cost_field,
                offset=12,
                datatype=PointField.FLOAT32,
                count=1,
            ),
        ]
        msg.is_bigendian = False
        msg.point_step = 16
        msg.row_step = 16 * count
        msg.data = raw
        msg.is_dense = True
        self.cloud_pub.publish(msg)
        self.published_points = count

    def tf_gap_window(self):
        """(start, end) of the TF outage in monotonic time, or None."""
        if not (self.tf_gap > 0.0) or self.duration <= 0.0:
            return None
        start = self.start_time + self.tf_gap_at * self.duration
        return (start, start + self.tf_gap)

    def in_tf_gap(self, when=None):
        window = self.tf_gap_window()
        if window is None:
            return False
        when = time.monotonic() if when is None else when
        return window[0] <= when < window[1]

    def publish_tf(self):
        if self.in_tf_gap():
            # P5 acceptance: the planner must drop clouds it cannot transform
            # instead of parking the single-threaded executor on the lookup.
            return
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = self.map_frame
        t.child_frame_id = self.robot_frame
        t.transform.translation.x = float(self.robot_x)
        t.transform.translation.y = float(self.robot_y)
        t.transform.translation.z = 0.0
        t.transform.rotation.w = 1.0
        self.tf_broadcaster.sendTransform(t)

    def on_map(self, msg):
        self.map_msgs += 1
        self.map_bytes += len(msg.data)

    def on_path(self, msg):
        del msg
        self.path_times.append(time.monotonic())

    # --- node log scraping -------------------------------------------------

    @staticmethod
    def _parse_kv(text):
        entry = {}
        for token in text.split():
            if "=" not in token:
                continue
            k, v = token.split("=", 1)
            try:
                entry[k] = float(v)
            except ValueError:
                pass
        return entry

    def on_log(self, msg):
        text = msg.msg
        if text.startswith("perf plan:"):
            entry = self._parse_kv(text[len("perf plan:") :])
            if entry:
                self.perf_lines.append(entry)
        elif text.startswith("perf cloud["):
            # "perf cloud[0]: pts=... tf=... points=... cells=..."
            _, _, rest = text.partition(":")
            entry = self._parse_kv(rest)
            if entry:
                self.cloud_lines.append(entry)

    # --- request loop ------------------------------------------------------

    def make_request(self):
        req = GetPlan.Request()
        req.start.header.frame_id = self.map_frame
        req.start.pose.position.x = float(self.robot_x)
        req.start.pose.position.y = float(self.robot_y)
        req.start.pose.position.z = 0.0
        req.start.pose.orientation.w = 1.0
        req.goal.header.frame_id = self.map_frame
        req.goal.pose.position.x = float(self.goal_x)
        req.goal.pose.position.y = float(self.goal_y)
        req.goal.pose.position.z = 0.0
        req.goal.pose.orientation.w = 1.0
        req.tolerance = 2.0
        return req

    def run(self):
        if not self.client.wait_for_service(timeout_sec=30.0):
            self.get_logger().error("get_plan service not available")
            return 1
        # Let a couple of clouds land before the first request.
        time.sleep(2.0 / max(self.cloud_rate, 0.01))

        latencies = []
        req_starts = []
        poses = []
        period = 1.0 / max(self.request_rate, 0.01)
        t_end = self.start_time + self.duration if self.duration > 0 else None
        n_total = 0
        while rclpy.ok():
            now = time.monotonic()
            if t_end is not None and now >= t_end:
                break
            t0 = time.monotonic()
            future = self.client.call_async(self.make_request())
            while rclpy.ok() and not future.done():
                if time.monotonic() - t0 > 60.0:
                    self.get_logger().error("get_plan timed out after 60 s")
                    break
                time.sleep(0.002)
            if not future.done():
                break
            dt = time.monotonic() - t0
            n_total += 1
            recording = (time.monotonic() - self.start_time) >= self.warmup
            if recording:
                latencies.append(dt)
                req_starts.append(t0)
                res = future.result()
                poses.append(len(res.plan.poses) if res is not None else 0)
            sleep = period - (time.monotonic() - t0)
            if sleep > 0:
                time.sleep(sleep)

        self.report(latencies, poses, n_total, req_starts)
        return 0

    def tf_gap_report(self, latencies, req_starts):
        """Worst path-message gap and worst client latency in the TF outage.

        ``path`` messages are the planning timer's output, so the gap between
        two of them is a direct measure of how long the single-threaded
        executor was unavailable.  Two windows are reported: ``_in_tf_gap_s``
        counts only what overlaps the outage itself, ``_from_tf_gap_s`` counts
        everything from the start of the outage to the end of the run, which is
        where the recovery cost shows up (clouds stamped inside the outage
        still have no transform once broadcasting resumes, so each one costs a
        full timeout).
        """
        window = self.tf_gap_window()
        gaps = [
            (b - a, a) for a, b in zip(self.path_times, self.path_times[1:])
        ]
        out = {
            "tf_gap_s": float(self.tf_gap),
            "path_msgs": len(self.path_times),
            "path_gap_max_s": max((g for g, _ in gaps), default=float("nan")),
            "path_gap_max_in_tf_gap_s": float("nan"),
            "path_gap_max_from_tf_gap_s": float("nan"),
            "latency_max_in_tf_gap_s": float("nan"),
            "latency_max_from_tf_gap_s": float("nan"),
            "requests_in_tf_gap": 0,
        }
        if window is None:
            return out
        start, end = window
        out["tf_gap_start_s"] = start - self.start_time
        out["tf_gap_end_s"] = end - self.start_time
        # A gap counts if the interval it spans overlaps the outage window.
        in_gap = [g for g, a in gaps if a < end and (a + g) > start]
        if in_gap:
            out["path_gap_max_in_tf_gap_s"] = max(in_gap)
        after = [g for g, a in gaps if (a + g) > start]
        if after:
            out["path_gap_max_from_tf_gap_s"] = max(after)
        lat = [
            d
            for d, t0 in zip(latencies, req_starts)
            if t0 < end and (t0 + d) > start
        ]
        out["requests_in_tf_gap"] = len(lat)
        if lat:
            out["latency_max_in_tf_gap_s"] = max(lat)
        lat_after = [d for d, t0 in zip(latencies, req_starts) if (t0 + d) > start]
        if lat_after:
            out["latency_max_from_tf_gap_s"] = max(lat_after)
        return out

    def report(self, latencies, poses, n_total, req_starts=None):
        req_starts = req_starts if req_starts is not None else []
        qs = _quantiles(latencies, (0.5, 0.9, 1.0))
        perf_keys = (
            "cells",
            "tf",
            "adhoc",
            "dijkstra",
            "map_cloud",
            "scan_trav",
            "scan_reach",
            "total",
        )
        # Skip the same warmup prefix on the node-side lines as on the client
        # side: the first lines come from grid-filling cycles.
        perf = self.perf_lines[1:] if len(self.perf_lines) > 1 else self.perf_lines
        perf_mean = {}
        perf_max = {}
        for k in perf_keys:
            vals = [e[k] for e in perf if k in e]
            perf_mean[k] = statistics.fmean(vals) if vals else float("nan")
            perf_max[k] = max(vals) if vals else float("nan")

        cloud_keys = ("pts", "tf", "points", "cells")
        cloud = self.cloud_lines[1:] if len(self.cloud_lines) > 1 else self.cloud_lines
        cloud_mean = {}
        cloud_max = {}
        for k in cloud_keys:
            vals = [e[k] for e in cloud if k in e]
            cloud_mean[k] = statistics.fmean(vals) if vals else float("nan")
            cloud_max[k] = max(vals) if vals else float("nan")

        result = {
            "field_size": self.field_size,
            "point_spacing": self.point_spacing,
            "grow": bool(self.grow),
            "cloud_points": int(self.published_points),
            "requests_total": n_total,
            "requests_recorded": len(latencies),
            "latency_mean_s": statistics.fmean(latencies) if latencies else float("nan"),
            "latency_p50_s": qs[0.5],
            "latency_p90_s": qs[0.9],
            "latency_max_s": qs[1.0],
            "path_poses_mean": statistics.fmean(poses) if poses else float("nan"),
            "subscribe_map": bool(self.subscribe_map),
            "map_msgs": self.map_msgs,
            "map_bytes": self.map_bytes,
            "node_perf_lines": len(self.perf_lines),
            "node_cloud_lines": len(self.cloud_lines),
            "cloud_mean": cloud_mean,
            "cloud_max": cloud_max,
            "node_mean": perf_mean,
            "node_max": perf_max,
            "tf_gap": self.tf_gap_report(latencies, req_starts),
        }
        text = json.dumps(result, indent=2, sort_keys=True)
        self.get_logger().info("bench result:\n" + text)
        print("BENCH_RESULT " + json.dumps(result, sort_keys=True), flush=True)
        if self.output and self.output != "none":
            with open(self.output, "w") as f:
                f.write(text + "\n")


def main(args=None):
    rclpy.init(args=args)
    node = BenchGridPlanner()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()
    rc = 0
    try:
        rc = node.run()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        executor.shutdown(timeout_sec=1.0)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return rc


if __name__ == "__main__":
    sys.exit(main())
