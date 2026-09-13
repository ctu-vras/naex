# Naex

A package for robot navigation and exploration (ROS 2).
It is a work-in-progress, so this description is incomplete and may be slightly
out of date.

## Nodes

### grid_planner

The `grid_planner` node maintains a multi-layer 2D cost grid built from input
point clouds and plans paths on it. It is the node used in production.

It provides the [nav_msgs/srv/GetPlan](https://docs.ros2.org/latest/api/nav_msgs/srv/GetPlan.html)
service `get_plan`. The `start` pose may be NaN, in which case the plan starts
from the current robot position; a NaN `goal` fails the request (there is no
exploration mode). Poses in a frame other than `map_frame` are transformed
into it. The last request is (by default) repeated periodically at
`planning_freq` and the updated plan is published.

#### Which cells the search runs between

The start cell (`vs` below is the start pose converted to a cell):

```
IF vs is explored:
  IF vs is traversable:              plan from vs
  ELSE IF the nearest traversable cell is within max_start_to_traversable_dist:
                                     plan from that cell
  ELSE:                              fail
ELSE (vs is unexplored):
  IF the nearest traversable cell is within max_start_to_traversable_dist:
                                     plan from that cell
  ELSE:                              answer with a straight line to the goal
```

The goal cell depends on the search:

- **Dijkstra** (`use_astar: false`) solves the whole grid from the start and
  ends the path at the reachable cell closest to the goal pose.
- **A\*** (`use_astar: true`) searches toward the goal pose only, and expands
  no cell farther than `astar_max_range` from the start cell. Then:
  - goal reached and the route is no longer than
    `max_relative_dist_to_goal` times the straight-line distance → the goal
    cell is the target;
  - goal reached but the route is much longer than the straight line → the
    cheapest frontier is considered instead, and taken if it gets closer to
    the goal than the goal cell itself does. This is the "drive the whole
    explored loop backwards instead of pushing through the unexplored" case;
  - goal cell exists but is an obstacle → the reachable cell closest to the
    goal;
  - goal cell does not exist (unexplored) → the cheapest frontier.

A **frontier** is a cell with at most `frontier_max_neighbors` existing
neighbours inside the `astar_max_range` crop, i.e. a cell on the edge of what
has been mapped, and at least `frontier_min_dist` from the robot. Frontiers are
grouped into connected components; the component that gets nearest to the goal
wins and its cheapest (lowest A\* *f*) cell becomes the temporary goal. The
component is published on `map_frontiers` for rviz.

Note that boost's A\* leaves a cell it never reached at
`std::numeric_limits<float>::max()`, not at infinity, so "unreachable" is
tested as *f* > 1e9 and not with `isfinite` after an A\* run
(`ShortestPaths::kUnreachableCost`).

#### Parameters

| Name | Type | Default |
|---|---|---|
| `position_field` | string | `x` |
| `cost_fields` | string[] | `[]` |
| `which_cloud` | int[] | `[]` |
| `cloud_weights` | double[] | `[]` |
| `cloud_levels` | int[] | `[]` (cloud *i* into cost layer *i*) |
| `min_cloud_values` | double[] | `-inf` per cost field (no filtering) |
| `inflation_radius` | double[] | `0.0` per cost field (m; disabled) |
| `map_frame` | string | `map` |
| `robot_frame` | string | `base_footprint` |
| `request_tf_timeout` | double | 0.5 (every TF lookup on the `get_plan` path) |
| `cloud_tf_timeout` | double | 0.2 (per-cloud lookup; late clouds are dropped) |
| `max_cloud_age` | double | 5.0 |
| `input_range` | double | 10.0 (crop of the input cloud; <= 0 disables; clamped to `map_range` when the map is bounded) |
| `map_range` | double | 0.0 (bound of the map; 0 = unbounded) |
| `evict_period` | double | 10.0 (max seconds between two evictions) |
| `cell_size` | double | 1.0 |
| `forget_factor` | double | 1.0 |
| `neighborhood` | int | 8 (4 or 8) |
| `num_input_clouds` | int | 1 |
| `input_queue_size` | int | 2 |
| `sensor_data_qos` | bool | false (true: best-effort input subscriptions) |
| `max_costs` | double[] | NaN per cost field / input cloud (absolute bound) |
| `max_costs_relative` | double[] | same as `max_costs`; a finite entry overrides it as `relative * cloud_weights[i]` |
| `default_costs` | double[] | 1.0 per cost field / input cloud |
| `use_astar` | bool | false (false: Dijkstra over the whole grid) |
| `astar_max_range` | double | 50.0 m |
| `frontier_min_dist` | double | 3.0 m |
| `frontier_max_neighbors` | int | 5 |
| `max_relative_dist_to_goal` | double | 2.0 |
| `frontier_dist_from_goal_cost` | double | 1.0 (weight of a frontier's euclidean distance to the goal in its score; 1.0 reproduces the plain A\* *f* value, higher penalizes a frontier further from the goal more) |
| `goal_snap_radius` | double | 0.0 m (0 disables goal snapping) |
| `goal_snap_level` | int | 0 (cost layer goal snapping checks) |
| `goal_snap_max_cost` | double | 0.0 (max `goal_snap_level` cost, weighted, of a cell the goal may snap to) |
| `max_start_to_traversable_dist` | double | 2.0 m |
| `append_goal_pose` | bool | true (append the requested goal as the last pose of a searched path; the straight-line fallback always has exactly two poses regardless) |
| `publish_occupancy_grid` | bool | true |
| `occupancy_grid_w`, `occupancy_grid_h` | int | 500 (cells; resolution is `cell_size`) |
| `planning_freq` | double | 1.0 (Hz; <= 0 disables re-planning) |
| `start_on_request` | bool | true |
| `stop_on_goal` | bool | true |
| `goal_reached_dist` | double | NaN |
| `adhoc_costs` | string[] | `[]` (e.g. `["sidelobes"]`) |
| `adhoc_layer` | int | 3 |
| `sidelobes_offset_distance` | double | 1.0 |
| `sidelobes_radius` | double | 0.5 |
| `sidelobes_cost` | double | 10.0 |
| `sidelobes_angle_offsets` | double[] | `[-90.0, -90.0]` (deg) |

Parameter types are strict: every floating-point parameter is a `double`
(`cell_size: 1` is rejected, use `1.0`) and every integer one an `int`.

##### Cost bounds: `max_costs` and `max_costs_relative`

A cell is traversable when every *bounded* cost layer is within its maximum. A
non-finite maximum means "this layer is not bounded" and is skipped, so
`[inf, 0.8, NaN, NaN]` bounds layer 1 and nothing else. (Before the A\* branch
the check stopped at the first non-finite entry, which silently unbounded every
layer behind an unbounded one.)

`max_costs` is in the units the grid stores, i.e. already multiplied by
`cloud_weights`. `max_costs_relative` is in the units of the input cloud and is
multiplied by `cloud_weights[i]` for you; a finite entry there overrides
`max_costs` for that layer. `max_costs_relative` is also the parameter meant to
be changed at runtime as a recovery behaviour — the node watches it with a
parameter callback and the next planning cycle picks the new bound up, because
the per-cell cost cache the search uses is rebuilt on every search.

##### Segmentation input: `min_cloud_values` and `inflation_radius`

Both are parallel to `cost_fields` (and validated at start-up the same way as
`which_cloud`/`cloud_weights`: a non-empty vector of the wrong length is a
configuration error). They exist for a binary segmentation cloud, where a
cost field is 0 (traversable) almost everywhere and only the rare obstacle
point should touch the grid. `min_cloud_values[j]` gates *creation*: a point
only creates a new cell for cost field *j* when its value is strictly greater
than the threshold; a cell that already exists is still updated regardless
(so a later 0 does clear a previously-flagged cell, it just cannot seed one).
`inflation_radius[j]` (metres) additionally stamps an above-threshold point's
cost onto every *other* cell within that radius, creating them as needed,
through the same forget-factor blend `update_cell_cost` uses -- meant to
inflate a segmented obstacle so the planner keeps clear of its footprint, not
just its centre point. Both default to a no-op (`-inf`, `0.0`), so the
default ingestion path is unaffected and pays no extra hashing; when
`map_range` is set, inflation never creates a cell beyond it (it would be
evicted on the next cycle anyway).

##### Bounding the map: `input_range` and `map_range`

The two are different things and both default to being useful only when set
deliberately.

`input_range` crops the **input**: a point farther than `input_range` from the
sensor origin (the translation of the cloud-to-map transform) is discarded
before it can touch the grid, together with any point whose coordinates are not
finite or fall outside the int16 cell range the grid can address (+-13.1 km at
`cell_size` 0.4). It bounds the per-cloud work, and it is what keeps the map
from being extended by far-away, low-confidence measurements — but it does
**not** bound the map, because cells are never removed by it: driving 1 km with
`input_range: 5.0` still creates about 60 k cells.

`map_range` bounds the **map**. Cells farther than `map_range` from the robot
are dropped, which caps the grid at
`(2 * ceil(map_range / cell_size) + 1)^2` cells and, with it, the memory and the
planning time (Dijkstra is close to linear in the cell count). A cell costs
roughly 85 B of grid state, of which 32 B is the flat neighbour table the
search reads instead of hashing a cell coordinate per edge; at 216 k cells the
whole node holds about 66 MB. `0.0`, the
default, is the historical behaviour: the map grows for the whole mission and
the planner gets slower the longer the robot drives. Eviction is done during
cloud ingestion, at most once per cloud, and only when the robot has moved a
quarter of `map_range`, when `evict_period` seconds have passed, or when the
grid has grown to more than 1.5x what the bound retains; a compaction over
216 k cells takes about 0.4 ms, so the amortised cost is negligible. Each
eviction is reported as a throttled `perf evict:` log line with the cell count
before and after, and `perf plan:` repeats the configured `map_range` on every
line.

Consequences of a bounded map, which is why it is opt-in:

- **Global planning degrades.** A goal outside `map_range` is not in the map,
  so the planner falls back to the nearest reachable cell — in practice the
  boundary cell closest to the goal, i.e. "drive toward the goal as far as the
  map goes". That is the same degradation the planner already has for a goal in
  unmapped space, but with `map_range` it happens by design.
- **Detours can be forgotten.** If `map_range` is smaller than the largest
  obstacle the robot has to circumnavigate, the part of the detour it has
  already driven falls out of the map and the plan can oscillate. Keep
  `map_range` several times the largest obstacle scale of the mission site;
  `100.0` is a safe starting value at `cell_size` 0.4 (about 250 k cells),
  `50.0` (about 63 k) once the site is known.
- **Set `input_range` no larger than `map_range`.** Otherwise every cloud
  re-creates the cells the last eviction dropped, which is correct but pure
  churn. Since it is easy to get wrong and expensive when you do, the node
  **clamps the crop it applies** to `map_range` whenever `map_range > 0` and
  `input_range` is either disabled (`<= 0`, NaN) or larger than `map_range`,
  and says so with a single `RCLCPP_WARN` at start-up. The declared parameter
  is left alone — `ros2 param get /grid_planner input_range` still reports what
  you set — only the ingestion crop is bounded. Profiling the uncropped
  `map_range: 30.0` configuration measured ~100 k cells created and evicted per
  cycle, 48 % of the process in the cell hash plus 19 % in `malloc`/`free`, and
  twice the CPU of the cropped run for a map of exactly the same size.

The map bound is a radius crop and not an age-based eviction on purpose: the
map is then a function of *where* the robot is, not of *when* it was somewhere,
so re-running the same trajectory at a different speed gives the same plan.

The node spins on a single-threaded executor, so a blocking transform lookup in
*any* callback stalls the planning timer, the cloud ingestion and the `get_plan`
service alike. Both timeouts are therefore bounded well below a second.

`cloud_tf_timeout` is the per-cloud one: a cloud whose transform is not
available within it is dropped (with a warning throttled to one per 2 s)
instead of parked on. It still has to cover one TF period — with the default
0.05 every drop observed in bag replay was an "extrapolation into the future"
by 1–6 ms against a 10 Hz TF, i.e. a transform that would have been there one
period later — so the default is `0.2`, two periods of a 10 Hz TF. Raise it
further only if the log still shows clouds being dropped while TF is healthy.

`request_tf_timeout` bounds every TF lookup on the `get_plan`/`plan` path: the
start and goal frame transforms, the robot pose lookup that replaces a NaN
start, and the robot-frame check of the ad-hoc layer. All of them ask for the
*latest available* transform (the request poses are stamped with zero time, not
with `now()`, so the buffer is never asked for a transform it cannot have yet),
so the timeout can only elapse when TF is genuinely absent — and then failing
fast with a warning is what the caller wants, not a stalled node. The total
time spent in these lookups is the `tf=` field of `perf plan:`.

#### Subscribed topics

- `input_cloud_0` … `input_cloud_<num_input_clouds - 1>`
  [[sensor_msgs/msg/PointCloud2](https://docs.ros2.org/latest/api/sensor_msgs/msg/PointCloud2.html)]

#### Published topics

- `map` [sensor_msgs/msg/PointCloud2] — the whole cost grid. Visualisation only,
  and **only published while the topic has at least one subscriber**: building it
  costs 20 B per cell every planning cycle (4.3 MB at 216 k cells). Published
  with `transient_local` QoS (depth 1), so a subscriber that joins after the
  last publish still gets that one cycle instead of nothing.
- `path` [[nav_msgs/msg/Path](https://docs.ros2.org/latest/api/nav_msgs/msg/Path.html)] — planned path.
  The first pose is the start pose of the request; with `append_goal_pose`
  (the default) the last one is the requested goal, so a goal checker
  downstream sees the real goal even when the search itself stopped at a
  frontier. With `append_goal_pose: false` the path ends at the last searched
  cell instead.
- `planning_freq` [std_msgs/msg/Float32] — the frequency the planner replans at.
- `map_occupancy_grid` [[nav_msgs/msg/OccupancyGrid](https://docs.ros2.org/latest/api/nav_msgs/msg/OccupancyGrid.html)]
  — `occupancy_grid_w` x `occupancy_grid_h` cells of `cell_size`, centred on
  the robot: 0 where the grid is traversable, 127 where it is not, -1 where
  nothing has been seen. Meant for the nav2 global costmap; published once per
  ingested cloud when `publish_occupancy_grid` is true, plus one empty grid at
  start-up so the costmap initializes. Unlike `map`, it is **not** guarded by
  the subscriber count.
- `map_frontiers` [visualization_msgs/msg/MarkerArray] — the winning frontier
  component, rviz only.

#### Services

- `get_plan` [nav_msgs/srv/GetPlan]
- `clear_plan_map` [[nav2_msgs/srv/ClearEntireCostmap](https://docs.ros2.org/latest/api/nav2_msgs/srv/ClearEntireCostmap.html)]
  — drops the accumulated grid. Empty request and response (that message type
  says nothing back); this is the name and type the robots and upstream use:

  ```
  ros2 service call /clear_plan_map nav2_msgs/srv/ClearEntireCostmap
  ```

- `clear_plan_map_trigger` [[std_srvs/srv/Trigger](https://docs.ros2.org/latest/api/std_srvs/srv/Trigger.html)]
  — the same clear, for a caller that wants the cell count back. The response
  is always `success: true` with `message: "map cleared: N cells"`, N being
  the cell count before the clear:

  ```
  ros2 service call /clear_plan_map_trigger std_srvs/srv/Trigger
  ```

  Both services share one implementation; only the response differs.

#### Actions

- `compute_path_to_pose` [[nav2_msgs/action/ComputePathToPose](https://docs.ros2.org/latest/api/nav2_msgs/action/ComputePathToPose.html)]
  — a `nav2_util::SimpleActionServer`, spinning its execute callback on its
  own thread. The start pose is the current robot pose
  (`nav2_util::getCurrentPose(map_frame_, robot_frame_)`, timeout
  `request_tf_timeout`); the goal is transformed into `map_frame` with
  `nav2_util::transformPoseInTargetFrame` (same timeout) if it isn't already
  in it. Both the action and `get_plan` search with the same core planner;
  only `get_plan`/the periodic timer remember the request for re-planning,
  the action does not. A search failure or a TF problem is reported as the
  matching `ComputePathToPose::Result::error_code` (`NO_VALID_PATH`,
  `TF_ERROR`, ...) instead of an exception reaching the client. Because the
  action's execute callback runs on its own thread while everything else
  (clouds, `get_plan`, the periodic timer) runs on the node's single-threaded
  executor, a mutex (see "Concurrency" below) serializes them; an action goal
  and a `get_plan` request or a cloud can therefore never touch `grid_` at
  the same time, only queue behind each other.

##### Concurrency

One `std::mutex` is held for the whole duration of a plan (`plan_safe()`,
`compute_plan()`) and of one cloud ingestion (`receive_cloud_safe()`,
including eviction and the occupancy-grid publish) and of a map clear
(`clear_map_impl()`), each acquired only at that top-level entry point — the
grid is never copied per plan (it can be tens of MB). The ceiling this
implies: cloud ingestion (and a competing plan) waits while another plan
runs, typically milliseconds with A\*. If that ever becomes a problem, the
upgrade path is a grid snapshot/RCU instead of a shared mutex.

### planner

The `planner` node internally builds a **point** map (not a grid) from input
point clouds to assess traversability and plan paths globally. It provides the
same `get_plan` service; `start` and `goal` may be NaN with the same meaning.

If `goal` is not provided, an exploration strategy selects it, maximizing
reward/cost ratio. The reward captures visiting points from close-enough
distance and prefers frontier points. The node assumes an external
localization is provided.

#### Parameters

Input and frames: `position_name`, `normal_name`, `map_frame`, `robot_frame`,
`max_cloud_age`, `input_range`, `num_input_clouds`, `input_queue_size`,
`points_min_dist`.

Traversability: `max_pitch`, `max_roll`, `inclination_penalty`,
`neighborhood_radius`, `min_points_obstacle`, `max_ground_diff_std`,
`max_mean_abs_ground_diff`, `edge_min_centroid_offset`, `min_dist_to_obstacle`,
`clearance_radius`, `clearance_low`, `clearance_high`.

Occupancy: `min_num_empty`, `min_empty_ratio`, `max_occ_counter`,
`min_empty_cos`.

Rewards and planning: `viewpoints_update_freq`, `max_vp_distance`,
`full_coverage_dist`, `coverage_dist_spread`, `path_cost_pow`,
`min_path_cost`, `planning_freq`, `random_start`, `plan_from_goal_dist`.

`min_points_obstacle` is a floating-point parameter despite its name.
`launch/planner.launch.py` documents working values; `planning_freq: 0.0`
turns the node into a mapper.

#### Subscribed topics

- `input_cloud_0`, `input_cloud_1`, … [sensor_msgs/msg/PointCloud2]

#### Published topics

- `viewpoints` [sensor_msgs/msg/PointCloud2] — viewpoints considered in
  rewards.
- `map` [sensor_msgs/msg/PointCloud2] — complete map used for planning; see the
  `flags` bit field for point labels (the enum is in `include/naex/types.h`).
- `updated_map` [sensor_msgs/msg/PointCloud2] — map deltas.
- `dirty_map` [sensor_msgs/msg/PointCloud2] — points queued for update.
- `local_map` [sensor_msgs/msg/PointCloud2] — local map around the robot.
- `path` [nav_msgs/msg/Path] — planned path.

#### Services

- `get_plan` [nav_msgs/srv/GetPlan]

### mule_planner

A local re-planner that sits behind `grid_planner`: it subscribes to a single
traversability cloud (`points`) and to the global path (`path`), both in the
sensor frame, and republishes the path on `~/path` with the part that is about
to hit an obstacle replanned around it.

Each cloud rebuilds a one-scan grid (the grid is cleared first, so nothing is
remembered between scans) and:

1. if the incoming path is obstacle-free, it is republished as is;
2. otherwise A\* runs from the sensor origin toward the last pose of the path,
   the furthest reachable pose on the path becomes the goal, and the replanned
   prefix is published with the original suffix appended when that suffix is
   free;
3. if only a prefix is traversable it is published alone, unless it is shorter
   than `min_traversable_path_length`, in which case an empty path is
   published.

With `path_sampling_dist` > 0 the published path is resampled to that spacing.

| Name | Type | Default |
|---|---|---|
| `max_cloud_age` | double | 0.5 s |
| `max_ts_diff` | double | 0.5 s (path vs. cloud stamp) |
| `min_traversable_path_length` | double | 0.0 m |
| `position_field` | string | `x` |
| `cost_field` | string | `traversability` |
| `astar_max_range` | double | 50.0 m |
| `obstacle_cost_threshold` | double | 0.7 |
| `max_start_to_traversable_dist` | double | 5.0 m |
| `neighborhood` | int | 8 |
| `path_sampling_dist` | double | 0.0 (0 disables resampling) |
| `cell_size` | double | 1.0 |
| `forget_factor` | double | 1.0 |
| `default_costs` | double[] | `[0.5]` |

Subscribes `points` [sensor_msgs/msg/PointCloud2, sensor-data QoS] and `path`
[nav_msgs/msg/Path]; publishes `~/path` [nav_msgs/msg/Path] and `planner_grid`
[sensor_msgs/msg/PointCloud2, rviz only, subscriber-guarded].

### traversability_node

Estimates per-point traversability cost for incoming clouds and republishes the
annotated cloud. Node name `traversability`.

| Name | Type | Default |
|---|---|---|
| `min_z`, `max_z` | double | NaN (no limit) |
| `support_radius` | double | 0.25 |
| `min_support` | int | 3 |
| `inclination_radius` | double | 0.5 |
| `inclination_weight` | double | 1.0 |
| `normal_std_weight` | double | 1.0 |
| `clearance_radius` | double | 0.5 |
| `clearance_low` | double | 0.1 |
| `clearance_high` | double | 0.5 |
| `obstacle_weight` | double | 1.0 |
| `remove_low_support` | bool | false |
| `fixed_frame` | string | `` (empty: no TF lookup) |
| `timeout` | double | 0.1 |

Subscribes `input`, publishes `output`, both
[sensor_msgs/msg/PointCloud2](https://docs.ros2.org/latest/api/sensor_msgs/msg/PointCloud2.html).

### lidar_model

Fits a spherical projection model to incoming clouds and reports the estimated
elevation/azimuth ranges and steps. Subscribes `cloud`
[sensor_msgs/msg/PointCloud2]. Parameter `check_model` (bool, default false)
additionally verifies the fitted model against the cloud.

### follower

The `follower` node (`path_follower.py`) follows published paths. At each
control step the closest point on the path, with an optional look-ahead
distance, is selected as navigation goal. A new path is only accepted once the
previous one has been completed or held for `keep_path` seconds. It stops to
avoid collisions, which are assessed from input point clouds, and it can
backtrack over the traversed poses if no valid path is received for
`backtrack_after` seconds.

#### Parameters

| Name | Type | Default |
|---|---|---|
| `map_frame` | string | `map` |
| `odom_frame` | string | `odom` (no-wait frame) |
| `robot_frame` | string | `base_footprint` |
| `control_freq` | double | 10.0 Hz (must be in (1, 25)) |
| `local_goal_dims` | string | `xy` (or `xyz`) |
| `goal_reached_dist` | double | 0.2 m |
| `goal_reached_angle` | double | 0.2 rad |
| `max_age` | double | 1.0 s |
| `max_path_dist` | double[] | `[0.5]` m; the tail is consumed first by reached goals |
| `look_ahead` | double | 1.0 m |
| `p_angle`, `p_dist` | double | 1.0 |
| `max_speed` | double | 1.0 m/s |
| `max_accel` | double | 1.0 m/s² |
| `max_force_through_speed` | double | 0.25 m/s |
| `turn_on_spot_angle` | double | π/6 rad |
| `max_angular_rate` | double | 1.0 rad/s |
| `max_angular_accel` | double | 2.0 rad/s² |
| `max_roll`, `max_pitch` | double | 0.7 rad |
| `keep_path` | double | 30.0 s |
| `increasing_waypoint_index` | bool | true |
| `estimate_path_costs` | bool | false |
| `keep_cloud_box` | double[6] | `[-4, 4, -4, 4, -4, 4]` m |
| `clearance_box` | double[6] | `[-0.6, 0.6, -0.5, 0.5, 0.0, 0.8]` m |
| `show_clearance_pos` | int[2] | `[-10, 10]` waypoints around the current one |
| `min_points_obstacle` | int | 1 (< 1 disables the clearance check) |
| `force_through_after` | double | 15.0 s |
| `allow_backward` | bool | true |
| `backtrack_after` | double | 30.0 s |

ROS 2 parameters cannot be nested, so the boxes that were
`[[xmin, xmax], [ymin, ymax], [zmin, zmax]]` in ROS 1 are flat six-element
arrays `[xmin, xmax, ymin, ymax, zmin, zmax]`.

#### Subscribed topics

- `path` [nav_msgs/msg/Path] — path to follow (reliable).
- `cloud` [sensor_msgs/msg/PointCloud2] — cloud for the clearance check
  (best effort).

#### Published topics

- `cmd_vel` [[geometry_msgs/msg/Twist](https://docs.ros2.org/latest/api/geometry_msgs/msg/Twist.html)]
  — velocity command.
- `control_path` [nav_msgs/msg/Path] — the path actually being followed.
- `~/markers` [visualization_msgs/msg/MarkerArray] — path, waypoint and
  clearance visualization.

## Scripts

- `get_plan.py` — one-shot client of the `get_plan` service. Parameters:
  `start`, `goal` (three floats, or a `"x,y,z"` string; NaN by default),
  `tolerance` (32.0), `verbose` (false). A NaN goal invokes exploration /
  go-home only with the legacy point-map `planner` node; `grid_planner`
  rejects a NaN goal.
- `mock_map_publisher.py`, `mock_tf_publisher.py`, `test_planner_client.py` —
  fixtures for `launch/test_planner.launch.py`.

## Usage

Launch the grid planner:

    ros2 launch naex grid_planner.launch.py

Run the grid planner against mock data:

    ros2 launch naex test_planner.launch.py

Launch the local re-planner behind it:

    ros2 launch naex mule_planner.launch.py

Benchmark the grid planner headlessly (synthetic clouds, no display):

    ros2 launch naex bench_grid_planner.launch.py field_size:=200.0 duration:=30.0

Launch the point-map planner:

    ros2 launch naex planner.launch.py points:=points_slow

Launch the follower:

    ros2 launch naex follower.launch.py path:=path cmd_vel:=cmd_vel

Every launch file takes a `use_sim_time` argument; run
`ros2 launch naex <file>.launch.py --show-args` for the rest.

The rviz2 configuration used by `test_planner.launch.py` is
`config/grid_planner_test.rviz`.

### Removed with the SubT era

The launch files of the DARPA SubT stack (`naex.launch.py`, `preproc.launch.py`,
`odom.launch.py`, `slam.launch.py`, `tf.launch.py`, `footprint.launch.py`,
`record.launch.py`, `mapper.launch.py`, `husky.launch.py`, `playback.launch.py`,
`play_skoda.launch.py`), the rviz1 configurations in `launch/legacy_rviz1/`, the
libpointmatcher configurations in `launch/dynamic_mapper/` and
`scripts/lidar_inertial_odom.py` were deleted: they launch packages that have no
ROS 2 release (`subt_virtual`, `robot_pose_ekf`, `ethzasl_icp_mapper`,
`static_transform_mux`) or configure robots that no longer exist (x1, x2, dtr,
jeanine, marv, tradr, absolem, husky). The per-robot parameter blocks were
likewise dropped from `planner.launch.py` and `follower.launch.py`. Git history
has them if a robot ever comes back.
