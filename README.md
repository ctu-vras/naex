# Naex

A package for robot navigation and exploration (ROS 2).
It is a work-in-progress, so this description is incomplete and may be slightly
out of date.

## Nodes

### grid_planner

The `grid_planner` node maintains a multi-layer 2D cost grid built from input
point clouds and plans paths on it. It is the node used in production.

It provides the [nav_msgs/srv/GetPlan](https://docs.ros2.org/latest/api/nav_msgs/srv/GetPlan.html)
service `get_plan`. Both `start` and `goal` poses may be NaN: an invalid start
means "plan from the current robot position", an invalid goal invokes the
exploration behaviour. The last request is (by default) repeated periodically at
`planning_freq` and the updated plan is published.

#### Parameters

| Name | Type | Default |
|---|---|---|
| `position_field` | string | `x` |
| `cost_fields` | string[] | `[]` |
| `which_cloud` | int[] | `[]` |
| `cloud_weights` | double[] | `[]` |
| `cloud_levels` | int[] | `[]` (cloud *i* into cost layer *i*) |
| `map_frame` | string | `map` |
| `robot_frame` | string | `base_footprint` |
| `tf_timeout` | double | 3.0 (robot pose lookup in `plan()`) |
| `cloud_tf_timeout` | double | 0.05 (per-cloud lookup; late clouds are dropped) |
| `max_cloud_age` | double | 5.0 |
| `input_range` | double | 10.0 (crop of the input cloud; <= 0 disables) |
| `map_range` | double | 0.0 (bound of the map; 0 = unbounded) |
| `evict_period` | double | 10.0 (max seconds between two evictions) |
| `cell_size` | double | 1.0 |
| `forget_factor` | double | 1.0 |
| `neighborhood` | int | 8 (4 or 8) |
| `num_input_clouds` | int | 1 |
| `input_queue_size` | int | 2 |
| `sensor_data_qos` | bool | false (true: best-effort input subscriptions) |
| `max_costs` | double[] | NaN per cost field / input cloud |
| `default_costs` | double[] | 1.0 per cost field / input cloud |
| `planning_freq` | double | 1.0 (Hz; <= 0 disables re-planning) |
| `start_on_request` | bool | true |
| `stop_on_goal` | bool | true |
| `goal_reached_dist` | double | NaN |
| `mode` | int | 2 |
| `adhoc_costs` | string[] | `[]` (e.g. `["sidelobes"]`) |
| `adhoc_layer` | int | 3 |
| `sidelobes_offset_distance` | double | 1.0 |
| `sidelobes_radius` | double | 0.5 |
| `sidelobes_cost` | double | 10.0 |
| `sidelobes_angle_offsets` | double[] | `[-90.0, -90.0]` (deg) |

Parameter types are strict: every floating-point parameter is a `double`
(`tf_timeout: 3` is rejected, use `3.0`) and every integer one an `int`.

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
  churn.

The map bound is a radius crop and not an age-based eviction on purpose: the
map is then a function of *where* the robot is, not of *when* it was somewhere,
so re-running the same trajectory at a different speed gives the same plan.

The node spins on a single-threaded executor, so a blocking transform lookup in
the cloud callback stalls the planning timer and the `get_plan` service as well.
`cloud_tf_timeout` is therefore deliberately short: a cloud whose transform is
not available within it is dropped (with a warning throttled to one per 2 s)
instead of parked on. Raise it only if the log shows clouds being dropped while
TF is healthy. `tf_timeout` is unrelated and still governs the single robot pose
lookup per planning cycle.

#### Subscribed topics

- `input_cloud_0` … `input_cloud_<num_input_clouds - 1>`
  [[sensor_msgs/msg/PointCloud2](https://docs.ros2.org/latest/api/sensor_msgs/msg/PointCloud2.html)]

#### Published topics

- `map` [sensor_msgs/msg/PointCloud2] — the whole cost grid. Visualisation only,
  and **only published while the topic has at least one subscriber**: building it
  costs 20 B per cell every planning cycle (4.3 MB at 216 k cells). A subscriber
  that joins late (rviz, or a `ros2 bag record` started after the node) misses
  the cycles before it connected.
- `path` [[nav_msgs/msg/Path](https://docs.ros2.org/latest/api/nav_msgs/msg/Path.html)] — planned path.
- `planning_freq` [std_msgs/msg/Float32] — the frequency the planner replans at.

#### Services

- `get_plan` [nav_msgs/srv/GetPlan]
- `clear_plan_map` [[nav2_msgs/srv/ClearEntireCostmap](https://github.com/ros-navigation/navigation2/blob/main/nav2_msgs/srv/ClearEntireCostmap.srv)]
  — drops the accumulated grid.

### planner

The `planner` node internally builds a **point** map (not a grid) from input
point clouds to assess traversability and plan paths globally. It provides the
same `get_plan` service; `start` and `goal` may be NaN with the same meaning.

If `goal` is not provided, an exploration strategy selects it, maximizing
reward/cost ratio. The reward captures visiting points from close-enough
distance and prefers frontier points. Visiting points with the current robot,
as opposed to other robots, may be preferred (`self_factor` > 0).
Positions of all robots are considered in assessing whether a point has been
observed. The node assumes an external localization is provided.

#### Parameters

Input and frames: `position_name`, `normal_name`, `map_frame`, `robot_frame`,
`robot_frames`, `max_cloud_age`, `input_range`, `num_input_clouds`,
`input_queue_size`, `points_min_dist`.

Traversability: `max_pitch`, `max_roll`, `inclination_penalty`,
`neighborhood_knn`, `neighborhood_radius`, `normal_radius`,
`max_nn_height_diff`, `min_points_obstacle`, `max_ground_diff_std`,
`max_mean_abs_ground_diff`, `edge_min_centroid_offset`, `min_dist_to_obstacle`,
`clearance_radius`, `clearance_low`, `clearance_high`.

Occupancy: `min_num_empty`, `min_empty_ratio`, `max_occ_counter`,
`min_empty_cos`.

Rewards and planning: `viewpoints_update_freq`, `min_vp_distance`,
`max_vp_distance`, `collect_rewards`, `full_coverage_dist`,
`coverage_dist_spread`, `self_factor`, `suppress_base_reward`, `path_cost_pow`,
`min_path_cost`, `planning_freq`, `random_start`, `plan_from_goal_dist`,
`bootstrap_z`.

`min_points_obstacle` is a floating-point parameter despite its name.
`launch/planner.launch.py` documents working values; `planning_freq: 0.0`
turns the node into a mapper.

#### Subscribed topics

- `input_cloud_0`, `input_cloud_1`, … [sensor_msgs/msg/PointCloud2]
- `input_map` [sensor_msgs/msg/PointCloud2]

#### Published topics

- `viewpoints`, `other_viewpoints` [sensor_msgs/msg/PointCloud2] — viewpoints
  considered in rewards, for this robot and the others.
- `map` [sensor_msgs/msg/PointCloud2] — complete map used for planning; see the
  `flags` bit field for point labels (the enum is in `include/naex/types.h`).
- `updated_map`, `map_diff` [sensor_msgs/msg/PointCloud2] — map deltas.
- `dirty_map` [sensor_msgs/msg/PointCloud2] — points queued for update.
- `local_map` [sensor_msgs/msg/PointCloud2] — local map around the robot.
- `path` [nav_msgs/msg/Path] — planned path.

#### Services

- `get_plan` [nav_msgs/srv/GetPlan]

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
| `use_path_theta` | string | `last` (`none`, `last`, `all`) |
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
  `tolerance` (32.0), `verbose` (false).
- `mock_map_publisher.py`, `mock_tf_publisher.py`, `test_planner_client.py` —
  fixtures for `launch/test_planner.launch.py`.

## Usage

Launch the grid planner:

    ros2 launch naex grid_planner.launch.py

Run the grid planner against mock data:

    ros2 launch naex test_planner.launch.py

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
