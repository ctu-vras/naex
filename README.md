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
| `input_range` | double | 10.0 |
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
`input_queue_size`, `points_min_dist`, `filter_robots`.

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
`launch/planner.launch.py` and `launch/mapper.launch.py` document working values.

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
- `lidar_inertial_odom.py` — work-in-progress lidar-inertial odometry; the
  projective ICP is still a stub. Needs `torch` (optional dependency of this
  package) and `ros2_numpy`.
- `mock_map_publisher.py`, `mock_tf_publisher.py`, `test_planner_client.py` —
  fixtures for `launch/test_planner.launch.py`.

## Usage

Launch the grid planner:

    ros2 launch naex grid_planner.launch.py

Run the grid planner against mock data:

    ros2 launch naex test_planner.launch.py

Launch the point-map planner (assumes localization in the `subt` frame):

    ros2 launch naex planner.launch.py robot:=X1 robot_type:=explorer_x1

Launch the follower:

    ros2 launch naex follower.launch.py robot:=X1 robot_type:=explorer_x1

Launch the whole stack (preprocessing, SLAM, planner, follower, recording):

    ros2 launch naex naex.launch.py

Play recorded bags from SubT virtual robots X1, X2, X3 in the current directory:

    bags=$(ls $(pwd)/*.mcap) ros2 launch naex playback.launch.py rate:=10.0

Every launch file takes a `use_sim_time` argument; run
`ros2 launch naex <file>.launch.py --show-args` for the rest.

### Launch files depending on unported packages

`odom.launch.py`, `slam.launch.py` and the `dynamic_mapper` part of
`husky.launch.py` still reference ROS 1-only packages (`subt_virtual`,
`robot_pose_ekf`, `ethzasl_icp_mapper`). The configuration is preserved and the
gaps are marked with `TODO(ros2)` comments in the files; those nodes cannot be
started until a ROS 2 equivalent is chosen. `preproc.launch.py` and
`husky.launch.py` use `pcl_ros` filter nodes in place of the ROS 1 PCL nodelets.

The libpointmatcher configurations in `launch/dynamic_mapper/` are not
ROS-version specific and were kept as they are. The old rviz1 configurations
were moved untouched to `launch/legacy_rviz1/`; they cannot be loaded by rviz2.
The rviz2 configuration used by `test_planner.launch.py` is
`config/grid_planner_test.rviz`.
