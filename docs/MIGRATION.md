# Migration from the current TF publishers

Migration is edge-based. Never enable the new authority while an old process
still publishes the same child frame.

## Current publishers to retire or reconfigure

| Current component | Current edge/output | Required migration |
|---|---|---|
| FAST-LIO `laser_mapping_node` | `lio_odom -> livox_frame` | add/use a TF-disable parameter; keep `/state_estimation` |
| ICP `transform_publisher` | `lio_map -> lio_odom` | remove from relocalization launch |
| ICP `global_odometry_publisher` | `/state_estimation` -> `/state_estimation_global` | retire after consumers use manager body odometry |
| SCAN `lidar_to_body_odom` | optional `lio_map -> scan_base_link` | stop node after planner consumes manager body odometry |
| ad-hoc static publisher | `livox_frame -> lidar` | remove; put alias in robot profile |
| SCAN grid map | `lio_map -> sliding_map` | retain; it owns an algorithm-private frame |
| robot state publisher | joint tree | retain, but ensure its root does not duplicate `scan_base_link` |

FAST-LIO currently always broadcasts its odometry TF. Authority promotion is
blocked until that broadcaster can be disabled without disabling the odometry
topic.

The manager intentionally subscribes to local `/state_estimation`
(`lio_odom -> tracking_sensor`). Do not feed `/state_estimation_global` back
into it: map alignment and sensor-to-body conversion are already performed by
the manager.

## Phase 1: shadow

1. Build and source `omni_tf_manager`.
2. Start the manager before SLAM/ICP, using `mode:=shadow`.
3. Keep all existing TF publishers running.
4. Compare candidate body odometry with the current planner body odometry.
5. Record a bag containing inputs, both body odometries, `/tf`, `/tf_static`
   and `/diagnostics`.

Matrix example:

```bash
ros2 launch omni_tf_manager omni_tf_manager.launch.py \
  config_file:="$(ros2 pkg prefix omni_tf_manager)/share/omni_tf_manager/config/matrix_xgw.yaml" \
  mode:=shadow
```

Expected candidate topics:

```bash
ros2 topic echo /omni/tf_manager/body_odom --once
ros2 topic echo /omni/tf_manager/body_odom_global --once
ros2 topic echo /omni/tf_manager/ready --once
```

Acceptance criteria:

- no rejected samples during nominal motion;
- candidate `map -> base` differs from the legacy result by less than the
  calibrated tolerance;
- yaw sign and forward motion agree with simulator/robot ground truth;
- readiness drops within `timeouts.sensor_odom_s` after SLAM stops;
- no TF is published by the manager in shadow mode.

## Phase 2: authority in simulation

1. Stop SCAN-Planner.
2. disable FAST-LIO TF while retaining `/state_estimation`;
3. remove ICP `transform_publisher` from the launch;
4. remove temporary static publishers;
5. start `omni_tf_manager mode:=authority` before ICP;
6. start SLAM and wait for `/omni/tf_manager/ready=true`;
7. remap Planner body input to `/omni/tf_manager/body_odom_global`;
8. keep Planner's `sliding_map` TF enabled.

The resulting core tree must contain exactly:

```text
lio_map -> lio_odom -> scan_base_link -> sensors
```

Audit with:

```bash
ros2 run tf2_tools view_frames
ros2 run tf2_ros tf2_echo lio_map scan_base_link
ros2 run tf2_ros tf2_echo scan_base_link livox_frame
ros2 topic info /tf --verbose
ros2 topic info /tf_static --verbose
```

## Phase 3: planner cutover

- remove `lidar_to_body_odom` from the default Planner launch;
- use manager global body odometry for planning and closed-loop control;
- gate autonomous command authorization on manager readiness;
- delete body-to-sensor constants from `run_matrix_planner.sh`;
- select the robot profile from deployment configuration instead.

## Phase 4: real robot

- create a real-robot profile from calibrated transforms, not Matrix values;
- replay a recorded stationary/motion bag in shadow mode;
- validate against surveyed poses and physical dimensions;
- perform localization-loss, timestamp, process-death and transform-jump fault
  injection before enabling autonomous motion.
