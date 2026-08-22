# Migration to the Omni TF authority

## Completed code cutover

| Legacy behavior | Current behavior |
|---|---|
| FAST_LIO always broadcasts estimator TF | `publish.tf_en=false`; odometry remains available |
| FAST_LIO labels the IMU state as `livox_frame` | child is configurable `omni_imu_link` |
| ICP result has no child and is treated as `map -> odom` | `/icp_result` explicitly carries `map -> LiDAR`; manager normalizes it |
| ICP global odometry adapter | manager publishes global body odometry |
| mapping and localization use different odom frame names | both use configured `omni_odom` |
| distributed `lio_*`, `scan_*`, vendor core frames | one `omni_*` frame contract |

The global point-cloud adapter remains because it transforms point data, not
TF ownership. SCAN-Planner no longer publishes `sliding_map` TF.

## Deployment order

1. Build `omni_tf_manager` first, or let colcon order it before `omni_slam` in
   the same workspace.
2. Select one calibrated TF profile.
3. Start `omni_tf_manager` before starting a mapping/localization epoch.
4. Start `omni_slam_manager` with the same `tf_profile_path`.
5. Consume `/omni/tf_manager/body_odom_global` in Planner.
6. Remove/disable Planner body-odometry and sensor static-TF adapters.
7. Audit `/tf` and `/tf_static` publisher counts.

Managed launch example:

```bash
ros2 launch omni_slam_manager manager.launch.py \
  tf_profile_path:=$(ros2 pkg prefix omni_tf_manager)/share/omni_tf_manager/config/matrix_xgw.yaml
```

Standalone Matrix mapping/relocalization scripts start the same profile in an
explicit fallback mode. Do not also launch another TF manager; use
`--no-tf-manager` only when an external manager already owns the tree.

## Required graph

```text
omni_map -> omni_odom -> omni_base_link
                             ├── omni_imu_link -> omni_lidar_link
                             ├── omni_depth_camera_link -> optical
                             └── omni_rgb_camera_link -> optical
```

Audit commands:

```bash
ros2 run tf2_tools view_frames
ros2 run tf2_ros tf2_echo omni_map omni_base_link
ros2 run tf2_ros tf2_echo omni_base_link omni_lidar_link
ros2 topic info /tf --verbose
ros2 topic info /tf_static --verbose
ros2 topic echo /omni/tf_manager/ready
```

Acceptance requires one parent and one publisher per child, correct forward
and yaw signs, stable sensor lever arms, timeout-driven readiness loss, and no
`lio_map`, `lio_odom`, `scan_base_link` or `livox_frame` in the managed tree.

## Remaining production work

- run the stationary/forward/yaw/slope Matrix acceptance bag;
- create reviewed profiles for each real robot and sensor installation;
- pin released `omni_slam` and `omni_tf_manager` tags together for deployment.
