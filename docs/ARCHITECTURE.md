# Architecture and safety contract

## Decision

The deployed system uses one TF-management component, not separate static,
dynamic and SLAM adapter nodes. `omni_tf_manager` publishes all fixed sensor
edges and both dynamic localization edges. `omni_slam` keeps its existing
state machine and typed status, and only calculates pose/alignment data.

This is a deliberate product-specific extension of the generic monitor-only
proposal in `ROS2_通用_TF_管理模块设计方案.md`. The profile format remains robot
agnostic. `omni_tf_manager` owns the stable `SlamStatus` interface contract;
the SLAM state machine publishes it and remains responsible only for state and
pose estimation.

## Edge ownership

| Edge | Source | Sole authority |
|---|---|---|
| `omni_map -> omni_odom` | identity in mapping; normalized ICP sensor pose in localization | `omni_tf_manager` |
| `omni_odom -> omni_base_link` | SLAM IMU pose plus calibrated lever arm | `omni_tf_manager` |
| body/IMU/LiDAR/camera fixed edges | versioned robot profile | `omni_tf_manager` |
| articulated joint edges | joint state | `robot_state_publisher` |

FAST_LIO and ICP are forbidden from broadcasting an edge in the managed tree.
FAST_LIO `publish.tf_en` is false and the old ICP transform publisher is no
longer built or launched.

## State behavior

The manager requests reliable, transient-local `/omni/slam/status`. Missing or
stale status is fail-closed and equivalent to stopped.

- stopped: keep static TF, clear dynamic session state, publish ready=false;
- mapping: initialize `T_map_odom` to identity for the epoch;
- localization: clear alignment and wait for the first valid ICP result;
- mode transition: clear prior odometry continuity and alignment state.

`timeouts.slam_status_s` prevents a crashed SLAM manager from leaving fresh
dynamic TF traffic. Previously published dynamic samples expire naturally.
Standalone simulation may set `slam_state.required=false` and an explicit
fallback mode; production launch must use status-driven operation.

## Pose semantics

FAST_LIO `state_point.pos/rot` describes the IMU pose. Its body-frame point
cloud is also converted into IMU coordinates. Therefore:

```text
/state_estimation.header.frame_id = frames.odom
/state_estimation.child_frame_id  = frames.tracking (IMU)
```

Labeling this pose as a LiDAR frame and then applying `base -> LiDAR` creates a
systematic body-position error. The manager instead resolves
`T_base_tracking` through the configured static tree.

ICP publishes an odometry-shaped transform with an explicit child:

```text
/icp_result.header.frame_id = frames.map
/icp_result.child_frame_id  = frames.icp_sensor (LiDAR)
/icp_result.pose            = T_map_icp_sensor
```

The manager resolves the reviewed `T_tracking_icp_sensor` from the static
tree and computes `T_map_odom = T_map_icp_sensor × inverse(T_tracking_icp_sensor)`
for the new localization epoch. A raw LiDAR pose is never relabeled as odom.

For closed-source simulators, optional typed relays in this same process
validate legacy sensor `frame_id` values and republish canonical Omni headers.
They do not create TF edges and are not additional nodes.

## Validation and failure policy

Startup fails for invalid/duplicate/disconnected/cyclic static transforms,
missing sensor roles, non-finite values, aliased core frames or naming-policy
violations. Runtime samples are rejected for wrong frames, invalid timestamps,
bad quaternion norms, impossible motion jumps or unsafe ICP reinitialization.

Readiness requires an active, fresh SLAM mode, fresh accepted odometry and (in
localization) a valid map alignment. Motion authorization should consume
readiness rather than infer health from topic existence.

## Coordinate convention

- ROS REP-103 right-handed mechanical frames: +X forward, +Y left, +Z up;
- metres and radians;
- camera optical frames: +Z forward, +X right, +Y down;
- every vendor/simulator conversion is completed before values enter a
  profile and recorded in its calibration metadata.

## Production gates

- exactly one publisher for every managed child frame;
- all managed frame and node names begin with `omni_`;
- calibrated profile/version/checksum tied to the deployed robot;
- stationary, forward, yaw and slope rosbag regression;
- status timeout, odometry timeout, malformed sample and process-death tests;
- canonical sensor topics verified against the configured mechanical/optical
  frame IDs before sensor fusion is enabled.
