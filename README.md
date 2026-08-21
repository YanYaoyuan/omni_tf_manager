# omni_tf_manager

`omni_tf_manager` is the single authority for the core localization and sensor
TF tree of Omni inspection robots. It is a ROS 2 Humble C++ package designed to
replace distributed TF publication in SLAM, planner adapters and deployment
scripts without coupling frame names to a particular robot model.

The target production tree is:

```text
lio_map
└── lio_odom
    └── scan_base_link
        ├── LiDAR link / input alias
        ├── IMU link
        ├── depth camera link
        │   └── depth optical frame
        └── RGB camera link
            └── RGB optical frame
```

Algorithm-private frames such as `sliding_map`, robot joint transforms and
mission frames remain owned by their respective components. The invariant is
one publisher per TF edge, not one TF process for the entire ROS graph.

## Capabilities

- `shadow` mode computes candidate body odometry without publishing TF;
- `authority` mode owns `map -> odom`, `odom -> base` and configured static
  sensor transforms;
- derives body odometry from a tracking-sensor odometry and a calibrated
  `base -> tracking_sensor` transform;
- publishes local and global body odometry with transformed pose/twist
  covariance;
- supports ICP, identity, parameter and disabled map-alignment sources;
- requires LiDAR, IMU, depth-camera and RGB-camera roles by default;
- validates the static tree, unique child ownership, timestamps, frame names,
  quaternion norms, discontinuities and stale data;
- publishes transient-local readiness and standard ROS diagnostics;
- keeps logical sensor IDs separate from actual frame names, allowing a new
  robot model to be migrated by adding a profile rather than changing C++.

## Build

```bash
cd /home/user/robot/omni_code/omni_navi
source /opt/ros/humble/setup.bash
colcon build --base-paths omni_tf_manager --symlink-install
source install/setup.bash
colcon test --base-paths omni_tf_manager --event-handlers console_direct+
colcon test-result --verbose
```

## Matrix xgw shadow validation

Start this node before the one-shot ICP result is published:

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch omni_tf_manager omni_tf_manager.launch.py \
  config_file:="$(ros2 pkg prefix omni_tf_manager)/share/omni_tf_manager/config/matrix_xgw.yaml" \
  mode:=shadow
```

Shadow output is intentionally namespaced to avoid colliding with the existing
planner adapter:

```text
/omni/tf_manager/body_odom
/omni/tf_manager/body_odom_global
/omni/tf_manager/ready
/diagnostics
```

Do not use `mode:=authority` until all legacy publishers for the same edges
have been disabled. See [docs/MIGRATION.md](docs/MIGRATION.md).

## Robot profiles

Profiles use stable logical transform IDs and configurable frame names:

```yaml
frames.base: scan_base_link
frames.tracking: vendor_lidar_frame
profile_version: 1.2.0
calibration_id: DOG_007_2026_08_21

static_transforms: [lidar_mount, imu_mount, depth_link, rgb_link]

static_transform.lidar_mount.role: lidar
static_transform.lidar_mount.parent_frame: scan_base_link
static_transform.lidar_mount.child_frame: vendor_lidar_frame
static_transform.lidar_mount.translation: [0.13, 0.0, 0.18]
static_transform.lidar_mount.rotation_rpy: [0.0, 0.0, 0.0]
```

Changing a vendor frame from `livox_frame` to `hesai_lidar` therefore requires
only a profile change. All translations are metres and all rotations are fixed
axis roll-pitch-yaw radians following ROS REP-103.

Profiles are immutable while the node is running. A profile or calibration
change requires a controlled node restart and should carry a new calibration
version in deployment configuration.

## Runtime contract

Input sensor odometry must satisfy:

```text
header.frame_id == frames.odom
child_frame_id  == frames.tracking
pose            == T_odom_tracking
twist           expressed in frames.tracking
```

For `map_to_odom.source=icp_pose`, `/icp_result` must contain `T_map_odom` and
use `header.frame_id == frames.map`.

Use FAST-LIO's local `/state_estimation` as the input. The existing
`/state_estimation_global` has already applied map alignment and must not be
used as manager input.

The manager computes:

```text
T_odom_base = T_odom_tracking × inverse(T_base_tracking)
T_map_base  = T_map_odom × T_odom_base
```

Invalid samples are rejected instead of being silently relabelled.

## Documentation

- [Architecture and safety contract](docs/ARCHITECTURE.md)
- [Migration from current SLAM and planner TF publishers](docs/MIGRATION.md)
- [Robot profile checklist](docs/ROBOT_PROFILE_CHECKLIST.md)
