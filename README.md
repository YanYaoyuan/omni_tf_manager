# omni_tf_manager

`omni_tf_manager` is the single TF authority for Omni inspection robots. It
owns the localization spine and calibrated fixed sensor tree while SLAM remains
responsible only for estimating poses.

The production tree is profile-driven and uses Omni-owned names:

```text
omni_map
└── omni_odom
    └── omni_base_link
        ├── omni_imu_link
        │   └── omni_lidar_link
        ├── omni_depth_camera_link
        │   └── omni_depth_camera_optical_frame
        └── omni_rgb_camera_link
            └── omni_rgb_camera_optical_frame
```

## Ownership and SLAM state

In `authority` mode this node is the only publisher of:

- dynamic `map -> odom` and `odom -> base`;
- configured static body, LiDAR, IMU, depth-camera and RGB-camera transforms.

It owns the typed `omni_tf_manager/msg/SlamStatus` contract, consumes it on
`/omni/slam/status`, and changes behavior without starting another bridge
process. `omni_slam_manager` publishes this status; no TF-manager package
depends on a SLAM implementation package.

| SLAM mode | Dynamic TF behavior |
|---|---|
| stopped, missing or stale status | no dynamic TF; readiness false |
| mapping | publish identity `map -> odom` and pose-derived `odom -> base` |
| localization | normalize ICP `map -> LiDAR` through static extrinsics, then publish the complete chain |

The node resets session pose/alignment state at every mode transition. Static
sensor TF remains available while SLAM is stopped. `shadow` mode performs the
same validation and body-odometry conversion but has no `/tf` or `/tf_static`
publisher.

FAST_LIO must use `publish.tf_en=false`. Its pose output is the IMU body pose,
so the odometry child contract is `frames.tracking` (normally
`omni_imu_link`), not the LiDAR frame.

## Inputs and outputs

Input odometry:

```text
topic             topics.sensor_odom (default /state_estimation)
header.frame_id   frames.odom
child_frame_id    frames.tracking
pose              T_odom_tracking
```

Localization alignment:

```text
topic             topics.icp_pose (default /icp_result)
header.frame_id   frames.map
child_frame_id    frames.icp_sensor
pose              T_map_icp_sensor
```

The manager computes:

```text
T_odom_base = T_odom_tracking × inverse(T_base_tracking)
T_map_odom  = T_map_icp_sensor × inverse(T_tracking_icp_sensor)
T_map_base  = T_map_odom × T_odom_base
```

Stable outputs are `/omni/tf_manager/body_odom`,
`/omni/tf_manager/body_odom_global`, `/omni/tf_manager/ready` and
`/diagnostics`.

## Configuration

All core and static frame names live in one robot profile. With
`naming.require_omni_prefix=true`, every frame in the managed tree must begin
with `omni_`. `omni_slam_manager` reads the same file through
`tf_profile_path`, then passes those names into FAST_LIO and ICP launch files.

Each robot model only replaces its YAML profile. Logical roles remain
`lidar`, `imu`, `depth_camera` and `rgb_camera`; C++ code does not contain a
vendor frame name.

`config/matrix_xgw.yaml` is verified against Matrix `config/config.json` and
the XGW MuJoCo model. Matrix's LiDAR and `livox_imu` site are co-located, so
its FAST-LIO `T_imu_lidar` is identity. The same profile enables typed header
normalization for the closed-source Matrix publishers.

`config/omni_dog.yaml` and `config/omni_vbot_dog.yaml` define the real-dog
LiDAR/IMU topics and reviewed 6DoF calibration sources. The generic dog remains
in `shadow` mode with unverified aliases. The Vbot profile runs in `authority`
mode; legacy TF publishers must therefore be disabled. Its aliases are verified
against a live S100 capture and the previous direct FAST-LIO integration: the
closed-source driver reports the shared `vita_lidar` header on both sensor
topics, so the relay normalizes each topic to its calibrated physical frame
while leaving the payload untouched.

## Sensor frame aliases

A sensor relay must declare:

```yaml
sensor_relay.lidar.operation: identity_frame_alias
sensor_relay.lidar.alias_verified: true
sensor_relay.lidar.input_frame: vendor_lidar_frame
sensor_relay.lidar.output_frame: omni_lidar_link
```

`identity_frame_alias` is a metadata correction, not a TF transform: the
message payload, timestamp and covariance remain byte-for-byte/numerically
unchanged. Use it only when the payload already uses the axes and origin of the
canonical physical frame. Authority mode refuses unreviewed aliases. Shadow
mode observes and reports their actual `frame_id` but publishes no canonical
sample while `alias_verified=false`.

Capture live headers before approval:

```bash
ros2 topic echo /front_lidar --once --field header
ros2 topic echo /front_lidar/imu --once --field header
ros2 topic echo /lidar_points --once --field header
ros2 topic echo /lidar_imu --once --field header
```

## Build and test

The manager owns its status interface and builds independently:

```bash
cd /home/user/robot/omni_code/omni_navi
source /opt/ros/humble/setup.bash
colcon build --base-paths omni_tf_manager --symlink-install
source install/setup.bash
ROS_LOG_DIR=/tmp/omni_ros_test_logs \
  colcon test --base-paths omni_tf_manager
colcon test-result --verbose
```

### RDK S100 artifact

Every successful push to `main` packages and uploads the standalone artifact
`omni-tf-manager-rdk-s100-aarch64`. It contains a relocatable ARM64 overlay,
the generated Python ROSIDL package, launch files, all robot profiles and a
manifest. The target must provide RDK OS and Humble/TogetheROS under
`/opt/tros/humble` or `/opt/ros/humble`.

The integrated `omni_slam` S100 artifact already embeds the same TF Manager
runtime. Use the standalone artifact only for profile/relay testing or an
independent TF Manager deployment; never start both copies simultaneously.

Vbot standalone relay verification:

```bash
source /app/script/env.sh
source /userdata/omni_navi/omni_tf_manager/setup.bash
ros2 launch omni_tf_manager omni_tf_manager.launch.py \
  config_file:=/userdata/omni_navi/omni_tf_manager/config/omni_tf_manager/omni_vbot_dog.yaml \
  mode:=profile
```

The Vbot profile runs in `authority` mode. It publishes the canonical sensor
topics plus `/tf` and `/tf_static`; do not run any legacy TF publisher beside
it.

For the integrated runtime, place `omni_tf_manager` and `omni_slam` in the
same colcon workspace. Colcon resolves `omni_tf_manager` before the SLAM
manager that publishes `SlamStatus`.

Normal managed launch:

```bash
ros2 launch omni_slam_manager manager.launch.py \
  tf_profile_path:=$(ros2 pkg prefix omni_tf_manager)/share/omni_tf_manager/config/matrix_xgw.yaml
```

The standalone Matrix scripts start this manager automatically with an
explicit mapping/localization fallback mode because they do not run the SLAM
control-plane node.

Revalidate Matrix model/config/profile consistency after every simulator
upgrade:

```bash
python3 tools/validate_matrix_xgw_profile.py
```

See [Architecture](docs/ARCHITECTURE.md),
[Migration](docs/MIGRATION.md), and the
[robot profile checklist](docs/ROBOT_PROFILE_CHECKLIST.md).
