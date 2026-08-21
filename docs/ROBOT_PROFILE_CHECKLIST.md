# Robot profile checklist

Create one reviewed YAML profile per robot model and sensor installation.
Vendor message frame names may differ, but managed TF names use the `omni_`
prefix and logical roles remain stable.

## Identity

- profile name and version;
- robot model and serial-number scope;
- sensor model/serial numbers;
- calibration date, operator and tool;
- source coordinate convention and conversion;
- profile checksum in deployment manifest.

## Core frames

- global map frame;
- continuous odometry frame;
- navigation body frame;
- SLAM tracking frame;
- exact SLAM odometry parent/child contract.
- `omni_slam_manager` is started with this file as `tf_profile_path`;
- FAST_LIO TF publication is disabled and its pose child is the IMU tracking
  frame.

## Required sensors

- LiDAR mechanical frame and any driver alias;
- IMU frame and LiDAR-to-IMU relationship;
- depth-camera mechanical and optical frames;
- RGB-camera mechanical and optical frames.

For each transform verify translation in metres, RPY in radians, direction
`parent -> child`, handedness, and whether the message header uses the
mechanical or optical frame.

## Runtime limits

- maximum expected linear/angular speed;
- timestamp age and future offset;
- odometry timeout;
- SLAM-status heartbeat timeout;
- allowed localization reinitialization jump;
- quaternion norm tolerance.

## Acceptance

- static tree validation passes;
- forward motion follows +X in base and tracking frames;
- positive yaw has the same sign in ground truth, SLAM and body output;
- stationary sensor-to-body transform is constant within calibration noise;
- camera projections align with LiDAR targets;
- manager readiness and diagnostics reach the safety supervisor;
- no child frame has multiple publishers in authority mode.
