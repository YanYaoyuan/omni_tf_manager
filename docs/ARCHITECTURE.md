# Architecture and safety contract

## Ownership boundary

`omni_tf_manager` owns only the robot's localization spine and calibrated
sensor mounts:

| Edge | Source data | Authority |
|---|---|---|
| `map -> odom` | ICP result or configured alignment | `omni_tf_manager` |
| `odom -> base` | tracking-sensor odometry + extrinsic | `omni_tf_manager` |
| `base -> sensor` | versioned robot profile | `omni_tf_manager` |
| camera link -> optical | REP-103 profile transform | `omni_tf_manager` |

The following remain outside this package:

- joint transforms: `robot_state_publisher`;
- `sliding_map`: SCAN-Planner grid-map implementation;
- temporary mission/checkpoint frames: mission owner;
- tool frames driven by joints: robot description/state publisher.

No other process may publish a child frame owned by this manager while it runs
in authority mode.

## Modes

### Shadow

Shadow is the mandatory first deployment phase. It subscribes to production
inputs, validates the profile and publishes candidate body odometry and
diagnostics, but never writes `/tf` or `/tf_static`.

### Authority

Authority publishes the configured static tree, `odom -> base`, and optionally
`map -> odom`. It must only be enabled after a graph audit and shutdown of
legacy broadcasters.

## Coordinate convention

- right-handed ROS REP-103;
- +X forward, +Y left, +Z up for robot mechanical frames;
- metres and radians in every profile;
- camera optical frames use +Z forward, +X right, +Y down;
- a simulator/vendor coordinate conversion is performed before values enter a
  profile and must be documented with the profile.

The manager deliberately does not guess units, handedness or degrees versus
radians. Ambiguous input is a configuration error.

## Validation and failure policy

The process fails during startup for:

- missing required sensor roles;
- duplicate static child frames;
- disconnected or cyclic static transforms;
- non-finite extrinsics;
- missing tracking frame;
- invalid operating mode or map-alignment source.

Individual runtime samples are rejected for:

- wrong parent/child frame;
- zero, stale, future or non-monotonic timestamps;
- invalid/non-normalized quaternions;
- translation or rotation jumps outside configured motion bounds;
- map reinitialization outside configured alignment bounds.

Readiness becomes false when body odometry is stale or a required map alignment
has not initialized. Autonomous motion authorization should consume
`/omni/tf_manager/ready`; topic existence alone is not sufficient.

## Map alignment lifecycle

The current relocalization contract treats the first accepted ICP result as
`T_map_odom`. By default it is immutable for the localization epoch and is
published through `/tf_static`.

If runtime relocalization is introduced, configure:

```yaml
map_to_odom.is_static: false
map_to_odom.allow_reinitialization: true
```

and define product-specific jump limits. Mission control must stop the robot
before accepting a discontinuous global correction.

## Covariance

- lever-arm coupling is applied when converting sensor pose covariance to the
  body origin;
- twist and twist covariance are rotated into the body frame and corrected for
  the lever arm;
- local body covariance is rotated into the global map frame;
- ICP covariance can be added to global pose covariance as a conservative
  approximation with `publish.add_map_covariance=true`.

The optional addition assumes independence and should be replaced by a
correlated estimator if tighter uncertainty consistency is required.

## Production deployment expectations

- profiles are version controlled and reviewed like source code;
- `profile_version` and `calibration_id` are exposed in diagnostics and must
  identify the deployed calibration artifact;
- every physical unit has a calibration record and checksum;
- authority mode is started by the system orchestrator before SLAM emits its
  one-shot ICP result;
- `/diagnostics` and readiness feed the safety/mission supervisor;
- a rosbag regression verifies each supported robot profile;
- duplicate-edge and stale-odometry fault injection are release gates.
