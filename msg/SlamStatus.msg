builtin_interfaces/Time stamp

uint8 MODE_STOPPED=0
uint8 MODE_MAPPING=1
uint8 MODE_LOCALIZATION=2
uint8 mode

uint8 STATE_STOPPED=0
uint8 STATE_STARTING_MAPPING=1
uint8 STATE_MAPPING=2
uint8 STATE_SAVING=3
uint8 STATE_MAP_READY=4
uint8 STATE_STARTING_LOCALIZATION=5
uint8 STATE_RELOCALIZING=6
uint8 STATE_LOCALIZED=7
uint8 STATE_DEGRADED=8
uint8 STATE_LOST=9
uint8 STATE_STOPPING=10
uint8 STATE_ERROR=11
uint8 state

string map_id
uint32 map_version
string map_checksum
bool initialized
float32 lidar_age_ms
float32 imu_age_ms
float32 odom_age_ms
# Newest ICP alignment score (lower is better). NaN while no source exists.
float32 fitness_score
float32 pose_covariance_norm
# Pose discontinuities observed in the current SLAM session.
uint32 pose_jump_count
uint32 reason_code
string reason_text
uint64 status_sequence
