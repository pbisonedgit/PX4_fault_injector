# Current PX4-side configuration and lifecycle state of one FAULT fault definition.
# This topic is also intended to provide ULog evidence of accepted parameters,
# activation, expiration, clearing, and the number of values actually transformed.

uint8 PROTOCOL_VERSION = 1

uint8 TARGET_NONE = 0
uint8 TARGET_IMU = 1
uint8 TARGET_MAG = 2
uint8 TARGET_GNSS = 3
uint8 TARGET_DOWNWARD_RANGE = 4
uint8 TARGET_BAROMETER = 5
uint8 TARGET_MOTOR = 6

uint8 MODE_NO_EFFECT = 0
uint8 MODE_ACCEL_BIAS = 1
uint8 MODE_ACCEL_NOISE = 2
uint8 MODE_GYRO_BIAS = 3
uint8 MODE_GYRO_NOISE = 4
uint8 MODE_MAG_BIAS = 5
uint8 MODE_MAG_NOISE = 6
uint8 MODE_GNSS_POSITION_BIAS = 7
uint8 MODE_GNSS_POSITION_NOISE = 8
uint8 MODE_GNSS_VELOCITY_BIAS = 9
uint8 MODE_GNSS_VELOCITY_NOISE = 10
uint8 MODE_DOWNWARD_RANGE_BIAS = 11
uint8 MODE_DOWNWARD_RANGE_NOISE = 12
uint8 MODE_DOWNWARD_RANGE_DEGRADATION = 13
uint8 MODE_BARO_ALTITUDE_BIAS = 14
uint8 MODE_BARO_ALTITUDE_NOISE = 15
uint8 MODE_MOTOR_OUTPUT_BIAS = 16
uint8 MODE_MOTOR_OUTPUT_DELAY = 17
uint8 MODE_MOTOR_MAX_OUTPUT_DEGRADATION = 18

uint8 STATE_EMPTY = 0
uint8 STATE_PENDING = 1
uint8 STATE_ACTIVE = 2
uint8 STATE_CLEARED = 3
uint8 STATE_EXPIRED = 4
uint8 STATE_REJECTED = 5

uint8 AXIS_X = 1
uint8 AXIS_Y = 2
uint8 AXIS_Z = 4

# For TARGET_MOTOR the axis_mask field is a motor-selection mask.
uint8 MOTOR_1 = 1
uint8 MOTOR_2 = 2
uint8 MOTOR_3 = 4
uint8 MOTOR_4 = 8
uint8 MOTOR_5 = 16
uint8 MOTOR_6 = 32
uint8 MOTOR_7 = 64
uint8 MOTOR_8 = 128

uint8 ORB_QUEUE_LENGTH = 8

uint64 timestamp                 # [us] PX4 time when this status was generated
uint64 activation_time_us        # [us] PX4 monotonic activation time
uint64 expiration_time_us        # [us] PX4 monotonic expiration time

uint32 scenario_id               # Scenario owning the fault
uint32 sequence                  # Command sequence that created or last updated the fault
uint32 applied_count             # Number of values actually transformed by a fault hook
uint32 instance_mask             # Bitmask selecting target instances
uint32 random_seed               # Accepted deterministic random seed
uint32 delay_us                  # [us] Accepted delay parameter

float32[3] vector                # Accepted target/mode-specific vector parameter
float32 scalar                   # Accepted target/mode-specific scalar parameter

uint16 fault_id                  # Scenario-local fault identifier

uint8 target_system              # External MAVLink system receiving this status
uint8 target_component           # External MAVLink component receiving this status
uint8 protocol_version           # FAULT fault protocol version used by PX4
uint8 target                     # TARGET_*
uint8 mode                       # MODE_*
uint8 state                      # STATE_*
uint8 axis_mask                  # AXIS_* for vector sensors; MOTOR_* mask for TARGET_MOTOR
