# FAULT fault-management command received by PX4 from an external scenario orchestrator.
# This topic only carries configuration. It does not itself modify sensor data,
# controller outputs, actuator commands, battery state, or flight behaviour.

uint8 PROTOCOL_VERSION = 1

uint8 ACTION_SET = 1
uint8 ACTION_CLEAR = 2
uint8 ACTION_CLEAR_ALL = 3
uint8 ACTION_QUERY = 4

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

uint64 timestamp                 # [us] PX4 time when mavlink_receiver published this command

uint32 scenario_id               # Scenario identifier supplied by the external orchestrator
uint32 sequence                  # Monotonically increasing command sequence
uint32 start_delay_ms            # [ms] Relative delay from PX4 command reception to activation
uint32 duration_ms               # [ms] Requested active duration
uint32 random_seed               # Deterministic seed for stochastic fault modes
uint32 delay_us                  # [us] Parameter for future delayed-data fault modes
uint32 instance_mask             # Bitmask selecting sensor, battery, or actuator instances

float32[3] vector                # Target/mode-specific vector parameter
float32 scalar                   # Target/mode-specific scalar parameter

uint16 fault_id                  # Scenario-local fault identifier

uint8 source_system              # MAVLink system that sent the command
uint8 source_component           # MAVLink component that sent the command
uint8 target_system              # MAVLink system expected to process the command
uint8 target_component           # MAVLink component expected to process the command
uint8 protocol_version           # FAULT fault protocol version
uint8 action                     # ACTION_*
uint8 target                     # TARGET_*
uint8 mode                       # MODE_*
uint8 axis_mask                  # AXIS_* for vector sensors; MOTOR_* mask for TARGET_MOTOR
