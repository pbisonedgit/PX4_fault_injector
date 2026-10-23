# Current manager-owned runtime configuration for GNSS velocity bias.
#
# vector axis semantics are local NED velocity in metres per second:
#   AXIS_X = north, AXIS_Y = east, AXIS_Z = down.
#
# The effect is applied only to gnssSample.vel supplied to EKF2. GNSS position,
# fix type, satellite count, accuracy estimates, timestamps, heading, jamming,
# spoofing and availability metadata are not changed. When active is false,
# axis_mask and bias_ned_m_s must be zero.

uint64 timestamp                 # [us] PX4 time when this effect snapshot was published

uint32 scenario_id               # Scenario owning the current GNSS velocity-bias definition, or 0 when empty
uint32 sequence                  # Command/lifecycle sequence associated with this snapshot

float32[3] bias_ned_m_s          # [m/s] Unreported north, east, down velocity offset

uint16 fault_id                  # Scenario-local fault identifier, or 0 when empty

uint8 axis_mask                  # AXIS_X=north, AXIS_Y=east, AXIS_Z=down; zero whenever inactive
bool active                      # True only while the concrete fault object is ACTIVE
