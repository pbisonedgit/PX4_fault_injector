# Current manager-owned runtime configuration for GNSS position bias.
#
# vector axis semantics are local NED displacement in metres:
#   AXIS_X = north, AXIS_Y = east, AXIS_Z = down.
#
# The effect is applied only to the gnssSample position supplied to EKF2. GNSS
# fix type, satellite count, accuracy estimates, velocity, timestamps, jamming,
# spoofing and availability metadata are not changed. When active is false,
# axis_mask and bias_ned_m must be zero.

uint64 timestamp                 # [us] PX4 time when this effect snapshot was published

uint32 scenario_id               # Scenario owning the current GNSS position-bias definition, or 0 when empty
uint32 sequence                  # Command/lifecycle sequence associated with this snapshot

float32[3] bias_ned_m            # [m] Unreported north, east, down position offset

uint16 fault_id                  # Scenario-local fault identifier, or 0 when empty

uint8 axis_mask                  # AXIS_X=north, AXIS_Y=east, AXIS_Z=down; zero whenever inactive
bool active                      # True only while the concrete fault object is ACTIVE
