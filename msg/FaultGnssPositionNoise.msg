# Current manager-owned runtime configuration for deterministic GNSS position noise.
#
# sigma axis semantics are local NED displacement in metres:
#   AXIS_X = north, AXIS_Y = east, AXIS_Z = down.
#
# Noise is applied only to the gnssSample position supplied to EKF2. GNSS fix
# type, satellite count, accuracy estimates, velocity, timestamps, jamming,
# spoofing and availability metadata are not changed. When active is false,
# random_seed, axis_mask and sigma_ned_m must be zero.

uint64 timestamp                 # [us] PX4 time when this effect snapshot was published

uint32 scenario_id               # Scenario owning the current GNSS position-noise definition, or 0 when empty
uint32 sequence                  # Command/lifecycle sequence associated with this snapshot
uint32 random_seed               # Deterministic external seed; every uint32 value including zero is valid while active

float32[3] sigma_ned_m           # [m] Per-axis standard deviation in north, east, down

uint16 fault_id                  # Scenario-local fault identifier, or 0 when empty

uint8 axis_mask                  # AXIS_X=north, AXIS_Y=east, AXIS_Z=down; zero whenever inactive
bool active                      # True only while the concrete fault object is ACTIVE
