# Current manager-owned runtime configuration for deterministic downward-range noise.
#
# sigma_m is the standard deviation in metres. Noise is applied only to the
# selected downward-facing distance measurement supplied to EKF2. The original
# distance_sensor topic and all health, quality, limit, orientation, device and
# availability metadata remain unchanged. When active is false, random_seed and
# sigma_m must be zero.

uint64 timestamp                 # [us] PX4 time when this effect snapshot was published

uint32 scenario_id               # Scenario owning the current definition, or 0 when empty
uint32 sequence                  # Command/lifecycle sequence associated with this snapshot
uint32 random_seed               # Deterministic external seed; every uint32 value is valid while active

float32 sigma_m                  # [m] Standard deviation of additive range noise

uint16 fault_id                  # Scenario-local fault identifier, or 0 when empty
bool active                      # True only while the concrete fault object is ACTIVE
