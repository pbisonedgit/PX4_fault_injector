# Current manager-owned runtime configuration for downward-range bias.
#
# The scalar bias is expressed in metres and is applied only to the selected
# downward-facing distance measurement supplied to EKF2. The original
# distance_sensor topic, device identity, limits, orientation, quality,
# variance, type and availability metadata remain unchanged. When active is
# false, bias_m must be zero.

uint64 timestamp                 # [us] PX4 time when this effect snapshot was published

uint32 scenario_id               # Scenario owning the current definition, or 0 when empty
uint32 sequence                  # Command/lifecycle sequence associated with this snapshot

float32 bias_m                   # [m] Unreported additive downward-range offset

uint16 fault_id                  # Scenario-local fault identifier, or 0 when empty
bool active                      # True only while the concrete fault object is ACTIVE
