# Current manager-owned runtime configuration for barometric-altitude bias.
#
# The scalar effect is expressed in metres and is applied only to the
# baroSample height supplied to EKF2. The raw vehicle_air_data topic, pressure,
# temperature, density, device identity, calibration count and reset semantics
# remain unchanged. When active is false, bias_m must be zero.

uint64 timestamp
uint32 scenario_id
uint32 sequence
float32 bias_m
uint16 fault_id
bool active
