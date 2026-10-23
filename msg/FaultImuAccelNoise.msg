# Current effective runtime configuration for the IMU acceleration-noise injector.
#
# This is a dedicated latest-value effect snapshot. The FAULT fault manager is
# the sole lifecycle authority.
#
# When active is false, axis_mask, random_seed, and sigma_m_s2 are zero.

uint64 timestamp

uint32 scenario_id
uint32 sequence
uint32 random_seed

float32[3] sigma_m_s2

uint16 fault_id

uint8 axis_mask
bool active
