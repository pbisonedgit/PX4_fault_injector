# Current effective runtime configuration for the IMU gyroscope-noise injector.
#
# This is a dedicated latest-value effect snapshot. The FAULT fault manager is
# the sole lifecycle authority.
#
# When active is false, axis_mask, random_seed, and sigma_rad_s are zero.

uint64 timestamp

uint32 scenario_id
uint32 sequence
uint32 random_seed

float32[3] sigma_rad_s

uint16 fault_id

uint8 axis_mask
bool active
