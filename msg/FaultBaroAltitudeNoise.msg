# Current manager-owned runtime configuration for deterministic barometric-
# altitude noise. sigma_m is a strictly positive standard deviation in metres.
# When active is false, random_seed and sigma_m must be zero.

uint64 timestamp
uint32 scenario_id
uint32 sequence
uint32 random_seed
float32 sigma_m
uint16 fault_id
bool active
