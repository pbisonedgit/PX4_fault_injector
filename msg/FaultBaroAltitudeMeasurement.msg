# Event-oriented evidence for one vehicle_air_data sample at the exact EKF2
# barometer-input convergence point. raw_altitude_m is copied from
# vehicle_air_data.baro_alt_meter. faulted_altitude_m is the value supplied to
# EKF2 after bias then deterministic noise. No pressure, temperature, density,
# device, calibration or reset metadata is faulted.

uint64 timestamp
uint64 timestamp_sample

uint32 device_id
uint32 bias_scenario_id
uint32 bias_definition_sequence
uint32 noise_scenario_id
uint32 noise_definition_sequence

float32 raw_altitude_m
float32 faulted_altitude_m
float32 bias_effect_m
float32 noise_effect_m
float32 rho

uint16 bias_fault_id
uint16 noise_fault_id

uint8 calibration_count
uint8 estimator_instance
bool reset
bool source_finite
bool bias_active
bool noise_active
bool bias_applied
bool noise_applied
