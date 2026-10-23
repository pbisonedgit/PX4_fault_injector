# Event-oriented evidence for one selected downward-facing distance_sensor sample.
# The event records source validity, range-degradation gating and the exact
# finite value supplied to EKF2. If no usable target is detected,
# faulted_distance_m is positive infinity and no range sample is supplied.

uint8 AVAILABILITY_VALID = 0
uint8 AVAILABILITY_NON_FINITE = 3
uint8 AVAILABILITY_RAW_BELOW_MIN = 4
uint8 AVAILABILITY_RAW_ABOVE_MAX = 5
uint8 AVAILABILITY_DEGRADED_RANGE = 6

uint64 timestamp
uint64 timestamp_sample

uint32 device_id
uint32 bias_scenario_id
uint32 bias_definition_sequence
uint32 noise_scenario_id
uint32 noise_definition_sequence
uint32 degradation_scenario_id
uint32 degradation_definition_sequence

float32 raw_distance_m
float32 faulted_distance_m
float32 bias_effect_m
float32 noise_effect_m
float32 min_distance_m
float32 nominal_max_distance_m
float32 effective_max_distance_m
float32 variance_m2

uint16 bias_fault_id
uint16 noise_fault_id
uint16 degradation_fault_id

int8 signal_quality
uint8 sensor_type
uint8 orientation
uint8 estimator_instance
uint8 availability_reason

bool source_valid
bool measurement_available
bool no_detection
bool bias_active
bool noise_active
bool range_degradation_active
bool bias_applied
bool noise_applied
bool range_degradation_applied
