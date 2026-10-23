# Raw and faulted allocated motor commands at the common PX4 actuator path.
# Fault order is delay -> bias -> maximum-output degradation.
# Current raw NaN/stop commands always remain NaN and cannot be revived by delay.

uint64 timestamp
uint64 timestamp_sample
uint64 delay_source_timestamp

uint32 bias_scenario_id
uint32 bias_definition_sequence
uint32 delay_scenario_id
uint32 delay_definition_sequence
uint32 max_output_scenario_id
uint32 max_output_definition_sequence
uint32 delay_us

float32 bias_norm
float32 max_output_norm

uint16 reversible_flags
uint16 raw_finite_mask
uint16 delay_applied_mask
uint16 bias_applied_mask
uint16 max_output_applied_mask
uint16 bias_fault_id
uint16 delay_fault_id
uint16 max_output_fault_id

uint8 motor_count
uint8 bias_motor_mask
uint8 delay_motor_mask
uint8 max_output_motor_mask
uint8 output_instance

bool bias_active
bool delay_active
bool max_output_active

float32[12] raw_control
float32[12] delayed_control
float32[12] faulted_control
