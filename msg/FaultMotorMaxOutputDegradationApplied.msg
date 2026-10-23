# Cumulative application feedback for one FAULT motor fault definition.
# The counter increments once per actuator_motors frame only when this fault
# actually changes at least one selected motor command committed downstream.

uint64 timestamp
uint64 timestamp_sample

uint32 scenario_id
uint32 definition_sequence
uint32 cumulative_applied_count

uint16 fault_id

uint8 output_instance
bool final_report
