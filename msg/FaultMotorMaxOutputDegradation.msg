# Active FAULT maximum motor-output degradation snapshot.
# max_output_norm is an absolute normalized-command cap in [0, 1).
# motor_mask bit0 selects Motor1 / actuator_motors.control[0], through bit7.

uint64 timestamp

uint32 scenario_id
uint32 sequence

float32 max_output_norm

uint16 fault_id

uint8 motor_mask
bool active
