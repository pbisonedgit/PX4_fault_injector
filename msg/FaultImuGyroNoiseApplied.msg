# Cumulative evidence that an EKF2 instance applied the current IMU gyroscope-noise definition.
#
# Each EKF2 instance owns one multi-instance publisher. cumulative_applied_count
# is monotonic for one scenario_id + definition_sequence + fault_id definition
# and resets to zero when that instance begins a different definition.
#
# This topic is feedback to the FAULT fault manager. It is not an IMU data topic.

uint64 timestamp
uint64 timestamp_sample

uint32 scenario_id
uint32 definition_sequence
uint32 cumulative_applied_count

uint16 fault_id

uint8 estimator_instance
bool final_report
