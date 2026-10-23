# Cumulative evidence that an EKF2 instance applied the current IMU acceleration-bias definition.
#
# Each EKF2 instance owns one multi-instance publisher. cumulative_applied_count
# is monotonic for one scenario_id + definition_sequence + fault_id definition
# and resets to zero when that instance begins a different definition.
#
# This topic is feedback to the FAULT fault manager. It is not an IMU data topic.

uint64 timestamp                         # [us] PX4 publication time
uint64 timestamp_sample                  # [us] Last IMU sample modified by this cumulative report, or 0 before any application

uint32 scenario_id                       # Scenario owning the applied definition
uint32 definition_sequence               # SET sequence that created the applied definition
uint32 cumulative_applied_count           # Samples modified by this EKF2 instance for this definition

uint16 fault_id                          # Scenario-local fault identifier

uint8 estimator_instance                 # EKF2 estimator instance; 255 when not assigned
bool final_report                        # True when the cached definition is being deactivated or replaced
