# Cumulative evidence that an EKF2 instance applied one GNSS velocity-noise definition.
#
# Each EKF2 instance owns one multi-instance publisher. The cumulative count is
# monotonic for one scenario_id + definition_sequence + fault_id definition and
# resets to zero when that instance begins a different definition.

uint64 timestamp                         # [us] PX4 publication time
uint64 timestamp_sample                  # [us] Last GNSS sample modified by this report, or 0 before any application

uint32 scenario_id                       # Scenario owning the applied definition
uint32 definition_sequence               # SET sequence that created the applied definition
uint32 cumulative_applied_count           # GNSS samples modified by this EKF2 instance

uint16 fault_id                          # Scenario-local fault identifier

uint8 estimator_instance                 # EKF2 estimator instance; 255 when not assigned
bool final_report                        # True when the cached definition is being deactivated or replaced
