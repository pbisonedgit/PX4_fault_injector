# Current effective runtime configuration for the IMU acceleration-bias injector.
#
# This is a dedicated latest-value effect snapshot, not the generic fault audit
# topic. The FAULT fault manager is the sole lifecycle authority and publishes
# this topic when the IMU acceleration-bias definition is configured, activated,
# cleared, or expired.
#
# When active is false, axis_mask and bias_m_s2 must be zero. EKF consumers must
# also default to an inactive zero-bias state before receiving the first sample.

uint64 timestamp                 # [us] PX4 time when this effect snapshot was published

uint32 scenario_id               # Scenario owning the current IMU bias definition, or 0 when empty
uint32 sequence                  # Command/lifecycle sequence associated with this snapshot

float32[3] bias_m_s2             # [m/s^2] Effective body-frame residual acceleration bias

uint16 fault_id                  # Scenario-local fault identifier, or 0 when empty

uint8 axis_mask                  # Active AXIS_* bitmask; zero whenever active is false
bool active                      # True only while the concrete fault object is ACTIVE
