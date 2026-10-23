#pragma once

#include "FaultInstance.hpp"

#include <cmath>

#include <uORB/topics/fault_command.h>

namespace fault_injection
{

class MotorMaxOutputDegradationFault final : public FaultInstance
{
public:
	float max_output_norm() const { return _max_output_norm; }
	uint8_t motor_mask() const { return spec().axis_mask; }
	void record_applied(uint32_t count) { mark_applied(count); }

protected:
	FaultConfigureResult validate_effect(const FaultSpec &spec) const override
	{
		if (spec.target != fault_command_s::TARGET_MOTOR) {
			return FaultConfigureResult::InvalidTarget;
		}
		if (spec.mode != fault_command_s::MODE_MOTOR_MAX_OUTPUT_DEGRADATION) {
			return FaultConfigureResult::InvalidMode;
		}
		if (spec.instance_mask != 0) {
			return FaultConfigureResult::InvalidInstanceMask;
		}
		if (spec.axis_mask == 0) {
			return FaultConfigureResult::InvalidAxisMask;
		}
		for (float value : spec.vector) {
			if (!std::isfinite(value) || (std::fpclassify(value) != FP_ZERO)) {
				return FaultConfigureResult::InvalidParameter;
			}
		}
		if ((spec.random_seed != 0) || (spec.delay_us != 0)
		    || !std::isfinite(spec.scalar)
		    || !(spec.scalar >= 0.0f)
		    || !(spec.scalar < 1.0f)) {
			return FaultConfigureResult::InvalidParameter;
		}
		return FaultConfigureResult::Accepted;
	}

	void configure_effect(const FaultSpec &spec) override { _max_output_norm = spec.scalar; }
	void reset_effect_to_neutral() override { _max_output_norm = 1.0f; }

private:
	float _max_output_norm{1.0f};
};

} // namespace fault_injection
