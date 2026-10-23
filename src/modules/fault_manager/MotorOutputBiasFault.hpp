#pragma once

#include "FaultInstance.hpp"

#include <cmath>

#include <uORB/topics/fault_command.h>

namespace fault_injection
{

class MotorOutputBiasFault final : public FaultInstance
{
public:
	float bias_norm() const { return _bias_norm; }
	uint8_t motor_mask() const { return spec().axis_mask; }
	void record_applied(uint32_t count) { mark_applied(count); }

protected:
	FaultConfigureResult validate_effect(const FaultSpec &spec) const override
	{
		if (spec.target != fault_command_s::TARGET_MOTOR) {
			return FaultConfigureResult::InvalidTarget;
		}
		if (spec.mode != fault_command_s::MODE_MOTOR_OUTPUT_BIAS) {
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
		    || !(std::fabs(spec.scalar) <= 1.0f)
		    || (std::fpclassify(spec.scalar) == FP_ZERO)) {
			return FaultConfigureResult::InvalidParameter;
		}
		return FaultConfigureResult::Accepted;
	}

	void configure_effect(const FaultSpec &spec) override { _bias_norm = spec.scalar; }
	void reset_effect_to_neutral() override { _bias_norm = 0.0f; }

private:
	float _bias_norm{0.0f};
};

} // namespace fault_injection
