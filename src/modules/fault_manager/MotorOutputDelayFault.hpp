#pragma once

#include "FaultInstance.hpp"

#include <cmath>

#include <uORB/topics/fault_command.h>

namespace fault_injection
{

class MotorOutputDelayFault final : public FaultInstance
{
public:
	static constexpr uint32_t MaximumDelayUs = 1000000U;

	uint32_t delay_us() const { return _delay_us; }
	uint8_t motor_mask() const { return spec().axis_mask; }
	void record_applied(uint32_t count) { mark_applied(count); }

protected:
	FaultConfigureResult validate_effect(const FaultSpec &spec) const override
	{
		if (spec.target != fault_command_s::TARGET_MOTOR) {
			return FaultConfigureResult::InvalidTarget;
		}
		if (spec.mode != fault_command_s::MODE_MOTOR_OUTPUT_DELAY) {
			return FaultConfigureResult::InvalidMode;
		}
		if (spec.instance_mask != 0) {
			return FaultConfigureResult::InvalidInstanceMask;
		}
		if (spec.axis_mask == 0) {
			return FaultConfigureResult::InvalidAxisMask;
		}
		for (float value : spec.vector) {
			if (!std::isfinite(value) || (fpclassify(value) != FP_ZERO)) {
				return FaultConfigureResult::InvalidParameter;
			}
		}
		if ((spec.random_seed != 0)
		    || !std::isfinite(spec.scalar)
		    || (fpclassify(spec.scalar) != FP_ZERO)
		    || (spec.delay_us == 0)
		    || (spec.delay_us > MaximumDelayUs)) {
			return FaultConfigureResult::InvalidParameter;
		}
		return FaultConfigureResult::Accepted;
	}

	void configure_effect(const FaultSpec &spec) override { _delay_us = spec.delay_us; }
	void reset_effect_to_neutral() override { _delay_us = 0; }

private:
	uint32_t _delay_us{0};
};

} // namespace fault_injection
