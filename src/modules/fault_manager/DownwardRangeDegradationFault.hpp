#pragma once

#include "FaultInstance.hpp"

#include <cmath>

#include <uORB/topics/fault_command.h>

namespace fault_injection
{

class DownwardRangeDegradationFault final : public FaultInstance
{
public:
	float effective_max_distance_m() const { return _effective_max_distance_m; }
	void record_applied(uint32_t count) { mark_applied(count); }

protected:
	FaultConfigureResult validate_effect(const FaultSpec &spec) const override
	{
		if (spec.target != fault_command_s::TARGET_DOWNWARD_RANGE) {
			return FaultConfigureResult::InvalidTarget;
		}
		if (spec.mode != fault_command_s::MODE_DOWNWARD_RANGE_DEGRADATION) {
			return FaultConfigureResult::InvalidMode;
		}
		if ((spec.instance_mask != 0) || (spec.axis_mask != 0)) {
			return (spec.instance_mask != 0)
			       ? FaultConfigureResult::InvalidInstanceMask
			       : FaultConfigureResult::InvalidAxisMask;
		}
		for (float value : spec.vector) {
			if (!std::isfinite(value) || (fpclassify(value) != FP_ZERO)) {
				return FaultConfigureResult::InvalidParameter;
			}
		}
		if ((spec.random_seed != 0) || (spec.delay_us != 0)
		    || !std::isfinite(spec.scalar) || !(spec.scalar > 0.0f)) {
			return FaultConfigureResult::InvalidParameter;
		}
		return FaultConfigureResult::Accepted;
	}

	void configure_effect(const FaultSpec &spec) override
	{
		_effective_max_distance_m = spec.scalar;
	}

	void reset_effect_to_neutral() override
	{
		_effective_max_distance_m = 0.0f;
	}

private:
	float _effective_max_distance_m{0.0f};
};

} // namespace fault_injection
