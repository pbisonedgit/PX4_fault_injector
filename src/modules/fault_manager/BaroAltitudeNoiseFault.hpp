#pragma once

#include "FaultInstance.hpp"

#include <cmath>

#include <uORB/topics/fault_command.h>

namespace fault_injection
{

class BaroAltitudeNoiseFault final : public FaultInstance
{
public:
	float sigma_m() const { return _sigma_m; }
	void record_applied(uint32_t count) { mark_applied(count); }

protected:
	FaultConfigureResult validate_effect(const FaultSpec &spec) const override
	{
		if (spec.target != fault_command_s::TARGET_BAROMETER) {
			return FaultConfigureResult::InvalidTarget;
		}
		if (spec.mode != fault_command_s::MODE_BARO_ALTITUDE_NOISE) {
			return FaultConfigureResult::InvalidMode;
		}
		if (spec.instance_mask != 0) {
			return FaultConfigureResult::InvalidInstanceMask;
		}
		if (spec.axis_mask != 0) {
			return FaultConfigureResult::InvalidAxisMask;
		}
		for (float value : spec.vector) {
			if (!std::isfinite(value) || (std::fpclassify(value) != FP_ZERO)) {
				return FaultConfigureResult::InvalidParameter;
			}
		}
		if ((spec.delay_us != 0) || !std::isfinite(spec.scalar)
		    || !(spec.scalar > 0.0f)) {
			return FaultConfigureResult::InvalidParameter;
		}
		return FaultConfigureResult::Accepted;
	}

	void configure_effect(const FaultSpec &spec) override { _sigma_m = spec.scalar; }
	void reset_effect_to_neutral() override { _sigma_m = 0.0f; }

private:
	float _sigma_m{0.0f};
};

} // namespace fault_injection
