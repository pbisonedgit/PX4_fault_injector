#pragma once

#include "FaultInstance.hpp"

namespace fault_injection
{

class NoEffectFault final : public FaultInstance
{
protected:
	FaultConfigureResult validate_effect(
		const FaultSpec &spec) const override
	{
		if (spec.target != 0) {
			return FaultConfigureResult::InvalidTarget;
		}

		if (spec.mode != 0) {
			return FaultConfigureResult::InvalidMode;
		}

		if (spec.instance_mask != 0) {
			return FaultConfigureResult::InvalidInstanceMask;
		}

		if (spec.axis_mask != 0) {
			return FaultConfigureResult::InvalidAxisMask;
		}

		return FaultConfigureResult::Accepted;
	}

	void configure_effect(const FaultSpec &spec) override
	{
		(void)spec;
	}

	void reset_effect_to_neutral() override
	{
		// Intentionally no effect state to reset.
	}
};

} // namespace fault_injection
