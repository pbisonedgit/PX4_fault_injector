#pragma once

#include "FaultInstance.hpp"

#include <cmath>
#include <cstddef>
#include <cstdint>

#include <lib/fault_injection/VectorEffectValidator.hpp>
#include <uORB/topics/fault_command.h>

namespace fault_injection
{

class MagBiasFault final : public FaultInstance
{
public:
	const float *bias_ga() const
	{
		return _bias_ga;
	}

	float bias_ga(std::size_t axis) const
	{
		return (axis < 3) ? _bias_ga[axis] : 0.0f;
	}

	void record_applied(uint32_t count)
	{
		mark_applied(count);
	}

protected:
	FaultConfigureResult validate_effect(
		const FaultSpec &spec) const override
	{
		if (spec.target != fault_command_s::TARGET_MAG) {
			return FaultConfigureResult::InvalidTarget;
		}

		if (spec.mode != fault_command_s::MODE_MAG_BIAS) {
			return FaultConfigureResult::InvalidMode;
		}

		/*
		 * This fault targets the effective magnetometer channel supplied to
		 * each EKF2 object, not a named physical magnetometer instance.
		 */
		if (spec.instance_mask != 0) {
			return FaultConfigureResult::InvalidInstanceMask;
		}

		const VectorEffectValidation validation =
			validate_vector_effect<3>(
				spec.axis_mask,
				spec.vector,
				SelectedValueRule::Finite,
				UnselectedValueRule::RequireZero);

		if (!validation.valid_mask) {
			return FaultConfigureResult::InvalidAxisMask;
		}

		if (!validation.valid_values) {
			return FaultConfigureResult::InvalidParameter;
		}

		/*
		 * MODE_MAG_BIAS uses vector[3] only. Delay and scalar parameters
		 * must remain neutral rather than being silently ignored.
		 */
		if ((spec.delay_us != 0)
		    || !std::isfinite(spec.scalar)
		    || (std::fpclassify(spec.scalar) != FP_ZERO)) {
			return FaultConfigureResult::InvalidParameter;
		}

		return FaultConfigureResult::Accepted;
	}

	void configure_effect(const FaultSpec &spec) override
	{
		for (std::size_t axis = 0; axis < 3; ++axis) {
			const uint8_t axis_bit =
				static_cast<uint8_t>(1U << axis);

			_bias_ga[axis] =
				((spec.axis_mask & axis_bit) != 0)
				? spec.vector[axis]
				: 0.0f;
		}
	}

	void reset_effect_to_neutral() override
	{
		_bias_ga[0] = 0.0f;
		_bias_ga[1] = 0.0f;
		_bias_ga[2] = 0.0f;
	}

private:
	float _bias_ga[3]{0.0f, 0.0f, 0.0f};
};

} // namespace fault_injection
