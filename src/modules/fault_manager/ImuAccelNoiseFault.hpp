#pragma once

#include "FaultInstance.hpp"

#include <cmath>
#include <cstddef>
#include <cstdint>

#include <lib/fault_injection/VectorEffectValidator.hpp>
#include <uORB/topics/fault_command.h>

namespace fault_injection
{

class ImuAccelNoiseFault final : public FaultInstance
{
public:
	const float *sigma_m_s2() const
	{
		return _sigma_m_s2;
	}

	float sigma_m_s2(std::size_t axis) const
	{
		return (axis < 3) ? _sigma_m_s2[axis] : 0.0f;
	}

	void record_applied(uint32_t count)
	{
		mark_applied(count);
	}

protected:
	FaultConfigureResult validate_effect(
		const FaultSpec &spec) const override
	{
		if (spec.target != fault_command_s::TARGET_IMU) {
			return FaultConfigureResult::InvalidTarget;
		}

		if (spec.mode != fault_command_s::MODE_ACCEL_NOISE) {
			return FaultConfigureResult::InvalidMode;
		}

		/*
		 * The first implementation targets the effective IMU channel supplied
		 * to each EKF2 object, not a named physical IMU instance.
		 */
		if (spec.instance_mask != 0) {
			return FaultConfigureResult::InvalidInstanceMask;
		}

		const VectorEffectValidation validation =
			validate_vector_effect<3>(
				spec.axis_mask,
				spec.vector,
				SelectedValueRule::StrictlyPositive,
				UnselectedValueRule::RequireZero);

		if (!validation.valid_mask) {
			return FaultConfigureResult::InvalidAxisMask;
		}

		if (!validation.valid_values) {
			return FaultConfigureResult::InvalidParameter;
		}

		/*
		 * MODE_ACCEL_NOISE uses vector[3], axis_mask, and random_seed.
		 * Delay and scalar parameters must remain neutral rather than being
		 * silently ignored. Every uint32 random_seed value, including zero,
		 * is accepted; the target-side generator will mix it into a valid
		 * per-definition internal state.
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

			_sigma_m_s2[axis] =
				((spec.axis_mask & axis_bit) != 0)
				? spec.vector[axis]
				: 0.0f;
		}
	}

	void reset_effect_to_neutral() override
	{
		_sigma_m_s2[0] = 0.0f;
		_sigma_m_s2[1] = 0.0f;
		_sigma_m_s2[2] = 0.0f;
	}

private:
	float _sigma_m_s2[3]{0.0f, 0.0f, 0.0f};
};

} // namespace fault_injection
