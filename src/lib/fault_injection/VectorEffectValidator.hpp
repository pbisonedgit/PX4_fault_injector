#pragma once

#include <cmath>
#include <cstddef>
#include <cstdint>

namespace fault_injection
{

enum class SelectedValueRule : uint8_t {
	Finite = 0,
	StrictlyPositive
};

enum class UnselectedValueRule : uint8_t {
	Ignore = 0,
	RequireZero
};

struct VectorEffectValidation
{
	bool valid_mask{false};
	bool valid_values{false};

	bool accepted() const
	{
		return valid_mask && valid_values;
	}
};

template<std::size_t Dimension>
VectorEffectValidation validate_vector_effect(
	uint32_t selection_mask,
	const float *values,
	SelectedValueRule selected_rule,
	UnselectedValueRule unselected_rule)
{
	static_assert(Dimension > 0,
		      "A vector effect needs at least one dimension");
	static_assert(Dimension < 32,
		      "The current selection mask supports at most 31 dimensions");

	constexpr uint32_t valid_selection_mask =
		(1U << Dimension) - 1U;

	VectorEffectValidation result{};
	result.valid_mask =
		(selection_mask != 0U)
		&& ((selection_mask & ~valid_selection_mask) == 0U);

	if (!result.valid_mask || (values == nullptr)) {
		return result;
	}

	for (std::size_t dimension = 0;
	     dimension < Dimension;
	     ++dimension) {
		const uint32_t dimension_bit =
			1U << dimension;

		if ((selection_mask & dimension_bit) != 0U) {
			if (!std::isfinite(values[dimension])) {
				return result;
			}

			if ((selected_rule
			     == SelectedValueRule::StrictlyPositive)
			    && !(values[dimension] > 0.0f)) {
				return result;
			}

		} else if ((unselected_rule
			    == UnselectedValueRule::RequireZero)
			   && (std::fpclassify(values[dimension])
			       != FP_ZERO)) {
			return result;
		}
	}

	result.valid_values = true;
	return result;
}

} // namespace fault_injection
