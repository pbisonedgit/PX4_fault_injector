#pragma once

#include "DefinitionIdentity.hpp"

#include <cstdint>

namespace fault_injection
{

struct CumulativeFeedbackBaseline
{
	DefinitionIdentity identity{};
	uint32_t cumulative_count{0};
	bool valid{false};
};

struct CumulativeFeedbackResult
{
	bool accepted{false};
	uint32_t delta{0};
};

class CumulativeFeedbackAggregator final
{
public:
	static CumulativeFeedbackResult consume(
		const DefinitionIdentity &expected_identity,
		const DefinitionIdentity &received_identity,
		uint32_t cumulative_count,
		CumulativeFeedbackBaseline &baseline)
	{
		CumulativeFeedbackResult result{};

		if (received_identity != expected_identity) {
			return result;
		}

		const bool same_definition =
			baseline.valid
			&& (baseline.identity
			    == received_identity);

		if (!same_definition) {
			result.delta = cumulative_count;

		} else if (cumulative_count
			   >= baseline.cumulative_count) {
			result.delta =
				cumulative_count
				- baseline.cumulative_count;

		} else {
			/*
			 * The publisher restarted while the same definition remained
			 * active, so its cumulative counter restarted from zero.
			 */
			result.delta = cumulative_count;
		}

		baseline.identity = received_identity;
		baseline.cumulative_count = cumulative_count;
		baseline.valid = true;

		result.accepted = true;
		return result;
	}
};

} // namespace fault_injection
