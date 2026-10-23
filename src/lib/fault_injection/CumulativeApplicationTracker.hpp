#pragma once

#include "DefinitionIdentity.hpp"

#include <cstdint>

namespace fault_injection
{

class CumulativeApplicationTracker final
{
public:
	explicit constexpr CumulativeApplicationTracker(
		uint64_t feedback_period_us = 100000ULL) :
		_feedback_period_us(feedback_period_us)
	{
	}

	void begin_definition(const DefinitionIdentity &identity)
	{
		_identity = identity;
		_cumulative_count = 0;
		_last_timestamp_sample = 0;
		_last_feedback_publish_time = 0;
		_has_definition = true;
	}

	void reset()
	{
		_identity = {};
		_cumulative_count = 0;
		_last_timestamp_sample = 0;
		_last_feedback_publish_time = 0;
		_has_definition = false;
	}

	bool has_definition() const
	{
		return _has_definition;
	}

	bool same_definition(const DefinitionIdentity &identity) const
	{
		return _has_definition && (_identity == identity);
	}

	const DefinitionIdentity &identity() const
	{
		return _identity;
	}

	void record_application(uint64_t timestamp_sample)
	{
		const uint32_t maximum = UINT32_MAX;

		if (_cumulative_count < maximum) {
			++_cumulative_count;
		}

		_last_timestamp_sample = timestamp_sample;
	}

	bool feedback_due(uint64_t now_us) const
	{
		return (_last_feedback_publish_time == 0)
		       || (now_us - _last_feedback_publish_time
			   >= _feedback_period_us);
	}

	void mark_feedback_published(uint64_t now_us)
	{
		_last_feedback_publish_time = now_us;
	}

	uint32_t cumulative_count() const
	{
		return _cumulative_count;
	}

	uint64_t last_timestamp_sample() const
	{
		return _last_timestamp_sample;
	}

	uint64_t last_feedback_publish_time() const
	{
		return _last_feedback_publish_time;
	}

	uint64_t feedback_period_us() const
	{
		return _feedback_period_us;
	}

private:
	DefinitionIdentity _identity{};

	const uint64_t _feedback_period_us;

	uint64_t _last_timestamp_sample{0};
	uint64_t _last_feedback_publish_time{0};

	uint32_t _cumulative_count{0};
	bool _has_definition{false};
};

} // namespace fault_injection
