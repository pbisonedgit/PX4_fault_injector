#include "FaultInstance.hpp"

namespace fault_injection
{

uint64_t FaultInstance::saturating_add(uint64_t lhs, uint64_t rhs)
{
	if (lhs > InfiniteTime - rhs) {
		return InfiniteTime;
	}

	return lhs + rhs;
}

FaultConfigureResult FaultInstance::configure(
	const FaultSpec &spec,
	uint64_t now_us)
{
	const FaultConfigureResult validation = validate_effect(spec);

	if (validation != FaultConfigureResult::Accepted) {
		return validation;
	}

	reset_effect_to_neutral();
	configure_effect(spec);

	_spec = spec;
	_received_time_us = now_us;
	_applied_count = 0;
	_has_definition = true;

	const uint64_t start_delay_us =
		static_cast<uint64_t>(spec.start_delay_ms) * 1000ULL;

	_activation_time_us = saturating_add(now_us, start_delay_us);

	if (spec.duration_ms == 0) {
		_expiration_time_us = InfiniteTime;

	} else {
		const uint64_t duration_us =
			static_cast<uint64_t>(spec.duration_ms) * 1000ULL;

		_expiration_time_us =
			saturating_add(_activation_time_us, duration_us);
	}

	_last_transition_time_us = now_us;

	if (_activation_time_us <= now_us) {
		_state = FaultState::Active;
		on_activate();

	} else {
		_state = FaultState::Pending;
	}

	return FaultConfigureResult::Accepted;
}

bool FaultInstance::clear(uint64_t now_us)
{
	if (!_has_definition) {
		return false;
	}

	if ((_state == FaultState::Cleared)
	    || (_state == FaultState::Expired)) {
		return false;
	}

	reset_effect_to_neutral();
	_state = FaultState::Cleared;
	_last_transition_time_us = now_us;
	on_clear();

	/*
	 * Preserve the accepted activation and expiration times for status and
	 * ULog evidence. The explicit Cleared state prevents further application.
	 */
	return true;
}

bool FaultInstance::update(uint64_t now_us)
{
	if (!_has_definition) {
		return false;
	}

	if (_state == FaultState::Pending) {
		if ((_expiration_time_us != InfiniteTime)
		    && (now_us >= _expiration_time_us)) {
			reset_effect_to_neutral();
			_state = FaultState::Expired;
			_last_transition_time_us = now_us;
			on_expire();
			return true;
		}

		if (now_us >= _activation_time_us) {
			_state = FaultState::Active;
			_last_transition_time_us = now_us;
			on_activate();
			return true;
		}
	}

	if ((_state == FaultState::Active)
	    && (_expiration_time_us != InfiniteTime)
	    && (now_us >= _expiration_time_us)) {
		reset_effect_to_neutral();
		_state = FaultState::Expired;
		_last_transition_time_us = now_us;
		on_expire();
		return true;
	}

	return false;
}

void FaultInstance::mark_applied(uint32_t count)
{
	const uint32_t maximum = UINT32_MAX;

	if (count > maximum - _applied_count) {
		_applied_count = maximum;

	} else {
		_applied_count += count;
	}
}

} // namespace fault_injection
