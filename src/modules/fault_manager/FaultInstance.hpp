#pragma once

#include <cstdint>
#include <limits>

namespace fault_injection
{

enum class FaultState : uint8_t {
	Empty = 0,
	Pending = 1,
	Active = 2,
	Cleared = 3,
	Expired = 4
};

enum class FaultConfigureResult : uint8_t {
	Accepted = 0,
	InvalidTarget,
	InvalidMode,
	InvalidInstanceMask,
	InvalidAxisMask,
	InvalidDuration,
	InvalidParameter,
	InternalError
};

struct FaultSpec {
	uint32_t scenario_id{0};
	uint32_t sequence{0};
	uint32_t start_delay_ms{0};
	uint32_t duration_ms{0};
	uint32_t random_seed{0};
	uint32_t delay_us{0};
	uint32_t instance_mask{0};

	float vector[3]{};
	float scalar{0.0f};

	uint16_t fault_id{0};

	uint8_t source_system{0};
	uint8_t source_component{0};
	uint8_t target_system{0};
	uint8_t target_component{0};
	uint8_t protocol_version{0};
	uint8_t target{0};
	uint8_t mode{0};
	uint8_t axis_mask{0};
};

class FaultInstance
{
public:
	static constexpr uint64_t InfiniteTime =
		std::numeric_limits<uint64_t>::max();

	FaultInstance() = default;
	virtual ~FaultInstance() = default;

	FaultInstance(const FaultInstance &) = delete;
	FaultInstance &operator=(const FaultInstance &) = delete;
	FaultInstance(FaultInstance &&) = delete;
	FaultInstance &operator=(FaultInstance &&) = delete;

	FaultConfigureResult configure(const FaultSpec &spec, uint64_t now_us);
	bool clear(uint64_t now_us);
	bool update(uint64_t now_us);

	bool has_definition() const
	{
		return _has_definition;
	}

	bool is_active() const
	{
		return _state == FaultState::Active;
	}

	bool matches(uint32_t scenario_id, uint16_t fault_id) const
	{
		return _has_definition
		       && (_spec.scenario_id == scenario_id)
		       && (_spec.fault_id == fault_id);
	}

	FaultState state() const
	{
		return _state;
	}

	uint8_t state_code() const
	{
		return static_cast<uint8_t>(_state);
	}

	const FaultSpec &spec() const
	{
		return _spec;
	}

	uint64_t received_time_us() const
	{
		return _received_time_us;
	}

	uint64_t activation_time_us() const
	{
		return _activation_time_us;
	}

	uint64_t expiration_time_us() const
	{
		return _expiration_time_us;
	}

	uint64_t last_transition_time_us() const
	{
		return _last_transition_time_us;
	}

	uint32_t applied_count() const
	{
		return _applied_count;
	}

protected:
	virtual FaultConfigureResult validate_effect(
		const FaultSpec &spec) const = 0;

	virtual void configure_effect(const FaultSpec &spec) = 0;
	virtual void reset_effect_to_neutral() = 0;

	virtual void on_activate()
	{
	}

	virtual void on_clear()
	{
	}

	virtual void on_expire()
	{
	}

	void mark_applied(uint32_t count = 1);

private:
	static uint64_t saturating_add(uint64_t lhs, uint64_t rhs);

	FaultSpec _spec{};

	uint64_t _received_time_us{0};
	uint64_t _activation_time_us{InfiniteTime};
	uint64_t _expiration_time_us{InfiniteTime};
	uint64_t _last_transition_time_us{0};

	uint32_t _applied_count{0};

	FaultState _state{FaultState::Empty};
	bool _has_definition{false};
};

} // namespace fault_injection
