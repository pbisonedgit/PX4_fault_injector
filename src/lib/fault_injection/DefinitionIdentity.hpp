#pragma once

#include <cstdint>

namespace fault_injection
{

struct DefinitionIdentity
{
	constexpr DefinitionIdentity() :
		scenario_id(0),
		definition_sequence(0),
		fault_id(0)
	{
	}

	constexpr DefinitionIdentity(
		uint32_t scenario_id_in,
		uint32_t definition_sequence_in,
		uint16_t fault_id_in) :
		scenario_id(scenario_id_in),
		definition_sequence(definition_sequence_in),
		fault_id(fault_id_in)
	{
	}

	uint32_t scenario_id;
	uint32_t definition_sequence;
	uint16_t fault_id;

	constexpr bool operator==(const DefinitionIdentity &other) const
	{
		return (scenario_id == other.scenario_id)
		       && (definition_sequence == other.definition_sequence)
		       && (fault_id == other.fault_id);
	}

	constexpr bool operator!=(const DefinitionIdentity &other) const
	{
		return !(*this == other);
	}
};

constexpr DefinitionIdentity make_definition_identity(
	uint32_t scenario_id,
	uint32_t definition_sequence,
	uint16_t fault_id)
{
	return DefinitionIdentity{
		scenario_id,
		definition_sequence,
		fault_id
	};
}

struct RandomDefinitionIdentity
{
	DefinitionIdentity definition{};
	uint32_t random_seed{0};

	/*
	 * Zero preserves the original IMU-acceleration noise sequence. Future
	 * targets may provide a stable non-zero domain without changing the
	 * lifecycle identity used by CLEAR, QUERY, and applied feedback.
	 */
	uint32_t stream_domain{0};
};

} // namespace fault_injection
