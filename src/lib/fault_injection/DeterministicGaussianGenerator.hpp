#pragma once

#include "DefinitionIdentity.hpp"
#include "FixedArray.hpp"

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <math.h>

namespace fault_injection
{

template<std::size_t ChannelCount>
class DeterministicGaussianGenerator final
{
	static_assert(ChannelCount > 0,
		      "A Gaussian generator needs at least one channel");

public:
	void initialize(const RandomDefinitionIdentity &identity)
	{
		uint32_t base_seed =
			mix32(identity.random_seed ^ 0xA511E9B3U);

		base_seed =
			mix32(
				base_seed
				^ identity.definition.scenario_id);

		base_seed =
			mix32(
				base_seed
				^ identity.definition.definition_sequence);

		base_seed =
			mix32(
				base_seed
				^ static_cast<uint32_t>(
					identity.definition.fault_id));

		/*
		 * A zero domain deliberately performs no extra mixing so the
		 * validated IMU-acceleration sequence remains bit-for-bit unchanged.
		 */
		if (identity.stream_domain != 0U) {
			base_seed =
				mix32(
					base_seed
					^ identity.stream_domain);
		}

		for (std::size_t channel = 0;
		     channel < ChannelCount;
		     ++channel) {
			const uint32_t salt =
				channel_seed_salt(channel);

			uint32_t channel_seed =
				mix32(base_seed ^ salt);

			/*
			 * xorshift32 has one degenerate state. The stable channel salt
			 * is the deterministic fallback for the one zero hash result.
			 */
			if (channel_seed == 0U) {
				channel_seed = salt;
			}

			_state[channel] = channel_seed;
			_gaussian_spare[channel] = 0.0;
			_has_gaussian_spare[channel] = false;
		}

		_initialized = true;
		++_initialization_count;
	}

	void reset()
	{
		for (std::size_t channel = 0;
		     channel < ChannelCount;
		     ++channel) {
			_state[channel] = 0U;
			_gaussian_spare[channel] = 0.0;
			_has_gaussian_spare[channel] = false;
		}

		_initialized = false;
	}

	bool initialized() const
	{
		return _initialized;
	}

	uint32_t initialization_count() const
	{
		return _initialization_count;
	}

	double sample(std::size_t channel)
	{
		if (!_initialized || (channel >= ChannelCount)) {
			return static_cast<double>(NAN);
		}

		if (_has_gaussian_spare[channel]) {
			_has_gaussian_spare[channel] = false;
			return _gaussian_spare[channel];
		}

		/*
		 * Map xorshift32 output to the open interval (0, 1), then use a
		 * fixed Box-Muller transform. Every channel owns its state and spare
		 * value, so sampling one channel cannot shift another channel.
		 */
		constexpr double InvTwoPow32 =
			1.0 / 4294967296.0;
		constexpr double TwoPi =
			6.283185307179586476925286766559;

		const double uniform_1 =
			(static_cast<double>(
				 next_uint32(_state[channel]))
			 + 0.5)
			* InvTwoPow32;

		const double uniform_2 =
			(static_cast<double>(
				 next_uint32(_state[channel]))
			 + 0.5)
			* InvTwoPow32;

		const double radius =
			std::sqrt(-2.0 * std::log(uniform_1));
		const double angle = TwoPi * uniform_2;

		const double gaussian_0 =
			radius * std::cos(angle);
		const double gaussian_1 =
			radius * std::sin(angle);

		_gaussian_spare[channel] = gaussian_1;
		_has_gaussian_spare[channel] = true;
		return gaussian_0;
	}

private:
	static uint32_t mix32(uint32_t value)
	{
		value ^= value >> 16U;
		value *= 0x7FEB352DU;
		value ^= value >> 15U;
		value *= 0x846CA68BU;
		value ^= value >> 16U;
		return value;
	}

	static uint32_t next_uint32(uint32_t &state)
	{
		state ^= state << 13U;
		state ^= state >> 17U;
		state ^= state << 5U;
		return state;
	}

	static uint32_t channel_seed_salt(std::size_t channel)
	{
		/*
		 * The first three salts are the validated acceleration-noise salts.
		 * Extra channels receive stable derived salts for future targets.
		 */
		switch (channel) {
		case 0:
			return 0x243F6A88U;

		case 1:
			return 0x85A308D3U;

		case 2:
			return 0x13198A2EU;

		default: {
				const uint32_t channel_number =
					static_cast<uint32_t>(channel + 1U);

				uint32_t salt =
					mix32(
						0x9E3779B9U
						^ channel_number);

				if (salt == 0U) {
					salt =
						0x9E3779B9U
						^ channel_number;
				}

				return salt;
			}
		}
	}

	FixedArray<uint32_t, ChannelCount> _state{};
	FixedArray<double, ChannelCount> _gaussian_spare{};
	FixedArray<bool, ChannelCount> _has_gaussian_spare{};

	uint32_t _initialization_count{0};
	bool _initialized{false};
};

} // namespace fault_injection
