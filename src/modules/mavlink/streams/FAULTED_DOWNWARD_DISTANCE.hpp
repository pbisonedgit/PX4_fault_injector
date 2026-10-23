/****************************************************************************
 *
 * Copyright (c) 2026.
 *
 ****************************************************************************/
#pragma once

#if defined(MAVLINK_MSG_ID_FAULTED_DOWNWARD_DISTANCE)

#include <uORB/Subscription.hpp>
#include <uORB/topics/faulted_downward_distance.h>

class MavlinkStreamFaultedDownwardDistance final : public MavlinkStream
{
public:
	static MavlinkStream *new_instance(Mavlink *mavlink)
	{
		return new MavlinkStreamFaultedDownwardDistance(mavlink);
	}

	static constexpr const char *get_name_static()
	{
		return "FAULTED_DOWNWARD_DISTANCE";
	}

	static constexpr uint16_t get_id_static()
	{
		return MAVLINK_MSG_ID_FAULTED_DOWNWARD_DISTANCE;
	}

	const char *get_name() const override { return get_name_static(); }
	uint16_t get_id() override { return get_id_static(); }

	unsigned get_size() override
	{
		return _downward_distance_sub.advertised()
		       ? MAVLINK_MSG_ID_FAULTED_DOWNWARD_DISTANCE_LEN
		       + MAVLINK_NUM_NON_PAYLOAD_BYTES : 0;
	}

private:
	explicit MavlinkStreamFaultedDownwardDistance(Mavlink *mavlink) :
		MavlinkStream(mavlink)
	{
	}

	uORB::Subscription _downward_distance_sub{ORB_ID(faulted_downward_distance), 0};

	bool send() override
	{
		faulted_downward_distance_s source{};
		if (!_downward_distance_sub.update(&source)) {
			return false;
		}

		mavlink_faulted_downward_distance_t message{};
		message.timestamp_us = source.timestamp;
		message.timestamp_sample_us = source.timestamp_sample;
		message.sample_age_us = source.sample_age_us;
		message.device_id = source.device_id;
		message.bias_scenario_id = source.bias_scenario_id;
		message.bias_definition_sequence = source.bias_definition_sequence;
		message.noise_scenario_id = source.noise_scenario_id;
		message.noise_definition_sequence = source.noise_definition_sequence;
		message.degradation_scenario_id = source.degradation_scenario_id;
		message.degradation_definition_sequence = source.degradation_definition_sequence;
		message.raw_distance_m = source.raw_distance_m;
		message.faulted_distance_m = source.faulted_distance_m;
		message.bias_effect_m = source.bias_effect_m;
		message.noise_effect_m = source.noise_effect_m;
		message.min_distance_m = source.min_distance_m;
		message.nominal_max_distance_m = source.nominal_max_distance_m;
		message.effective_max_distance_m = source.effective_max_distance_m;
		message.variance_m2 = source.variance_m2;
		message.bias_fault_id = source.bias_fault_id;
		message.noise_fault_id = source.noise_fault_id;
		message.degradation_fault_id = source.degradation_fault_id;
		message.signal_quality = source.signal_quality;
		message.sensor_type = source.sensor_type;
		message.orientation = source.orientation;
		message.estimator_instance = source.estimator_instance;
		message.availability_reason = source.availability_reason;
		message.source_present = source.source_present;
		message.source_fresh = source.source_fresh;
		message.source_valid = source.source_valid;
		message.measurement_available = source.measurement_available;
		message.no_detection = source.no_detection;
		message.bias_active = source.bias_active;
		message.noise_active = source.noise_active;
		message.range_degradation_active = source.range_degradation_active;
		message.bias_applied = source.bias_applied;
		message.noise_applied = source.noise_applied;
		message.range_degradation_applied = source.range_degradation_applied;

		mavlink_msg_faulted_downward_distance_send_struct(
			_mavlink->get_channel(), &message);
		return true;
	}
};

#endif // MAVLINK_MSG_ID_FAULTED_DOWNWARD_DISTANCE
