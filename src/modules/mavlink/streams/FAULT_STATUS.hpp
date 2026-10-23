/****************************************************************************
 *
 * Copyright (c) 2026.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the conditions of the PX4
 * BSD-3-Clause licence are met.
 *
 ****************************************************************************/

/**
 * @file FAULT_STATUS.hpp
 *
 * Event-oriented MAVLink stream exporting fault_status uORB messages.
 *
 * This stream is passive instrumentation. It serializes generic lifecycle and
 * applied-count evidence already produced by fault_manager. It does not
 * configure, schedule, clear, query, or apply any fault.
 */

#pragma once

#if defined(MAVLINK_MSG_ID_FAULT_STATUS)

#include <uORB/Subscription.hpp>
#include <uORB/topics/fault_status.h>

class MavlinkStreamFaultStatus final : public MavlinkStream
{
public:
	static MavlinkStream *new_instance(Mavlink *mavlink)
	{
		return new MavlinkStreamFaultStatus(mavlink);
	}

	static constexpr const char *get_name_static()
	{
		return "FAULT_STATUS";
	}

	static constexpr uint16_t get_id_static()
	{
		return MAVLINK_MSG_ID_FAULT_STATUS;
	}

	const char *get_name() const override
	{
		return get_name_static();
	}

	uint16_t get_id() override
	{
		return get_id_static();
	}

	unsigned get_size() override
	{
		/* Event topic: queued updates cannot be predicted in advance. */
		return 0;
	}

private:
	explicit MavlinkStreamFaultStatus(Mavlink *mavlink) :
		MavlinkStream(mavlink)
	{
	}

	uORB::Subscription _fault_status_sub{ORB_ID(fault_status)};

	bool send() override
	{
		bool sent = false;

		static constexpr size_t MESSAGE_SIZE =
			MAVLINK_MSG_ID_FAULT_STATUS_LEN
			+ MAVLINK_NUM_NON_PAYLOAD_BYTES;

		int processed_updates = 0;

		while ((_mavlink->get_free_tx_buf() >= MESSAGE_SIZE)
		       && _fault_status_sub.updated()
		       && (processed_updates < fault_status_s::ORB_QUEUE_LENGTH)) {

			processed_updates++;

			const unsigned generation_before =
				_fault_status_sub.get_last_generation();

			fault_status_s status{};

			if (!_fault_status_sub.update(&status)) {
				continue;
			}

			const unsigned generation_after =
				_fault_status_sub.get_last_generation();

			if (generation_after != generation_before + 1) {
				PX4_ERR(
					"FAULT fault status lost, generation %u -> %u",
					generation_before,
					generation_after
				);
			}

			mavlink_fault_status_t message{};

			message.timestamp_us = status.timestamp;
			message.activation_time_us = status.activation_time_us;
			message.expiration_time_us = status.expiration_time_us;
			message.scenario_id = status.scenario_id;
			message.sequence = status.sequence;
			message.applied_count = status.applied_count;
			message.instance_mask = status.instance_mask;
			message.random_seed = status.random_seed;
			message.delay_us = status.delay_us;
			message.vector[0] = status.vector[0];
			message.vector[1] = status.vector[1];
			message.vector[2] = status.vector[2];
			message.scalar = status.scalar;
			message.fault_id = status.fault_id;
			message.target_system = status.target_system;
			message.target_component = status.target_component;
			message.protocol_version = status.protocol_version;
			message.target = status.target;
			message.mode = status.mode;
			message.state = status.state;
			message.axis_mask = status.axis_mask;

			mavlink_msg_fault_status_send_struct(
				_mavlink->get_channel(),
				&message
			);

			sent = true;
		}

		return sent;
	}
};

#endif // MAVLINK_MSG_ID_FAULT_STATUS
