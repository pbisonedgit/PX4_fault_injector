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
 * @file FAULT_ACK.hpp
 *
 * Event-oriented MAVLink stream exporting fault_ack uORB messages.
 *
 * This stream only serializes acknowledgements already produced by
 * fault_manager. It does not validate commands, create acknowledgements,
 * store faults, schedule faults, or modify flight data.
 */

#pragma once

#if defined(MAVLINK_MSG_ID_FAULT_ACK)

#include <uORB/Subscription.hpp>
#include <uORB/topics/fault_ack.h>

class MavlinkStreamFaultAck final : public MavlinkStream
{
public:
	static MavlinkStream *new_instance(Mavlink *mavlink)
	{
		return new MavlinkStreamFaultAck(mavlink);
	}

	static constexpr const char *get_name_static()
	{
		return "FAULT_ACK";
	}

	static constexpr uint16_t get_id_static()
	{
		return MAVLINK_MSG_ID_FAULT_ACK;
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
		/*
		 * fault_ack is an event topic. The number of queued
		 * acknowledgements waiting at a scheduler iteration cannot be
		 * predicted in advance.
		 */
		return 0;
	}

private:
	explicit MavlinkStreamFaultAck(Mavlink *mavlink) :
		MavlinkStream(mavlink)
	{
	}

	uORB::Subscription _fault_ack_sub{ORB_ID(fault_ack)};

	bool send() override
	{
		bool sent = false;

		static constexpr size_t MESSAGE_SIZE =
			MAVLINK_MSG_ID_FAULT_ACK_LEN
			+ MAVLINK_NUM_NON_PAYLOAD_BYTES;

		int processed_updates = 0;

		/*
		 * Drain all acknowledgements currently available to this subscriber,
		 * up to the uORB queue length. Other subscribers retain independent
		 * generation counters.
		 */
		while ((_mavlink->get_free_tx_buf() >= MESSAGE_SIZE)
		       && _fault_ack_sub.updated()
		       && (processed_updates < fault_ack_s::ORB_QUEUE_LENGTH)) {

			processed_updates++;

			const unsigned generation_before =
				_fault_ack_sub.get_last_generation();

			fault_ack_s ack{};

			if (!_fault_ack_sub.update(&ack)) {
				continue;
			}

			const unsigned generation_after =
				_fault_ack_sub.get_last_generation();

			if (generation_after != generation_before + 1) {
				PX4_ERR(
					"FAULT fault ack lost, generation %u -> %u",
					generation_before,
					generation_after
				);
			}

			mavlink_fault_ack_t message{};

			message.timestamp_us = ack.timestamp;
			message.scenario_id = ack.scenario_id;
			message.sequence = ack.sequence;
			message.fault_id = ack.fault_id;
			message.target_system = ack.target_system;
			message.target_component = ack.target_component;
			message.protocol_version = ack.protocol_version;
			message.result = ack.result;
			message.reason = ack.reason;
			message.state = ack.state;

			mavlink_msg_fault_ack_send_struct(
				_mavlink->get_channel(),
				&message
			);

			sent = true;
		}

		return sent;
	}
};

#endif // MAVLINK_MSG_ID_FAULT_ACK
