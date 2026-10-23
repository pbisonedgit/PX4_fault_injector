/****************************************************************************
 *
 *   FAULT motor-output fault injection at the common allocated-command path.
 *
 ****************************************************************************/
#pragma once

#include <lib/fault_injection/FixedArray.hpp>
#include <cstddef>
#include <cstdint>

#include <lib/fault_injection/CumulativeApplicationTracker.hpp>

#include <uORB/Publication.hpp>
#include <uORB/PublicationMulti.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/topics/actuator_motors.h>
#include <uORB/topics/fault_motor_output_bias.h>
#include <uORB/topics/fault_motor_output_bias_applied.h>
#include <uORB/topics/fault_motor_output_delay.h>
#include <uORB/topics/fault_motor_output_delay_applied.h>
#include <uORB/topics/fault_motor_max_output_degradation.h>
#include <uORB/topics/fault_motor_max_output_degradation_applied.h>
#include <uORB/topics/fault_motor_output_measurement.h>

class MotorFaultInjector final
{
public:
	static constexpr uint8_t OutputInstance = 0;
	static constexpr std::size_t SelectableMotorCount = 8;
	static constexpr std::size_t HistoryCapacity = 512;
	static constexpr uint32_t MaximumDelayUs = 1000000U;

	void update();
	void apply(actuator_motors_s &motors, uint8_t motor_count);
	void print_status() const;

private:
	struct HistoryFrame {
		uint64_t timestamp{0};
		float control[actuator_motors_s::NUM_CONTROLS]{};
	};

	static bool selected(uint8_t motor_mask, std::size_t index);
	static bool values_differ(float lhs, float rhs);
	static float constrain_motor_command(float value, bool reversible);
	static float constrain_to_degraded_max(
		float value, bool reversible, float max_output_norm);

	void update_bias();
	void update_delay();
	void update_max_output();

	bool same_bias_definition(const fault_motor_output_bias_s &snapshot) const;
	bool same_delay_definition(const fault_motor_output_delay_s &snapshot) const;
	bool same_max_output_definition(
		const fault_motor_max_output_degradation_s &snapshot) const;

	bool publish_bias_feedback(bool final_report);
	bool publish_delay_feedback(bool final_report);
	bool publish_max_output_feedback(bool final_report);

	void record_bias_application(uint64_t timestamp_sample);
	void record_delay_application(uint64_t timestamp_sample);
	void record_max_output_application(uint64_t timestamp_sample);

	void store_history(const actuator_motors_s &raw);
	bool find_delayed_frame(uint64_t target_timestamp, HistoryFrame &frame) const;

	uORB::Subscription _bias_sub{ORB_ID(fault_motor_output_bias)};
	uORB::Subscription _delay_sub{ORB_ID(fault_motor_output_delay)};
	uORB::Subscription _max_output_sub{
		ORB_ID(fault_motor_max_output_degradation)};

	uORB::PublicationMulti<fault_motor_output_bias_applied_s>
		_bias_applied_pub{ORB_ID(fault_motor_output_bias_applied)};
	uORB::PublicationMulti<fault_motor_output_delay_applied_s>
		_delay_applied_pub{ORB_ID(fault_motor_output_delay_applied)};
	uORB::PublicationMulti<fault_motor_max_output_degradation_applied_s>
		_max_output_applied_pub{
			ORB_ID(fault_motor_max_output_degradation_applied)};
	uORB::Publication<fault_motor_output_measurement_s> _measurement_pub{
		ORB_ID(fault_motor_output_measurement)};

	fault_motor_output_bias_s _last_bias_snapshot{};
	fault_motor_output_delay_s _last_delay_snapshot{};
	fault_motor_max_output_degradation_s _last_max_output_snapshot{};

	fault_injection::CumulativeApplicationTracker _bias_tracker{100000ULL};
	fault_injection::CumulativeApplicationTracker _delay_tracker{100000ULL};
	fault_injection::CumulativeApplicationTracker _max_output_tracker{100000ULL};

	fault_injection::FixedArray<HistoryFrame, HistoryCapacity> _history{};
	std::size_t _history_head{0};
	std::size_t _history_count{0};

	float _bias_norm{0.0f};
	float _max_output_norm{1.0f};
	uint32_t _delay_us{0};
	uint8_t _bias_motor_mask{0};
	uint8_t _delay_motor_mask{0};
	uint8_t _max_output_motor_mask{0};

	uint32_t _bias_update_count{0};
	uint32_t _delay_update_count{0};
	uint32_t _max_output_update_count{0};
	uint32_t _delay_history_hit_count{0};
	uint32_t _delay_history_miss_count{0};
	uint32_t _measurement_publish_count{0};

	bool _bias_active{false};
	bool _delay_active{false};
	bool _max_output_active{false};
};
