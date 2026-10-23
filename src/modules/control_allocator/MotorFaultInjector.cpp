/****************************************************************************
 *
 *   FAULT motor-output fault injection at the common allocated-command path.
 *
 ****************************************************************************/
#include "MotorFaultInjector.hpp"

#include <float.h>
#include <cmath>

#include <drivers/drv_hrt.h>
#include <mathlib/math/Functions.hpp>
#include <px4_platform_common/log.h>

bool MotorFaultInjector::selected(uint8_t motor_mask, std::size_t index)
{
	return (index < SelectableMotorCount)
	       && ((motor_mask & (1u << index)) != 0);
}

bool MotorFaultInjector::values_differ(float lhs, float rhs)
{
	const bool lhs_finite = std::isfinite(lhs);
	const bool rhs_finite = std::isfinite(rhs);

	if (lhs_finite != rhs_finite) {
		return true;
	}

	if (!lhs_finite) {
		return false;
	}

	return std::fabs(lhs - rhs) > 1e-6f;
}

float MotorFaultInjector::constrain_motor_command(
	float value, bool reversible)
{
	return reversible
	       ? math::constrain(value, -1.0f, 1.0f)
	       : math::constrain(value, 0.0f, 1.0f);
}

float MotorFaultInjector::constrain_to_degraded_max(
	float value, bool reversible, float max_output_norm)
{
	return reversible
	       ? math::constrain(value, -max_output_norm, max_output_norm)
	       : math::constrain(value, 0.0f, max_output_norm);
}

bool MotorFaultInjector::same_bias_definition(
	const fault_motor_output_bias_s &snapshot) const
{
	return _bias_active && snapshot.active
	       && _bias_tracker.same_definition(
		       fault_injection::make_definition_identity(
			       snapshot.scenario_id, snapshot.sequence, snapshot.fault_id));
}

bool MotorFaultInjector::same_delay_definition(
	const fault_motor_output_delay_s &snapshot) const
{
	return _delay_active && snapshot.active
	       && _delay_tracker.same_definition(
		       fault_injection::make_definition_identity(
			       snapshot.scenario_id, snapshot.sequence, snapshot.fault_id));
}

bool MotorFaultInjector::same_max_output_definition(
	const fault_motor_max_output_degradation_s &snapshot) const
{
	return _max_output_active && snapshot.active
	       && _max_output_tracker.same_definition(
		       fault_injection::make_definition_identity(
			       snapshot.scenario_id, snapshot.sequence, snapshot.fault_id));
}

bool MotorFaultInjector::publish_bias_feedback(bool final_report)
{
	fault_motor_output_bias_applied_s feedback{};
	feedback.timestamp = hrt_absolute_time();
	feedback.timestamp_sample = _bias_tracker.last_timestamp_sample();
	const fault_injection::DefinitionIdentity &identity = _bias_tracker.identity();
	feedback.scenario_id = identity.scenario_id;
	feedback.definition_sequence = identity.definition_sequence;
	feedback.cumulative_applied_count = _bias_tracker.cumulative_count();
	feedback.fault_id = identity.fault_id;
	feedback.output_instance = OutputInstance;
	feedback.final_report = final_report;

	if (_bias_applied_pub.publish(feedback)) {
		_bias_tracker.mark_feedback_published(feedback.timestamp);
		return true;
	}

	return false;
}

bool MotorFaultInjector::publish_delay_feedback(bool final_report)
{
	fault_motor_output_delay_applied_s feedback{};
	feedback.timestamp = hrt_absolute_time();
	feedback.timestamp_sample = _delay_tracker.last_timestamp_sample();
	const fault_injection::DefinitionIdentity &identity = _delay_tracker.identity();
	feedback.scenario_id = identity.scenario_id;
	feedback.definition_sequence = identity.definition_sequence;
	feedback.cumulative_applied_count = _delay_tracker.cumulative_count();
	feedback.fault_id = identity.fault_id;
	feedback.output_instance = OutputInstance;
	feedback.final_report = final_report;

	if (_delay_applied_pub.publish(feedback)) {
		_delay_tracker.mark_feedback_published(feedback.timestamp);
		return true;
	}

	return false;
}

bool MotorFaultInjector::publish_max_output_feedback(bool final_report)
{
	fault_motor_max_output_degradation_applied_s feedback{};
	feedback.timestamp = hrt_absolute_time();
	feedback.timestamp_sample = _max_output_tracker.last_timestamp_sample();
	const fault_injection::DefinitionIdentity &identity = _max_output_tracker.identity();
	feedback.scenario_id = identity.scenario_id;
	feedback.definition_sequence = identity.definition_sequence;
	feedback.cumulative_applied_count = _max_output_tracker.cumulative_count();
	feedback.fault_id = identity.fault_id;
	feedback.output_instance = OutputInstance;
	feedback.final_report = final_report;

	if (_max_output_applied_pub.publish(feedback)) {
		_max_output_tracker.mark_feedback_published(feedback.timestamp);
		return true;
	}

	return false;
}

void MotorFaultInjector::update_bias()
{
	fault_motor_output_bias_s snapshot{};

	if (!_bias_sub.update(&snapshot)) {
		return;
	}

	const bool same_definition = same_bias_definition(snapshot);

	if (_bias_active && !same_definition) {
		publish_bias_feedback(true);
	}

	_last_bias_snapshot = snapshot;
	++_bias_update_count;
	_bias_active = false;
	_bias_motor_mask = 0;
	_bias_norm = 0.0f;

	if (!snapshot.active || (snapshot.motor_mask == 0)
	    || !std::isfinite(snapshot.bias_norm)
	    || !(std::fabs(snapshot.bias_norm) <= 1.0f)
	    || (fpclassify(snapshot.bias_norm) == FP_ZERO)) {
		return;
	}

	if (!same_definition) {
		_bias_tracker.begin_definition(
			fault_injection::make_definition_identity(
				snapshot.scenario_id, snapshot.sequence, snapshot.fault_id));
	}

	_bias_active = true;
	_bias_motor_mask = snapshot.motor_mask;
	_bias_norm = snapshot.bias_norm;
	publish_bias_feedback(false);
}

void MotorFaultInjector::update_delay()
{
	fault_motor_output_delay_s snapshot{};

	if (!_delay_sub.update(&snapshot)) {
		return;
	}

	const bool same_definition = same_delay_definition(snapshot);

	if (_delay_active && !same_definition) {
		publish_delay_feedback(true);
	}

	_last_delay_snapshot = snapshot;
	++_delay_update_count;
	_delay_active = false;
	_delay_motor_mask = 0;
	_delay_us = 0;

	if (!snapshot.active || (snapshot.motor_mask == 0)
	    || (snapshot.delay_us == 0)
	    || (snapshot.delay_us > MaximumDelayUs)) {
		return;
	}

	if (!same_definition) {
		_delay_tracker.begin_definition(
			fault_injection::make_definition_identity(
				snapshot.scenario_id, snapshot.sequence, snapshot.fault_id));
	}

	_delay_active = true;
	_delay_motor_mask = snapshot.motor_mask;
	_delay_us = snapshot.delay_us;
	publish_delay_feedback(false);
}

void MotorFaultInjector::update_max_output()
{
	fault_motor_max_output_degradation_s snapshot{};

	if (!_max_output_sub.update(&snapshot)) {
		return;
	}

	const bool same_definition = same_max_output_definition(snapshot);

	if (_max_output_active && !same_definition) {
		publish_max_output_feedback(true);
	}

	_last_max_output_snapshot = snapshot;
	++_max_output_update_count;
	_max_output_active = false;
	_max_output_motor_mask = 0;
	_max_output_norm = 1.0f;

	if (!snapshot.active || (snapshot.motor_mask == 0)
	    || !std::isfinite(snapshot.max_output_norm)
	    || !(snapshot.max_output_norm >= 0.0f)
	    || !(snapshot.max_output_norm < 1.0f)) {
		return;
	}

	if (!same_definition) {
		_max_output_tracker.begin_definition(
			fault_injection::make_definition_identity(
				snapshot.scenario_id, snapshot.sequence, snapshot.fault_id));
	}

	_max_output_active = true;
	_max_output_motor_mask = snapshot.motor_mask;
	_max_output_norm = snapshot.max_output_norm;
	publish_max_output_feedback(false);
}

void MotorFaultInjector::update()
{
	update_bias();
	update_delay();
	update_max_output();
}

void MotorFaultInjector::record_bias_application(uint64_t timestamp_sample)
{
	_bias_tracker.record_application(timestamp_sample);

	if (_bias_tracker.feedback_due(hrt_absolute_time())) {
		publish_bias_feedback(false);
	}
}

void MotorFaultInjector::record_delay_application(uint64_t timestamp_sample)
{
	_delay_tracker.record_application(timestamp_sample);

	if (_delay_tracker.feedback_due(hrt_absolute_time())) {
		publish_delay_feedback(false);
	}
}

void MotorFaultInjector::record_max_output_application(
	uint64_t timestamp_sample)
{
	_max_output_tracker.record_application(timestamp_sample);

	if (_max_output_tracker.feedback_due(hrt_absolute_time())) {
		publish_max_output_feedback(false);
	}
}

void MotorFaultInjector::store_history(const actuator_motors_s &raw)
{
	HistoryFrame &frame = _history[_history_head];
	frame.timestamp = raw.timestamp;

	for (std::size_t index = 0;
	     index < actuator_motors_s::NUM_CONTROLS;
	     ++index) {
		frame.control[index] = raw.control[index];
	}

	_history_head = (_history_head + 1) % HistoryCapacity;

	if (_history_count < HistoryCapacity) {
		++_history_count;
	}
}

bool MotorFaultInjector::find_delayed_frame(
	uint64_t target_timestamp, HistoryFrame &frame) const
{
	for (std::size_t age = 0; age < _history_count; ++age) {
		const std::size_t index =
			(_history_head + HistoryCapacity - 1 - age) % HistoryCapacity;
		const HistoryFrame &candidate = _history[index];

		if ((candidate.timestamp > 0)
		    && (candidate.timestamp <= target_timestamp)) {
			frame = candidate;
			return true;
		}
	}

	return false;
}

void MotorFaultInjector::apply(
	actuator_motors_s &motors, uint8_t motor_count)
{
	const actuator_motors_s raw = motors;
	store_history(raw);

	fault_motor_output_measurement_s measurement{};
	measurement.timestamp = hrt_absolute_time();
	measurement.timestamp_sample = raw.timestamp_sample;
	measurement.reversible_flags = raw.reversible_flags;
	measurement.motor_count =
		(motor_count < actuator_motors_s::NUM_CONTROLS)
		? motor_count
		: actuator_motors_s::NUM_CONTROLS;
	measurement.output_instance = OutputInstance;

	if (_bias_active) {
		measurement.bias_active = true;
		measurement.bias_scenario_id = _last_bias_snapshot.scenario_id;
		measurement.bias_definition_sequence = _last_bias_snapshot.sequence;
		measurement.bias_fault_id = _last_bias_snapshot.fault_id;
		measurement.bias_motor_mask = _bias_motor_mask;
		measurement.bias_norm = _bias_norm;
	}

	if (_delay_active) {
		measurement.delay_active = true;
		measurement.delay_scenario_id = _last_delay_snapshot.scenario_id;
		measurement.delay_definition_sequence = _last_delay_snapshot.sequence;
		measurement.delay_fault_id = _last_delay_snapshot.fault_id;
		measurement.delay_motor_mask = _delay_motor_mask;
		measurement.delay_us = _delay_us;
	}

	if (_max_output_active) {
		measurement.max_output_active = true;
		measurement.max_output_scenario_id =
			_last_max_output_snapshot.scenario_id;
		measurement.max_output_definition_sequence =
			_last_max_output_snapshot.sequence;
		measurement.max_output_fault_id =
			_last_max_output_snapshot.fault_id;
		measurement.max_output_motor_mask = _max_output_motor_mask;
		measurement.max_output_norm = _max_output_norm;
	} else {
		measurement.max_output_norm = 1.0f;
	}

	float delayed_control[actuator_motors_s::NUM_CONTROLS]{};

	for (std::size_t index = 0;
	     index < actuator_motors_s::NUM_CONTROLS;
	     ++index) {
		measurement.raw_control[index] = raw.control[index];
		delayed_control[index] = raw.control[index];
		measurement.delayed_control[index] = raw.control[index];

		if (std::isfinite(raw.control[index])) {
			measurement.raw_finite_mask |= (1u << index);
		}
	}

	HistoryFrame delayed_frame{};
	bool has_delayed_frame = false;

	if (_delay_active) {
		const uint64_t target_timestamp =
			(raw.timestamp > _delay_us) ? (raw.timestamp - _delay_us) : 0;
		has_delayed_frame =
			find_delayed_frame(target_timestamp, delayed_frame);

		if (has_delayed_frame) {
			measurement.delay_source_timestamp = delayed_frame.timestamp;
			++_delay_history_hit_count;

		} else {
			++_delay_history_miss_count;
		}
	}

	if (_delay_active && has_delayed_frame) {
		for (std::size_t index = 0;
		     (index < measurement.motor_count)
		     && (index < SelectableMotorCount);
		     ++index) {
			/*
			 * Current raw NaN is authoritative stop/disarmed state and may
			 * never be replaced by an older finite command.
			 */
			if (!selected(_delay_motor_mask, index)
			    || !std::isfinite(raw.control[index])) {
				continue;
			}

			const float candidate = delayed_frame.control[index];

			if (values_differ(candidate, delayed_control[index])) {
				delayed_control[index] = candidate;
				measurement.delayed_control[index] = candidate;
				measurement.delay_applied_mask |= (1u << index);
			}
		}
	}

	float working[actuator_motors_s::NUM_CONTROLS]{};

	for (std::size_t index = 0;
	     index < actuator_motors_s::NUM_CONTROLS;
	     ++index) {
		working[index] = delayed_control[index];
	}

	if (_bias_active) {
		for (std::size_t index = 0;
		     (index < measurement.motor_count)
		     && (index < SelectableMotorCount);
		     ++index) {
			if (!selected(_bias_motor_mask, index)
			    || !std::isfinite(working[index])) {
				continue;
			}

			const bool reversible =
				(raw.reversible_flags & (1u << index)) != 0;
			const float candidate = constrain_motor_command(
				working[index] + _bias_norm, reversible);

			if (values_differ(candidate, working[index])) {
				working[index] = candidate;
				measurement.bias_applied_mask |= (1u << index);
			}
		}
	}

	if (_max_output_active) {
		for (std::size_t index = 0;
		     (index < measurement.motor_count)
		     && (index < SelectableMotorCount);
		     ++index) {
			if (!selected(_max_output_motor_mask, index)
			    || !std::isfinite(working[index])) {
				continue;
			}

			const bool reversible =
				(raw.reversible_flags & (1u << index)) != 0;
			const float candidate = constrain_to_degraded_max(
				working[index], reversible, _max_output_norm);

			if (values_differ(candidate, working[index])) {
				working[index] = candidate;
				measurement.max_output_applied_mask |= (1u << index);
			}
		}
	}

	for (std::size_t index = 0;
	     index < actuator_motors_s::NUM_CONTROLS;
	     ++index) {
		motors.control[index] = working[index];
		measurement.faulted_control[index] = working[index];
	}

	if (measurement.delay_applied_mask != 0) {
		record_delay_application(raw.timestamp_sample);
	}

	if (measurement.bias_applied_mask != 0) {
		record_bias_application(raw.timestamp_sample);
	}

	if (measurement.max_output_applied_mask != 0) {
		record_max_output_application(raw.timestamp_sample);
	}

	if (_measurement_pub.publish(measurement)) {
		++_measurement_publish_count;
	}
}

void MotorFaultInjector::print_status() const
{
	PX4_INFO(
		"fault motor history: samples=%u/%u hits=%u misses=%u measurements=%u",
		(unsigned)_history_count, (unsigned)HistoryCapacity,
		(unsigned)_delay_history_hit_count,
		(unsigned)_delay_history_miss_count,
		(unsigned)_measurement_publish_count);
	PX4_INFO(
		"fault motor bias: updates=%u active=%u mask=0x%02x value=%.6f cumulative=%u",
		(unsigned)_bias_update_count, (unsigned)_bias_active,
		(unsigned)_bias_motor_mask, (double)_bias_norm,
		(unsigned)_bias_tracker.cumulative_count());
	PX4_INFO(
		"fault motor delay: updates=%u active=%u mask=0x%02x delay_us=%u cumulative=%u",
		(unsigned)_delay_update_count, (unsigned)_delay_active,
		(unsigned)_delay_motor_mask, (unsigned)_delay_us,
		(unsigned)_delay_tracker.cumulative_count());
	PX4_INFO(
		"fault motor max output: updates=%u active=%u mask=0x%02x cap=%.6f cumulative=%u",
		(unsigned)_max_output_update_count, (unsigned)_max_output_active,
		(unsigned)_max_output_motor_mask, (double)_max_output_norm,
		(unsigned)_max_output_tracker.cumulative_count());
}
