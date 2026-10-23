/****************************************************************************
 *
 *   FAULT GNSS position-bias runtime snapshot consumer.
 *
 ****************************************************************************/

#include "GnssPositionBiasInjector.hpp"
#include "GnssPositionFaultUtils.hpp"

#include <drivers/drv_hrt.h>
#include <lib/fault_injection/VectorEffectValidator.hpp>

void GnssPositionBiasInjector::reset_effect_to_neutral()
{
	_bias_ned_m[0] = 0.0f;
	_bias_ned_m[1] = 0.0f;
	_bias_ned_m[2] = 0.0f;
	_axis_mask = 0;
	_active = false;
}

void GnssPositionBiasInjector::set_estimator_instance(int estimator_instance)
{
	if ((estimator_instance >= 0) && (estimator_instance < UINT8_MAX)) {
		_estimator_instance = static_cast<uint8_t>(estimator_instance);
	} else {
		_estimator_instance = UINT8_MAX;
	}
}

bool GnssPositionBiasInjector::same_active_definition(
	const fault_gnss_position_bias_s &snapshot) const
{
	const fault_injection::DefinitionIdentity identity =
		fault_injection::make_definition_identity(
			snapshot.scenario_id,
			snapshot.sequence,
			snapshot.fault_id);

	return _active
	       && snapshot.active
	       && _application_tracker.same_definition(identity);
}

bool GnssPositionBiasInjector::publish_feedback(bool final_report)
{
	fault_gnss_position_bias_applied_s feedback{};
	feedback.timestamp = hrt_absolute_time();
	feedback.timestamp_sample = _application_tracker.last_timestamp_sample();

	const fault_injection::DefinitionIdentity &identity =
		_application_tracker.identity();

	feedback.scenario_id = identity.scenario_id;
	feedback.definition_sequence = identity.definition_sequence;
	feedback.cumulative_applied_count = _application_tracker.cumulative_count();
	feedback.fault_id = identity.fault_id;
	feedback.estimator_instance = _estimator_instance;
	feedback.final_report = final_report;

	if (_applied_feedback_pub.publish(feedback)) {
		_application_tracker.mark_feedback_published(feedback.timestamp);
		++_feedback_published_count;
		return true;
	}

	++_feedback_publish_failure_count;
	return false;
}

bool GnssPositionBiasInjector::update(int estimator_instance)
{
	set_estimator_instance(estimator_instance);

	fault_gnss_position_bias_s snapshot{};

	if (!_snapshot_sub.update(&snapshot)) {
		return false;
	}

	const bool same_definition = same_active_definition(snapshot);

	if (_active && !same_definition) {
		publish_feedback(true);
	}

	_last_snapshot = snapshot;
	_has_snapshot = true;
	++_update_count;
	reset_effect_to_neutral();

	if (!snapshot.active) {
		++_neutral_snapshot_count;
		return true;
	}

	const fault_injection::VectorEffectValidation validation =
		fault_injection::validate_vector_effect<AxisCount>(
			snapshot.axis_mask,
			snapshot.bias_ned_m,
			fault_injection::SelectedValueRule::Finite,
			fault_injection::UnselectedValueRule::RequireZero);

	if (!validation.accepted()) {
		++_neutral_snapshot_count;
		return true;
	}

	for (std::size_t axis = 0; axis < AxisCount; ++axis) {
		const uint8_t axis_bit = static_cast<uint8_t>(1U << axis);

		if ((snapshot.axis_mask & axis_bit) != 0) {
			_bias_ned_m[axis] = snapshot.bias_ned_m[axis];
		}
	}

	if (!same_definition) {
		_application_tracker.begin_definition(
			fault_injection::make_definition_identity(
				snapshot.scenario_id,
				snapshot.sequence,
				snapshot.fault_id));
	}

	_axis_mask = snapshot.axis_mask;
	_active = true;
	++_active_snapshot_count;
	publish_feedback(false);
	return true;
}

bool GnssPositionBiasInjector::record_application(
	uint64_t timestamp_sample,
	int estimator_instance)
{
	if (!_active) {
		return false;
	}

	set_estimator_instance(estimator_instance);
	_application_tracker.record_application(timestamp_sample);

	const uint64_t now_us = hrt_absolute_time();

	if (_application_tracker.feedback_due(now_us)) {
		publish_feedback(false);
	}

	return true;
}

bool GnssPositionBiasInjector::apply(
	estimator::gnssSample &gnss_sample,
	uint64_t timestamp_sample,
	int estimator_instance)
{
	if (!_active) {
		return false;
	}

	const matrix::Vector3f offset_ned_m{
		_bias_ned_m[0],
		_bias_ned_m[1],
		_bias_ned_m[2]
	};

	estimator::gnssSample faulted_sample{gnss_sample};

	if (!fault_gnss::apply_ned_position_offset(
		    faulted_sample,
		    offset_ned_m)) {
		return false;
	}

	gnss_sample = faulted_sample;
	return record_application(timestamp_sample, estimator_instance);
}
