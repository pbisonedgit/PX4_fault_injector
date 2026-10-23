/****************************************************************************
 *
 *   FAULT magnetometer bias runtime snapshot consumer.
 *
 ****************************************************************************/

#include "MagBiasInjector.hpp"

#include <drivers/drv_hrt.h>
#include <lib/fault_injection/VectorEffectValidator.hpp>

void MagBiasInjector::reset_effect_to_neutral()
{
	_bias_ga[0] = 0.0f;
	_bias_ga[1] = 0.0f;
	_bias_ga[2] = 0.0f;

	_axis_mask = 0;
	_active = false;
}

void MagBiasInjector::set_estimator_instance(
	int estimator_instance)
{
	if ((estimator_instance >= 0)
	    && (estimator_instance < UINT8_MAX)) {
		_estimator_instance =
			static_cast<uint8_t>(estimator_instance);

	} else {
		_estimator_instance = UINT8_MAX;
	}
}

bool MagBiasInjector::same_active_definition(
	const fault_mag_bias_s &snapshot) const
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

bool MagBiasInjector::publish_feedback(bool final_report)
{
	fault_mag_bias_applied_s feedback{};
	feedback.timestamp = hrt_absolute_time();
	feedback.timestamp_sample =
		_application_tracker.last_timestamp_sample();

	const fault_injection::DefinitionIdentity &identity =
		_application_tracker.identity();

	feedback.scenario_id = identity.scenario_id;
	feedback.definition_sequence =
		identity.definition_sequence;
	feedback.cumulative_applied_count =
		_application_tracker.cumulative_count();

	feedback.fault_id = identity.fault_id;
	feedback.estimator_instance = _estimator_instance;
	feedback.final_report = final_report;

	if (_applied_feedback_pub.publish(feedback)) {
		_application_tracker.mark_feedback_published(
			feedback.timestamp);
		++_feedback_published_count;
		return true;
	}

	++_feedback_publish_failure_count;
	return false;
}

bool MagBiasInjector::update(int estimator_instance)
{
	set_estimator_instance(estimator_instance);

	fault_mag_bias_s snapshot{};

	if (!_snapshot_sub.update(&snapshot)) {
		return false;
	}

	const bool same_definition =
		same_active_definition(snapshot);

	/*
	 * Flush the cumulative value for the old active definition before a
	 * neutral snapshot or a different definition replaces the local cache.
	 */
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
		fault_injection::validate_vector_effect<3>(
			snapshot.axis_mask,
			snapshot.bias_ga,
			fault_injection::SelectedValueRule::Finite,
			fault_injection::UnselectedValueRule::Ignore);

	if (!validation.accepted()) {
		++_neutral_snapshot_count;
		return true;
	}

	for (std::size_t axis = 0; axis < 3; ++axis) {
		const uint8_t axis_bit =
			static_cast<uint8_t>(1U << axis);

		if ((snapshot.axis_mask & axis_bit) != 0) {
			_bias_ga[axis] =
				snapshot.bias_ga[axis];
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

	/*
	 * Advertise a zero-count baseline as soon as the definition becomes
	 * active. This makes the feedback path independently testable before the
	 * next stage starts modifying magnetometer samples.
	 */
	publish_feedback(false);
	return true;
}

bool MagBiasInjector::record_application(
	uint64_t timestamp_sample,
	int estimator_instance)
{
	if (!_active) {
		return false;
	}

	set_estimator_instance(estimator_instance);

	_application_tracker.record_application(
		timestamp_sample);

	const uint64_t now_us = hrt_absolute_time();

	if (_application_tracker.feedback_due(now_us)) {
		publish_feedback(false);
	}

	return true;
}

bool MagBiasInjector::apply(
	matrix::Vector3f &magnetic_field_ga,
	uint64_t timestamp_sample,
	int estimator_instance)
{
	if (!_active || !magnetic_field_ga.isAllFinite()) {
		return false;
	}

	const matrix::Vector3f correction{
		_bias_ga[0], _bias_ga[1], _bias_ga[2]
	};

	if (!correction.isAllFinite() || !correction.longerThan(0.0f)) {
		return false;
	}

	const matrix::Vector3f corrected_field = magnetic_field_ga + correction;

	if (!corrected_field.isAllFinite()) {
		return false;
	}

	magnetic_field_ga = corrected_field;
	return record_application(timestamp_sample, estimator_instance);
}
