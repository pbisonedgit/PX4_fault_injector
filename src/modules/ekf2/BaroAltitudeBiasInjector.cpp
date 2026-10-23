/****************************************************************************
 *
 *   FAULT barometric-altitude bias runtime snapshot consumer.
 *
 ****************************************************************************/
#include "BaroAltitudeBiasInjector.hpp"

#include <cmath>

#include <drivers/drv_hrt.h>

void BaroAltitudeBiasInjector::reset_effect_to_neutral()
{
	_bias_m = 0.0f;
	_active = false;
}

void BaroAltitudeBiasInjector::set_estimator_instance(int estimator_instance)
{
	_estimator_instance = ((estimator_instance >= 0) && (estimator_instance < UINT8_MAX))
		? static_cast<uint8_t>(estimator_instance) : UINT8_MAX;
}

bool BaroAltitudeBiasInjector::same_active_definition(
	const fault_baro_altitude_bias_s &snapshot) const
{
	return _active && snapshot.active
	       && _application_tracker.same_definition(
		       fault_injection::make_definition_identity(
			       snapshot.scenario_id, snapshot.sequence, snapshot.fault_id));
}

bool BaroAltitudeBiasInjector::publish_feedback(bool final_report)
{
	fault_baro_altitude_bias_applied_s feedback{};
	feedback.timestamp = hrt_absolute_time();
	feedback.timestamp_sample = _application_tracker.last_timestamp_sample();
	const fault_injection::DefinitionIdentity &identity = _application_tracker.identity();
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

bool BaroAltitudeBiasInjector::update(int estimator_instance)
{
	set_estimator_instance(estimator_instance);
	fault_baro_altitude_bias_s snapshot{};
	if (!_snapshot_sub.update(&snapshot)) { return false; }
	const bool same_definition = same_active_definition(snapshot);
	if (_active && !same_definition) { publish_feedback(true); }
	_last_snapshot = snapshot;
	_has_snapshot = true;
	++_update_count;
	reset_effect_to_neutral();
	if (!snapshot.active || !std::isfinite(snapshot.bias_m)) {
		++_neutral_snapshot_count;
		return true;
	}
	if (!same_definition) {
		_application_tracker.begin_definition(
			fault_injection::make_definition_identity(
				snapshot.scenario_id, snapshot.sequence, snapshot.fault_id));
	}
	_bias_m = snapshot.bias_m;
	_active = true;
	++_active_snapshot_count;
	publish_feedback(false);
	return true;
}

bool BaroAltitudeBiasInjector::record_application(
	uint64_t timestamp_sample, int estimator_instance)
{
	if (!_active) { return false; }
	set_estimator_instance(estimator_instance);
	_application_tracker.record_application(timestamp_sample);
	const uint64_t now_us = hrt_absolute_time();
	if (_application_tracker.feedback_due(now_us)) { publish_feedback(false); }
	return true;
}

bool BaroAltitudeBiasInjector::apply(
	float &altitude_m, uint64_t timestamp_sample,
	int estimator_instance, float &applied_bias_m)
{
	applied_bias_m = 0.0f;
	if (!_active || !std::isfinite(altitude_m)) { return false; }
	const float faulted_altitude_m = altitude_m + _bias_m;
	if (!std::isfinite(faulted_altitude_m)) { return false; }
	altitude_m = faulted_altitude_m;
	applied_bias_m = _bias_m;
	return record_application(timestamp_sample, estimator_instance);
}
