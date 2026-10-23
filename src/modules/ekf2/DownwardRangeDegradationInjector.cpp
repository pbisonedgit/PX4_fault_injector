/****************************************************************************
 *
 *   FAULT downward-range effective-maximum-distance degradation consumer.
 *
 ****************************************************************************/
#include "DownwardRangeDegradationInjector.hpp"

#include <algorithm>
#include <cmath>

#include <drivers/drv_hrt.h>

void DownwardRangeDegradationInjector::reset_effect_to_neutral()
{
	_configured_max_distance_m = 0.0f;
	_active = false;
}

void DownwardRangeDegradationInjector::set_estimator_instance(int estimator_instance)
{
	_estimator_instance = ((estimator_instance >= 0) && (estimator_instance < UINT8_MAX))
		? static_cast<uint8_t>(estimator_instance) : UINT8_MAX;
}

bool DownwardRangeDegradationInjector::same_active_definition(
	const fault_downward_range_degradation_s &snapshot) const
{
	return _active && snapshot.active
	       && _application_tracker.same_definition(
		       fault_injection::make_definition_identity(
			       snapshot.scenario_id, snapshot.sequence, snapshot.fault_id));
}

bool DownwardRangeDegradationInjector::publish_feedback(bool final_report)
{
	fault_downward_range_degradation_applied_s feedback{};
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
		return true;
	}

	return false;
}

bool DownwardRangeDegradationInjector::update(int estimator_instance)
{
	set_estimator_instance(estimator_instance);
	fault_downward_range_degradation_s snapshot{};

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

	if (!snapshot.active || !std::isfinite(snapshot.effective_max_distance_m)
	    || !(snapshot.effective_max_distance_m > 0.0f)) {
		++_neutral_snapshot_count;
		return true;
	}

	if (!same_definition) {
		_application_tracker.begin_definition(
			fault_injection::make_definition_identity(
				snapshot.scenario_id, snapshot.sequence, snapshot.fault_id));
	}

	_configured_max_distance_m = snapshot.effective_max_distance_m;
	_active = true;
	++_active_snapshot_count;
	publish_feedback(false);
	return true;
}

float DownwardRangeDegradationInjector::effective_max_distance_m(
	float nominal_max_distance_m) const
{
	if (!_active || !std::isfinite(nominal_max_distance_m)
	    || !(nominal_max_distance_m > 0.0f)) {
		return nominal_max_distance_m;
	}

	return std::min(_configured_max_distance_m, nominal_max_distance_m);
}

bool DownwardRangeDegradationInjector::record_application(
	uint64_t timestamp_sample, int estimator_instance)
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

bool DownwardRangeDegradationInjector::gate(
	float raw_distance_m, float nominal_max_distance_m,
	uint64_t timestamp_sample, int estimator_instance,
	float &effective_max_distance_m_out)
{
	effective_max_distance_m_out = effective_max_distance_m(nominal_max_distance_m);

	if (!_active || !std::isfinite(raw_distance_m)
	    || !std::isfinite(effective_max_distance_m_out)
	    || !(raw_distance_m > effective_max_distance_m_out)) {
		return false;
	}

	return record_application(timestamp_sample, estimator_instance);
}
