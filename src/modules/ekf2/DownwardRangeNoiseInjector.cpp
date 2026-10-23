/****************************************************************************
 *
 *   FAULT deterministic downward-range noise runtime snapshot consumer.
 *
 ****************************************************************************/
#include "DownwardRangeNoiseInjector.hpp"

#include <cmath>
#include <limits>

#include <drivers/drv_hrt.h>

void DownwardRangeNoiseInjector::reset_effect_to_neutral()
{
	_sigma_m = 0.0f;
	_random_seed = 0;
	_active = false;
}

void DownwardRangeNoiseInjector::reset_generator_diagnostics()
{
	_first_noise_m = 0.0f;
	_last_noise_m = 0.0f;
	_generated_sample_count = 0;
}

void DownwardRangeNoiseInjector::set_estimator_instance(int estimator_instance)
{
	_estimator_instance = ((estimator_instance >= 0) && (estimator_instance < UINT8_MAX))
		? static_cast<uint8_t>(estimator_instance) : UINT8_MAX;
}

bool DownwardRangeNoiseInjector::same_active_definition(
	const fault_downward_range_noise_s &snapshot) const
{
	return _active && snapshot.active
	       && _application_tracker.same_definition(
		       fault_injection::make_definition_identity(
			       snapshot.scenario_id, snapshot.sequence, snapshot.fault_id));
}

bool DownwardRangeNoiseInjector::publish_feedback(bool final_report)
{
	fault_downward_range_noise_applied_s feedback{};
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

bool DownwardRangeNoiseInjector::update(int estimator_instance)
{
	set_estimator_instance(estimator_instance);
	fault_downward_range_noise_s snapshot{};
	if (!_snapshot_sub.update(&snapshot)) { return false; }
	const bool same_definition = same_active_definition(snapshot);
	if (_active && !same_definition) { publish_feedback(true); }
	_last_snapshot = snapshot;
	_has_snapshot = true;
	++_update_count;
	reset_effect_to_neutral();
	if (!snapshot.active || !std::isfinite(snapshot.sigma_m) || !(snapshot.sigma_m > 0.0f)) {
		_gaussian_generator.reset();
		++_neutral_snapshot_count;
		return true;
	}
	if (!same_definition) {
		const fault_injection::DefinitionIdentity identity =
			fault_injection::make_definition_identity(
				snapshot.scenario_id, snapshot.sequence, snapshot.fault_id);
		_application_tracker.begin_definition(identity);
		fault_injection::RandomDefinitionIdentity random_identity{};
		random_identity.definition = identity;
		random_identity.random_seed = snapshot.random_seed;
		random_identity.stream_domain = StreamDomain;
		_gaussian_generator.initialize(random_identity);
		reset_generator_diagnostics();
	}
	_sigma_m = snapshot.sigma_m;
	_random_seed = snapshot.random_seed;
	_active = true;
	++_active_snapshot_count;
	publish_feedback(false);
	return true;
}

bool DownwardRangeNoiseInjector::record_application(
	uint64_t timestamp_sample, int estimator_instance)
{
	if (!_active) { return false; }
	set_estimator_instance(estimator_instance);
	_application_tracker.record_application(timestamp_sample);
	const uint64_t now_us = hrt_absolute_time();
	if (_application_tracker.feedback_due(now_us)) { publish_feedback(false); }
	return true;
}

bool DownwardRangeNoiseInjector::apply(
	float &range_m, uint64_t timestamp_sample,
	int estimator_instance, float &applied_noise_m)
{
	applied_noise_m = 0.0f;
	if (!_active || !_gaussian_generator.initialized()
	    || !std::isfinite(range_m) || !(range_m > 0.0f)) { return false; }
	const double standard_normal = _gaussian_generator.sample(0);
	const double scaled_noise = static_cast<double>(_sigma_m) * standard_normal;
	if (!std::isfinite(standard_normal) || !std::isfinite(scaled_noise)) { return false; }
	const float noise_m = static_cast<float>(scaled_noise);
	const float faulted_range_m = range_m + noise_m;
	if (!std::isfinite(faulted_range_m) || !(faulted_range_m > 0.0f)) { return false; }
	const bool first_application = (_application_tracker.cumulative_count() == 0);
	range_m = faulted_range_m;
	if (!record_application(timestamp_sample, estimator_instance)) { return false; }
	if (_generated_sample_count < std::numeric_limits<uint32_t>::max()) { ++_generated_sample_count; }
	if (first_application) { _first_noise_m = noise_m; }
	_last_noise_m = noise_m;
	applied_noise_m = noise_m;
	return true;
}
