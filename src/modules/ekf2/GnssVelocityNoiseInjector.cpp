/****************************************************************************
 *
 *   FAULT deterministic GNSS velocity-noise runtime snapshot consumer.
 *
 ****************************************************************************/

#include "GnssVelocityNoiseInjector.hpp"

#include <stdint.h>

#include <cmath>

#include <drivers/drv_hrt.h>
#include <lib/fault_injection/VectorEffectValidator.hpp>

void GnssVelocityNoiseInjector::reset_effect_to_neutral()
{
	_sigma_ned_m_s[0] = 0.0f;
	_sigma_ned_m_s[1] = 0.0f;
	_sigma_ned_m_s[2] = 0.0f;
	_random_seed = 0;
	_axis_mask = 0;
	_active = false;
}

void GnssVelocityNoiseInjector::reset_generator_diagnostics()
{
	_first_noise_ned_m_s = matrix::Vector3f{0.0f, 0.0f, 0.0f};
	_last_noise_ned_m_s = matrix::Vector3f{0.0f, 0.0f, 0.0f};
	_generated_sample_count = 0;
}

void GnssVelocityNoiseInjector::set_estimator_instance(int estimator_instance)
{
	if ((estimator_instance >= 0) && (estimator_instance < UINT8_MAX)) {
		_estimator_instance = static_cast<uint8_t>(estimator_instance);
	} else {
		_estimator_instance = UINT8_MAX;
	}
}

bool GnssVelocityNoiseInjector::same_active_definition(
	const fault_gnss_velocity_noise_s &snapshot) const
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

bool GnssVelocityNoiseInjector::publish_feedback(bool final_report)
{
	fault_gnss_velocity_noise_applied_s feedback{};
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

bool GnssVelocityNoiseInjector::update(int estimator_instance)
{
	set_estimator_instance(estimator_instance);

	fault_gnss_velocity_noise_s snapshot{};

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
		_gaussian_generator.reset();
		++_neutral_snapshot_count;
		return true;
	}

	const fault_injection::VectorEffectValidation validation =
		fault_injection::validate_vector_effect<AxisCount>(
			snapshot.axis_mask,
			snapshot.sigma_ned_m_s,
			fault_injection::SelectedValueRule::StrictlyPositive,
			fault_injection::UnselectedValueRule::RequireZero);

	if (!validation.accepted()) {
		_gaussian_generator.reset();
		++_neutral_snapshot_count;
		return true;
	}

	for (std::size_t axis = 0; axis < AxisCount; ++axis) {
		const uint8_t axis_bit = static_cast<uint8_t>(1U << axis);

		if ((snapshot.axis_mask & axis_bit) != 0) {
			_sigma_ned_m_s[axis] = snapshot.sigma_ned_m_s[axis];
		}
	}

	if (!same_definition) {
		const fault_injection::DefinitionIdentity identity =
			fault_injection::make_definition_identity(
				snapshot.scenario_id,
				snapshot.sequence,
				snapshot.fault_id);

		_application_tracker.begin_definition(identity);

		fault_injection::RandomDefinitionIdentity random_identity{};
		random_identity.definition = identity;
		random_identity.random_seed = snapshot.random_seed;
		random_identity.stream_domain = StreamDomain;
		_gaussian_generator.initialize(random_identity);
		reset_generator_diagnostics();
	}

	_random_seed = snapshot.random_seed;
	_axis_mask = snapshot.axis_mask;
	_active = true;
	++_active_snapshot_count;
	publish_feedback(false);
	return true;
}

bool GnssVelocityNoiseInjector::record_application(
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

bool GnssVelocityNoiseInjector::apply(
	estimator::gnssSample &gnss_sample,
	uint64_t timestamp_sample,
	int estimator_instance)
{
	if (!_active
	    || !_gaussian_generator.initialized()
	    || !gnss_sample.vel.isAllFinite()) {
		return false;
	}

	matrix::Vector3f noise_ned_m_s{0.0f, 0.0f, 0.0f};

	for (std::size_t axis = 0; axis < AxisCount; ++axis) {
		const uint8_t axis_bit = static_cast<uint8_t>(1U << axis);

		if ((_axis_mask & axis_bit) == 0) {
			continue;
		}

		const double standard_normal = _gaussian_generator.sample(axis);
		const double scaled_noise =
			static_cast<double>(_sigma_ned_m_s[axis]) * standard_normal;

		if (!std::isfinite(standard_normal) || !std::isfinite(scaled_noise)) {
			return false;
		}

		noise_ned_m_s(axis) = static_cast<float>(scaled_noise);
	}

	if (!noise_ned_m_s.isAllFinite()) {
		return false;
	}

	const matrix::Vector3f faulted_velocity =
		gnss_sample.vel + noise_ned_m_s;

	if (!faulted_velocity.isAllFinite()) {
		return false;
	}

	const bool first_application =
		(_application_tracker.cumulative_count() == 0);

	gnss_sample.vel = faulted_velocity;

	if (!record_application(timestamp_sample, estimator_instance)) {
		return false;
	}

	if (_generated_sample_count < UINT32_MAX) {
		++_generated_sample_count;
	}

	if (first_application) {
		_first_noise_ned_m_s = noise_ned_m_s;
	}

	_last_noise_ned_m_s = noise_ned_m_s;
	return true;
}
