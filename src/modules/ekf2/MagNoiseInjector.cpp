/****************************************************************************
 *
 *   FAULT magnetometer-noise runtime snapshot consumer.
 *
 ****************************************************************************/

#include "MagNoiseInjector.hpp"

#include <cmath>
#include <limits>

#include <drivers/drv_hrt.h>
#include <lib/fault_injection/VectorEffectValidator.hpp>

void MagNoiseInjector::reset_effect_to_neutral()
{
	_sigma_ga[0] = 0.0f;
	_sigma_ga[1] = 0.0f;
	_sigma_ga[2] = 0.0f;

	_random_seed = 0;
	_axis_mask = 0;
	_active = false;
}

void MagNoiseInjector::reset_generator_diagnostics()
{
	_first_noise_ga =
		matrix::Vector3f{0.0f, 0.0f, 0.0f};
	_last_noise_ga =
		matrix::Vector3f{0.0f, 0.0f, 0.0f};

	_generated_sample_count = 0;
}

void MagNoiseInjector::set_estimator_instance(int estimator_instance)
{
	if ((estimator_instance >= 0) && (estimator_instance < UINT8_MAX)) {
		_estimator_instance = static_cast<uint8_t>(estimator_instance);
	} else {
		_estimator_instance = UINT8_MAX;
	}
}

bool MagNoiseInjector::same_active_definition(
	const fault_mag_noise_s &snapshot) const
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

bool MagNoiseInjector::publish_feedback(bool final_report)
{
	fault_mag_noise_applied_s feedback{};
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

bool MagNoiseInjector::update(int estimator_instance)
{
	set_estimator_instance(estimator_instance);

	fault_mag_noise_s snapshot{};

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
			snapshot.sigma_ga,
			fault_injection::SelectedValueRule::StrictlyPositive,
			fault_injection::UnselectedValueRule::RequireZero);

	if (!validation.accepted()) {
		_gaussian_generator.reset();
		++_neutral_snapshot_count;
		return true;
	}

	for (std::size_t axis = 0; axis < AxisCount; ++axis) {
		const uint8_t axis_bit =
			static_cast<uint8_t>(1U << axis);

		if ((snapshot.axis_mask & axis_bit) != 0) {
			_sigma_ga[axis] =
				snapshot.sigma_ga[axis];
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
		random_identity.random_seed =
			snapshot.random_seed;

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

bool MagNoiseInjector::record_application(
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

bool MagNoiseInjector::apply(
	matrix::Vector3f &magnetic_field_ga,
	uint64_t timestamp_sample,
	int estimator_instance)
{
	if (!_active
	    || !_gaussian_generator.initialized()
	    || !magnetic_field_ga.isAllFinite()) {
		return false;
	}

	matrix::Vector3f noise_ga{
		0.0f,
		0.0f,
		0.0f
	};

	for (std::size_t axis = 0; axis < AxisCount; ++axis) {
		const uint8_t axis_bit =
			static_cast<uint8_t>(1U << axis);

		if ((_axis_mask & axis_bit) == 0) {
			continue;
		}

		const double standard_normal =
			_gaussian_generator.sample(axis);

		const double scaled_noise =
			static_cast<double>(_sigma_ga[axis])
			* standard_normal;

		if (!std::isfinite(standard_normal)
		    || !std::isfinite(scaled_noise)) {
			return false;
		}

		noise_ga(axis) =
			static_cast<float>(scaled_noise);
	}

	if (_generated_sample_count
	    < std::numeric_limits<uint32_t>::max()) {
		++_generated_sample_count;
	}

	if (!noise_ga.isAllFinite() || !noise_ga.longerThan(0.0f)) {
		return false;
	}

	const matrix::Vector3f faulted_field =
		magnetic_field_ga + noise_ga;

	if (!faulted_field.isAllFinite()) {
		return false;
	}

	const bool first_application =
		(_application_tracker.cumulative_count() == 0);

	magnetic_field_ga = faulted_field;

	if (!record_application(
		    timestamp_sample,
		    estimator_instance)) {
		return false;
	}

	if (first_application) {
		_first_noise_ga = noise_ga;
	}

	_last_noise_ga = noise_ga;
	return true;
}
