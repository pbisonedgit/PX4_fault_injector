#include "FaultManager.hpp"

#include <cstddef>
#include <inttypes.h>

namespace
{

fault_injection::FaultSpec make_fault_spec(
	const fault_command_s &command)
{
	fault_injection::FaultSpec spec{};

	spec.scenario_id = command.scenario_id;
	spec.sequence = command.sequence;
	spec.start_delay_ms = command.start_delay_ms;
	spec.duration_ms = command.duration_ms;
	spec.random_seed = command.random_seed;
	spec.delay_us = command.delay_us;
	spec.instance_mask = command.instance_mask;

	spec.vector[0] = command.vector[0];
	spec.vector[1] = command.vector[1];
	spec.vector[2] = command.vector[2];
	spec.scalar = command.scalar;

	spec.fault_id = command.fault_id;

	spec.source_system = command.source_system;
	spec.source_component = command.source_component;
	spec.target_system = command.target_system;
	spec.target_component = command.target_component;
	spec.protocol_version = command.protocol_version;
	spec.target = command.target;
	spec.mode = command.mode;
	spec.axis_mask = command.axis_mask;

	return spec;
}

uint8_t configure_result_to_ack_reason(
	fault_injection::FaultConfigureResult result)
{
	switch (result) {
	case fault_injection::FaultConfigureResult::Accepted:
		return fault_ack_s::REASON_NONE;

	case fault_injection::FaultConfigureResult::InvalidTarget:
		return fault_ack_s::REASON_INVALID_TARGET;

	case fault_injection::FaultConfigureResult::InvalidMode:
		return fault_ack_s::REASON_INVALID_MODE;

	case fault_injection::FaultConfigureResult::InvalidInstanceMask:
		return fault_ack_s::REASON_INVALID_INSTANCE_MASK;

	case fault_injection::FaultConfigureResult::InvalidAxisMask:
		return fault_ack_s::REASON_INVALID_AXIS_MASK;

	case fault_injection::FaultConfigureResult::InvalidDuration:
		return fault_ack_s::REASON_INVALID_DURATION;

	case fault_injection::FaultConfigureResult::InvalidParameter:
		return fault_ack_s::REASON_INVALID_PARAMETER;

	case fault_injection::FaultConfigureResult::InternalError:
	default:
		return fault_ack_s::REASON_NONE;
	}
}

} // namespace

FaultManager::FaultManager() :
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::lp_default)
{
}

bool FaultManager::init()
{
	/*
	 * Publish an explicit neutral latest-value snapshot before the periodic
	 * work starts. A later EKF-side consumer therefore never depends on
	 * whether it started before or after this module.
	 *
	 * This stage still does not transform any flight data.
	 */
	publish_imu_accel_bias_effect(
		_fault_registry.imu_accel_bias(),
		0);

	publish_imu_accel_noise_effect(
		_fault_registry.imu_accel_noise(),
		0);

	publish_imu_gyro_bias_effect(
		_fault_registry.imu_gyro_bias(),
		0);

	publish_mag_bias_effect(
		_fault_registry.mag_bias(),
		0);
	publish_mag_noise_effect(
		_fault_registry.mag_noise(),
		0);
	publish_gnss_position_bias_effect(
		_fault_registry.gnss_position_bias(),
		0);
	publish_gnss_position_noise_effect(
		_fault_registry.gnss_position_noise(),
		0);
	publish_gnss_velocity_bias_effect(
		_fault_registry.gnss_velocity_bias(),
		0);
	publish_gnss_velocity_noise_effect(
		_fault_registry.gnss_velocity_noise(),
		0);
	publish_downward_range_bias_effect(_fault_registry.downward_range_bias(), 0);
	publish_downward_range_noise_effect(_fault_registry.downward_range_noise(), 0);
	publish_downward_range_degradation_effect(_fault_registry.downward_range_degradation(), 0);
	publish_baro_altitude_bias_effect(_fault_registry.baro_altitude_bias(), 0);
	publish_baro_altitude_noise_effect(_fault_registry.baro_altitude_noise(), 0);
	publish_motor_output_bias_effect(_fault_registry.motor_output_bias(), 0);
	publish_motor_output_delay_effect(_fault_registry.motor_output_delay(), 0);
	publish_motor_max_output_degradation_effect(
		_fault_registry.motor_max_output_degradation(), 0);

	ScheduleOnInterval(100_ms);
	return true;
}

void FaultManager::Run()
{
	if (should_exit()) {
		ScheduleClear();
		exit_and_cleanup();
		return;
	}

	++_run_count;

	const uint64_t now_us = hrt_absolute_time();

	process_imu_accel_bias_applied();
	process_imu_accel_noise_applied();
	process_imu_gyro_bias_applied();
	process_mag_bias_applied();
	process_mag_noise_applied();
	process_gnss_position_bias_applied();
	process_gnss_position_noise_applied();
	process_gnss_velocity_bias_applied();
	process_gnss_velocity_noise_applied();
	process_downward_range_bias_applied();
	process_downward_range_noise_applied();
	process_downward_range_degradation_applied();
	process_baro_altitude_bias_applied();
	process_baro_altitude_noise_applied();
	process_motor_output_bias_applied();
	process_motor_output_delay_applied();
	process_motor_max_output_degradation_applied();
	process_imu_gyro_noise_applied();

	fault_injection::FaultInstance *changed_faults[
		fault_injection::FaultRegistry::FaultCount
	]{};

	const std::size_t changed_count =
		_fault_registry.update(
			now_us,
			changed_faults,
			fault_injection::FaultRegistry::FaultCount);

	for (std::size_t index = 0;
	     (index < changed_count)
	     && (index < fault_injection::FaultRegistry::FaultCount);
	     ++index) {
		if (changed_faults[index] != nullptr) {
			publish_lifecycle_status(*changed_faults[index]);
			publish_runtime_effect(
				*changed_faults[index],
				changed_faults[index]->spec().sequence);
		}
	}

	fault_command_s command{};

	while (_fault_command_sub.update(&command)) {
		_last_command = command;
		_has_last_command = true;
		++_received_command_count;

		process_command(command);
	}
}


void FaultManager::process_imu_accel_bias_applied()
{
	for (std::size_t index = 0;
	     index < _imu_accel_bias_applied_subs.size();
	     ++index) {
		fault_imu_accel_bias_applied_s report{};

		if (_imu_accel_bias_applied_subs[index].update(&report)) {
			_last_imu_accel_bias_applied = report;
			_has_last_imu_accel_bias_applied = true;
			_last_imu_accel_bias_applied_accepted = false;
			++_received_imu_accel_bias_applied_count;

			process_imu_accel_bias_applied_report(
				index,
				report);
		}
	}
}

void FaultManager::process_imu_accel_bias_applied_report(
	std::size_t subscription_index,
	const fault_imu_accel_bias_applied_s &report)
{
	if (subscription_index
	    >= _imu_accel_bias_applied_baselines.size()) {
		++_rejected_imu_accel_bias_applied_count;
		return;
	}

	fault_injection::ImuAccelBiasFault &fault =
		_fault_registry.imu_accel_bias();

	if (!fault.has_definition()
	    || (report.estimator_instance == UINT8_MAX)) {
		++_rejected_imu_accel_bias_applied_count;
		return;
	}

	const fault_injection::FaultSpec &spec = fault.spec();

	const fault_injection::CumulativeFeedbackResult result =
		fault_injection::CumulativeFeedbackAggregator::consume(
			fault_injection::make_definition_identity(
				spec.scenario_id,
				spec.sequence,
				spec.fault_id),
			fault_injection::make_definition_identity(
				report.scenario_id,
				report.definition_sequence,
				report.fault_id),
			report.cumulative_applied_count,
			_imu_accel_bias_applied_baselines[
				subscription_index]);

	if (!result.accepted) {
		++_rejected_imu_accel_bias_applied_count;
		return;
	}

	_last_imu_accel_bias_applied_delta =
		result.delta;
	_last_imu_accel_bias_applied_accepted = true;
	++_accepted_imu_accel_bias_applied_count;

	if (result.delta > 0) {
		fault.record_applied(result.delta);
		_recorded_imu_accel_bias_sample_count +=
			result.delta;

		/*
		 * Publish an updated generic audit snapshot only when the
		 * authoritative applied_count changes.
		 */
		publish_lifecycle_status(fault);
	}
}

void FaultManager::process_imu_accel_noise_applied()
{
	for (std::size_t index = 0;
	     index < _imu_accel_noise_applied_subs.size();
	     ++index) {
		fault_imu_accel_noise_applied_s report{};

		if (_imu_accel_noise_applied_subs[index].update(&report)) {
			_last_imu_accel_noise_applied = report;
			_has_last_imu_accel_noise_applied = true;
			_last_imu_accel_noise_applied_accepted = false;
			++_received_imu_accel_noise_applied_count;

			process_imu_accel_noise_applied_report(index, report);
		}
	}
}

void FaultManager::process_imu_accel_noise_applied_report(
	std::size_t subscription_index,
	const fault_imu_accel_noise_applied_s &report)
{
	if (subscription_index
	    >= _imu_accel_noise_applied_baselines.size()) {
		++_rejected_imu_accel_noise_applied_count;
		return;
	}

	fault_injection::ImuAccelNoiseFault &fault =
		_fault_registry.imu_accel_noise();

	if (!fault.has_definition()
	    || (report.estimator_instance == UINT8_MAX)) {
		++_rejected_imu_accel_noise_applied_count;
		return;
	}

	const fault_injection::FaultSpec &spec = fault.spec();

	const fault_injection::CumulativeFeedbackResult result =
		fault_injection::CumulativeFeedbackAggregator::consume(
			fault_injection::make_definition_identity(
				spec.scenario_id,
				spec.sequence,
				spec.fault_id),
			fault_injection::make_definition_identity(
				report.scenario_id,
				report.definition_sequence,
				report.fault_id),
			report.cumulative_applied_count,
			_imu_accel_noise_applied_baselines[
				subscription_index]);

	if (!result.accepted) {
		++_rejected_imu_accel_noise_applied_count;
		return;
	}

	_last_imu_accel_noise_applied_delta =
		result.delta;
	_last_imu_accel_noise_applied_accepted = true;
	++_accepted_imu_accel_noise_applied_count;

	if (result.delta > 0) {
		fault.record_applied(result.delta);
		_recorded_imu_accel_noise_sample_count +=
			result.delta;
		publish_lifecycle_status(fault);
	}
}

void FaultManager::process_imu_gyro_noise_applied()
{
	for (std::size_t index = 0;
	     index < _imu_gyro_noise_applied_subs.size();
	     ++index) {
		fault_imu_gyro_noise_applied_s report{};

		if (_imu_gyro_noise_applied_subs[index].update(&report)) {
			_last_imu_gyro_noise_applied = report;
			_has_last_imu_gyro_noise_applied = true;
			_last_imu_gyro_noise_applied_accepted = false;
			++_received_imu_gyro_noise_applied_count;

			process_imu_gyro_noise_applied_report(index, report);
		}
	}
}

void FaultManager::process_imu_gyro_noise_applied_report(
	std::size_t subscription_index,
	const fault_imu_gyro_noise_applied_s &report)
{
	if (subscription_index
	    >= _imu_gyro_noise_applied_baselines.size()) {
		++_rejected_imu_gyro_noise_applied_count;
		return;
	}

	fault_injection::ImuGyroNoiseFault &fault =
		_fault_registry.imu_gyro_noise();

	if (!fault.has_definition()
	    || (report.estimator_instance == UINT8_MAX)) {
		++_rejected_imu_gyro_noise_applied_count;
		return;
	}

	const fault_injection::FaultSpec &spec = fault.spec();

	const fault_injection::CumulativeFeedbackResult result =
		fault_injection::CumulativeFeedbackAggregator::consume(
			fault_injection::make_definition_identity(
				spec.scenario_id,
				spec.sequence,
				spec.fault_id),
			fault_injection::make_definition_identity(
				report.scenario_id,
				report.definition_sequence,
				report.fault_id),
			report.cumulative_applied_count,
			_imu_gyro_noise_applied_baselines[
				subscription_index]);

	if (!result.accepted) {
		++_rejected_imu_gyro_noise_applied_count;
		return;
	}

	_last_imu_gyro_noise_applied_delta =
		result.delta;
	_last_imu_gyro_noise_applied_accepted = true;
	++_accepted_imu_gyro_noise_applied_count;

	if (result.delta > 0) {
		fault.record_applied(result.delta);
		_recorded_imu_gyro_noise_sample_count +=
			result.delta;
		publish_lifecycle_status(fault);
	}
}

void FaultManager::process_imu_gyro_bias_applied()
{
	for (std::size_t index = 0;
	     index < _imu_gyro_bias_applied_subs.size();
	     ++index) {
		fault_imu_gyro_bias_applied_s report{};

		if (_imu_gyro_bias_applied_subs[index].update(&report)) {
			_last_imu_gyro_bias_applied = report;
			_has_last_imu_gyro_bias_applied = true;
			_last_imu_gyro_bias_applied_accepted = false;
			++_received_imu_gyro_bias_applied_count;
			process_imu_gyro_bias_applied_report(index, report);
		}
	}
}

void FaultManager::process_imu_gyro_bias_applied_report(
	std::size_t subscription_index,
	const fault_imu_gyro_bias_applied_s &report)
{
	if (subscription_index >= _imu_gyro_bias_applied_baselines.size()) {
		++_rejected_imu_gyro_bias_applied_count;
		return;
	}

	fault_injection::ImuGyroBiasFault &fault = _fault_registry.imu_gyro_bias();

	if (!fault.has_definition() || (report.estimator_instance == UINT8_MAX)) {
		++_rejected_imu_gyro_bias_applied_count;
		return;
	}

	const fault_injection::FaultSpec &spec = fault.spec();
	const fault_injection::CumulativeFeedbackResult result =
		fault_injection::CumulativeFeedbackAggregator::consume(
			fault_injection::make_definition_identity(
				spec.scenario_id, spec.sequence, spec.fault_id),
			fault_injection::make_definition_identity(
				report.scenario_id,
				report.definition_sequence,
				report.fault_id),
			report.cumulative_applied_count,
			_imu_gyro_bias_applied_baselines[subscription_index]);

	if (!result.accepted) {
		++_rejected_imu_gyro_bias_applied_count;
		return;
	}

	_last_imu_gyro_bias_applied_delta = result.delta;
	_last_imu_gyro_bias_applied_accepted = true;
	++_accepted_imu_gyro_bias_applied_count;

	if (result.delta > 0) {
		fault.record_applied(result.delta);
		_recorded_imu_gyro_bias_sample_count += result.delta;
		publish_lifecycle_status(fault);
	}
}

void FaultManager::process_mag_bias_applied()
{
	for (std::size_t index = 0;
	     index < _mag_bias_applied_subs.size();
	     ++index) {
		fault_mag_bias_applied_s report{};

		if (_mag_bias_applied_subs[index].update(&report)) {
			_last_mag_bias_applied = report;
			_has_last_mag_bias_applied = true;
			_last_mag_bias_applied_accepted = false;
			++_received_mag_bias_applied_count;
			process_mag_bias_applied_report(index, report);
		}
	}
}

void FaultManager::process_mag_bias_applied_report(
	std::size_t subscription_index,
	const fault_mag_bias_applied_s &report)
{
	if (subscription_index >= _mag_bias_applied_baselines.size()) {
		++_rejected_mag_bias_applied_count;
		return;
	}

	fault_injection::MagBiasFault &fault = _fault_registry.mag_bias();

	if (!fault.has_definition() || (report.estimator_instance == UINT8_MAX)) {
		++_rejected_mag_bias_applied_count;
		return;
	}

	const fault_injection::FaultSpec &spec = fault.spec();
	const fault_injection::CumulativeFeedbackResult result =
		fault_injection::CumulativeFeedbackAggregator::consume(
			fault_injection::make_definition_identity(
				spec.scenario_id, spec.sequence, spec.fault_id),
			fault_injection::make_definition_identity(
				report.scenario_id,
				report.definition_sequence,
				report.fault_id),
			report.cumulative_applied_count,
			_mag_bias_applied_baselines[subscription_index]);

	if (!result.accepted) {
		++_rejected_mag_bias_applied_count;
		return;
	}

	_last_mag_bias_applied_delta = result.delta;
	_last_mag_bias_applied_accepted = true;
	++_accepted_mag_bias_applied_count;

	if (result.delta > 0) {
		fault.record_applied(result.delta);
		_recorded_mag_bias_sample_count += result.delta;
		publish_lifecycle_status(fault);
	}
}

void FaultManager::process_mag_noise_applied()
{
	for (std::size_t index = 0; index < _mag_noise_applied_subs.size(); ++index) {
		fault_mag_noise_applied_s report{};
		if (_mag_noise_applied_subs[index].update(&report)) {
			_last_mag_noise_applied = report;
			_has_last_mag_noise_applied = true;
			_last_mag_noise_applied_accepted = false;
			++_received_mag_noise_applied_count;
			process_mag_noise_applied_report(index, report);
		}
	}
}

void FaultManager::process_mag_noise_applied_report(
	std::size_t subscription_index,
	const fault_mag_noise_applied_s &report)
{
	if (subscription_index >= _mag_noise_applied_baselines.size()) {
		++_rejected_mag_noise_applied_count;
		return;
	}
	fault_injection::MagNoiseFault &fault = _fault_registry.mag_noise();
	if (!fault.has_definition() || (report.estimator_instance == UINT8_MAX)) {
		++_rejected_mag_noise_applied_count;
		return;
	}
	const fault_injection::FaultSpec &spec = fault.spec();
	const fault_injection::CumulativeFeedbackResult result =
		fault_injection::CumulativeFeedbackAggregator::consume(
			fault_injection::make_definition_identity(spec.scenario_id, spec.sequence, spec.fault_id),
			fault_injection::make_definition_identity(report.scenario_id, report.definition_sequence, report.fault_id),
			report.cumulative_applied_count,
			_mag_noise_applied_baselines[subscription_index]);
	if (!result.accepted) {
		++_rejected_mag_noise_applied_count;
		return;
	}
	_last_mag_noise_applied_delta = result.delta;
	_last_mag_noise_applied_accepted = true;
	++_accepted_mag_noise_applied_count;
	if (result.delta > 0) {
		fault.record_applied(result.delta);
		_recorded_mag_noise_sample_count += result.delta;
		publish_lifecycle_status(fault);
	}
}

void FaultManager::process_gnss_position_bias_applied()
{
	for (std::size_t index = 0;
	     index < _gnss_position_bias_applied_subs.size();
	     ++index) {
		fault_gnss_position_bias_applied_s report{};

		if (_gnss_position_bias_applied_subs[index].update(&report)) {
			_last_gnss_position_bias_applied = report;
			_has_last_gnss_position_bias_applied = true;
			_last_gnss_position_bias_applied_accepted = false;
			++_received_gnss_position_bias_applied_count;
			process_gnss_position_bias_applied_report(index, report);
		}
	}
}

void FaultManager::process_gnss_position_bias_applied_report(
	std::size_t subscription_index,
	const fault_gnss_position_bias_applied_s &report)
{
	if (subscription_index
	    >= _gnss_position_bias_applied_baselines.size()) {
		++_rejected_gnss_position_bias_applied_count;
		return;
	}

	fault_injection::GnssPositionBiasFault &fault =
		_fault_registry.gnss_position_bias();

	if (!fault.has_definition()
	    || (report.estimator_instance == UINT8_MAX)) {
		++_rejected_gnss_position_bias_applied_count;
		return;
	}

	const fault_injection::FaultSpec &spec = fault.spec();
	const fault_injection::CumulativeFeedbackResult result =
		fault_injection::CumulativeFeedbackAggregator::consume(
			fault_injection::make_definition_identity(
				spec.scenario_id,
				spec.sequence,
				spec.fault_id),
			fault_injection::make_definition_identity(
				report.scenario_id,
				report.definition_sequence,
				report.fault_id),
			report.cumulative_applied_count,
			_gnss_position_bias_applied_baselines[
				subscription_index]);

	if (!result.accepted) {
		++_rejected_gnss_position_bias_applied_count;
		return;
	}

	_last_gnss_position_bias_applied_delta = result.delta;
	_last_gnss_position_bias_applied_accepted = true;
	++_accepted_gnss_position_bias_applied_count;

	if (result.delta > 0) {
		fault.record_applied(result.delta);
		_recorded_gnss_position_bias_sample_count += result.delta;
		publish_lifecycle_status(fault);
	}
}

void FaultManager::process_gnss_position_noise_applied()
{
	for (std::size_t index = 0;
	     index < _gnss_position_noise_applied_subs.size();
	     ++index) {
		fault_gnss_position_noise_applied_s report{};

		if (_gnss_position_noise_applied_subs[index].update(&report)) {
			_last_gnss_position_noise_applied = report;
			_has_last_gnss_position_noise_applied = true;
			_last_gnss_position_noise_applied_accepted = false;
			++_received_gnss_position_noise_applied_count;
			process_gnss_position_noise_applied_report(index, report);
		}
	}
}

void FaultManager::process_gnss_position_noise_applied_report(
	std::size_t subscription_index,
	const fault_gnss_position_noise_applied_s &report)
{
	if (subscription_index
	    >= _gnss_position_noise_applied_baselines.size()) {
		++_rejected_gnss_position_noise_applied_count;
		return;
	}

	fault_injection::GnssPositionNoiseFault &fault =
		_fault_registry.gnss_position_noise();

	if (!fault.has_definition()
	    || (report.estimator_instance == UINT8_MAX)) {
		++_rejected_gnss_position_noise_applied_count;
		return;
	}

	const fault_injection::FaultSpec &spec = fault.spec();
	const fault_injection::CumulativeFeedbackResult result =
		fault_injection::CumulativeFeedbackAggregator::consume(
			fault_injection::make_definition_identity(
				spec.scenario_id,
				spec.sequence,
				spec.fault_id),
			fault_injection::make_definition_identity(
				report.scenario_id,
				report.definition_sequence,
				report.fault_id),
			report.cumulative_applied_count,
			_gnss_position_noise_applied_baselines[
				subscription_index]);

	if (!result.accepted) {
		++_rejected_gnss_position_noise_applied_count;
		return;
	}

	_last_gnss_position_noise_applied_delta = result.delta;
	_last_gnss_position_noise_applied_accepted = true;
	++_accepted_gnss_position_noise_applied_count;

	if (result.delta > 0) {
		fault.record_applied(result.delta);
		_recorded_gnss_position_noise_sample_count += result.delta;
		publish_lifecycle_status(fault);
	}
}

void FaultManager::process_gnss_velocity_bias_applied()
{
	for (std::size_t index = 0;
	     index < _gnss_velocity_bias_applied_subs.size();
	     ++index) {
		fault_gnss_velocity_bias_applied_s report{};

		if (_gnss_velocity_bias_applied_subs[index].update(&report)) {
			_last_gnss_velocity_bias_applied = report;
			_has_last_gnss_velocity_bias_applied = true;
			_last_gnss_velocity_bias_applied_accepted = false;
			++_received_gnss_velocity_bias_applied_count;
			process_gnss_velocity_bias_applied_report(index, report);
		}
	}
}

void FaultManager::process_gnss_velocity_bias_applied_report(
	std::size_t subscription_index,
	const fault_gnss_velocity_bias_applied_s &report)
{
	if (subscription_index
	    >= _gnss_velocity_bias_applied_baselines.size()) {
		++_rejected_gnss_velocity_bias_applied_count;
		return;
	}

	fault_injection::GnssVelocityBiasFault &fault =
		_fault_registry.gnss_velocity_bias();

	if (!fault.has_definition()
	    || (report.estimator_instance == UINT8_MAX)) {
		++_rejected_gnss_velocity_bias_applied_count;
		return;
	}

	const fault_injection::FaultSpec &spec = fault.spec();
	const fault_injection::CumulativeFeedbackResult result =
		fault_injection::CumulativeFeedbackAggregator::consume(
			fault_injection::make_definition_identity(
				spec.scenario_id,
				spec.sequence,
				spec.fault_id),
			fault_injection::make_definition_identity(
				report.scenario_id,
				report.definition_sequence,
				report.fault_id),
			report.cumulative_applied_count,
			_gnss_velocity_bias_applied_baselines[
				subscription_index]);

	if (!result.accepted) {
		++_rejected_gnss_velocity_bias_applied_count;
		return;
	}

	_last_gnss_velocity_bias_applied_delta = result.delta;
	_last_gnss_velocity_bias_applied_accepted = true;
	++_accepted_gnss_velocity_bias_applied_count;

	if (result.delta > 0) {
		fault.record_applied(result.delta);
		_recorded_gnss_velocity_bias_sample_count += result.delta;
		publish_lifecycle_status(fault);
	}
}

void FaultManager::process_gnss_velocity_noise_applied()
{
	for (std::size_t index = 0;
	     index < _gnss_velocity_noise_applied_subs.size();
	     ++index) {
		fault_gnss_velocity_noise_applied_s report{};

		if (_gnss_velocity_noise_applied_subs[index].update(&report)) {
			_last_gnss_velocity_noise_applied = report;
			_has_last_gnss_velocity_noise_applied = true;
			_last_gnss_velocity_noise_applied_accepted = false;
			++_received_gnss_velocity_noise_applied_count;
			process_gnss_velocity_noise_applied_report(index, report);
		}
	}
}

void FaultManager::process_gnss_velocity_noise_applied_report(
	std::size_t subscription_index,
	const fault_gnss_velocity_noise_applied_s &report)
{
	if (subscription_index
	    >= _gnss_velocity_noise_applied_baselines.size()) {
		++_rejected_gnss_velocity_noise_applied_count;
		return;
	}

	fault_injection::GnssVelocityNoiseFault &fault =
		_fault_registry.gnss_velocity_noise();

	if (!fault.has_definition()
	    || (report.estimator_instance == UINT8_MAX)) {
		++_rejected_gnss_velocity_noise_applied_count;
		return;
	}

	const fault_injection::FaultSpec &spec = fault.spec();
	const fault_injection::CumulativeFeedbackResult result =
		fault_injection::CumulativeFeedbackAggregator::consume(
			fault_injection::make_definition_identity(
				spec.scenario_id,
				spec.sequence,
				spec.fault_id),
			fault_injection::make_definition_identity(
				report.scenario_id,
				report.definition_sequence,
				report.fault_id),
			report.cumulative_applied_count,
			_gnss_velocity_noise_applied_baselines[
				subscription_index]);

	if (!result.accepted) {
		++_rejected_gnss_velocity_noise_applied_count;
		return;
	}

	_last_gnss_velocity_noise_applied_delta = result.delta;
	_last_gnss_velocity_noise_applied_accepted = true;
	++_accepted_gnss_velocity_noise_applied_count;

	if (result.delta > 0) {
		fault.record_applied(result.delta);
		_recorded_gnss_velocity_noise_sample_count += result.delta;
		publish_lifecycle_status(fault);
	}
}

void FaultManager::process_command(
	const fault_command_s &command)
{
	uint8_t result = fault_ack_s::RESULT_REJECTED;
	uint8_t reason = fault_ack_s::REASON_NONE;
	uint8_t state = fault_ack_s::STATE_REJECTED;

	const fault_injection::FaultInstance *status_faults[
		fault_injection::FaultRegistry::FaultCount
	]{};
	std::size_t status_count = 0;

	if (command.protocol_version != fault_command_s::PROTOCOL_VERSION) {
		reason = fault_ack_s::REASON_INVALID_PROTOCOL_VERSION;

	} else if ((command.action < fault_command_s::ACTION_SET)
		   || (command.action > fault_command_s::ACTION_QUERY)) {
		reason = fault_ack_s::REASON_INVALID_ACTION;

	} else {
		const uint64_t now_us = hrt_absolute_time();

		switch (command.action) {
		case fault_command_s::ACTION_SET: {
				fault_injection::FaultInstance *fault =
					_fault_registry.find(
						command.target,
						command.mode);

				if (fault == nullptr) {
					reason = _fault_registry.supports_target(
						command.target)
						? fault_ack_s::REASON_INVALID_MODE
						: fault_ack_s::REASON_INVALID_TARGET;
					break;
				}

				/*
				 * CLEAR and QUERY identify a definition by
				 * scenario_id + fault_id. Enforce global uniqueness across
				 * concrete object types so that identity never becomes
				 * ambiguous after the registry grows beyond one object.
				 */
				const fault_injection::FaultInstance *identity_fault =
					_fault_registry.find_by_identity(
						command.scenario_id,
						command.fault_id);

				if ((identity_fault != nullptr)
				    && (identity_fault != fault)) {
					result = fault_ack_s::RESULT_DUPLICATE;
					reason =
						fault_ack_s::REASON_DUPLICATE_FAULT_ID;
					state = identity_fault->state_code();
					break;
				}

				const fault_injection::FaultConfigureResult configure_result =
					fault->configure(
						make_fault_spec(command),
						now_us);

				if (configure_result
				    == fault_injection::FaultConfigureResult::Accepted) {
					result = fault_ack_s::RESULT_ACCEPTED;
					reason = fault_ack_s::REASON_NONE;
					state = fault->state_code();
					status_faults[status_count++] = fault;

				} else {
					reason =
						configure_result_to_ack_reason(
							configure_result);
				}

				break;
			}

		case fault_command_s::ACTION_CLEAR: {
				fault_injection::FaultInstance *fault =
					_fault_registry.find_by_identity(
						command.scenario_id,
						command.fault_id);

				if (fault == nullptr) {
					result = fault_ack_s::RESULT_NOT_FOUND;
					reason =
						fault_ack_s::REASON_UNKNOWN_FAULT_ID;
					state = fault_ack_s::STATE_EMPTY;
					break;
				}

				if ((fault->state()
				     != fault_injection::FaultState::Cleared)
				    && (fault->state()
					!= fault_injection::FaultState::Expired)) {
					fault->clear(now_us);
				}

				result = fault_ack_s::RESULT_ACCEPTED;
				reason = fault_ack_s::REASON_NONE;
				state = fault->state_code();
				status_faults[status_count++] = fault;
				break;
			}

		case fault_command_s::ACTION_CLEAR_ALL: {
				fault_injection::FaultInstance *cleared_faults[
					fault_injection::FaultRegistry::FaultCount
				]{};

				const std::size_t cleared_count =
					_fault_registry.clear_all(
						now_us,
						cleared_faults,
						fault_injection::FaultRegistry::FaultCount);

				result = fault_ack_s::RESULT_ACCEPTED;
				reason = fault_ack_s::REASON_NONE;
				state = (cleared_count > 0)
					? fault_ack_s::STATE_CLEARED
					: fault_ack_s::STATE_EMPTY;

				for (std::size_t index = 0;
				     (index < cleared_count)
				     && (index
					 < fault_injection::FaultRegistry::FaultCount);
				     ++index) {
					if (cleared_faults[index] != nullptr) {
						status_faults[status_count++] =
							cleared_faults[index];
					}
				}

				break;
			}

		case fault_command_s::ACTION_QUERY: {
				const fault_injection::FaultInstance *fault =
					_fault_registry.find_by_identity(
						command.scenario_id,
						command.fault_id);

				if (fault == nullptr) {
					result = fault_ack_s::RESULT_NOT_FOUND;
					reason =
						fault_ack_s::REASON_UNKNOWN_FAULT_ID;
					state = fault_ack_s::STATE_EMPTY;
					break;
				}

				result = fault_ack_s::RESULT_ACCEPTED;
				reason = fault_ack_s::REASON_NONE;
				state = fault->state_code();
				status_faults[status_count++] = fault;
				break;
			}

		default:
			reason = fault_ack_s::REASON_INVALID_ACTION;
			break;
		}
	}

	publish_ack(command, result, reason, state);

	for (std::size_t index = 0; index < status_count; ++index) {
		if (status_faults[index] != nullptr) {
			publish_status(
				*status_faults[index],
				command.source_system,
				command.source_component,
				command.sequence);

			/*
			 * QUERY is observational. It publishes the generic audit
			 * snapshot above, but it must not create a new runtime-effect
			 * event.
			 */
			if (command.action
			    != fault_command_s::ACTION_QUERY) {
				publish_runtime_effect(
					*status_faults[index],
					command.sequence);
			}
		}
	}
}

void FaultManager::process_downward_range_bias_applied()
{
	for (std::size_t index = 0; index < _downward_range_bias_applied_subs.size(); ++index) {
		fault_downward_range_bias_applied_s report{};
		if (_downward_range_bias_applied_subs[index].update(&report)) {
			process_downward_range_bias_applied_report(index, report);
		}
	}
}

void FaultManager::process_downward_range_bias_applied_report(
	std::size_t subscription_index,
	const fault_downward_range_bias_applied_s &report)
{
	if (subscription_index >= _downward_range_bias_applied_baselines.size()) { return; }
	fault_injection::DownwardRangeBiasFault &fault = _fault_registry.downward_range_bias();
	if (!fault.has_definition() || (report.estimator_instance == UINT8_MAX)) { return; }
	const fault_injection::FaultSpec &spec = fault.spec();
	const fault_injection::CumulativeFeedbackResult result =
		fault_injection::CumulativeFeedbackAggregator::consume(
			fault_injection::make_definition_identity(spec.scenario_id, spec.sequence, spec.fault_id),
			fault_injection::make_definition_identity(report.scenario_id, report.definition_sequence, report.fault_id),
			report.cumulative_applied_count,
			_downward_range_bias_applied_baselines[subscription_index]);
	if (result.accepted && (result.delta > 0)) {
		fault.record_applied(result.delta);
		publish_lifecycle_status(fault);
	}
}

void FaultManager::process_downward_range_noise_applied()
{
	for (std::size_t index = 0; index < _downward_range_noise_applied_subs.size(); ++index) {
		fault_downward_range_noise_applied_s report{};
		if (_downward_range_noise_applied_subs[index].update(&report)) {
			process_downward_range_noise_applied_report(index, report);
		}
	}
}

void FaultManager::process_downward_range_noise_applied_report(
	std::size_t subscription_index,
	const fault_downward_range_noise_applied_s &report)
{
	if (subscription_index >= _downward_range_noise_applied_baselines.size()) { return; }
	fault_injection::DownwardRangeNoiseFault &fault = _fault_registry.downward_range_noise();
	if (!fault.has_definition() || (report.estimator_instance == UINT8_MAX)) { return; }
	const fault_injection::FaultSpec &spec = fault.spec();
	const fault_injection::CumulativeFeedbackResult result =
		fault_injection::CumulativeFeedbackAggregator::consume(
			fault_injection::make_definition_identity(spec.scenario_id, spec.sequence, spec.fault_id),
			fault_injection::make_definition_identity(report.scenario_id, report.definition_sequence, report.fault_id),
			report.cumulative_applied_count,
			_downward_range_noise_applied_baselines[subscription_index]);
	if (result.accepted && (result.delta > 0)) {
		fault.record_applied(result.delta);
		publish_lifecycle_status(fault);
	}
}

void FaultManager::process_downward_range_degradation_applied()
{
	for (std::size_t index = 0; index < _downward_range_degradation_applied_subs.size(); ++index) {
		fault_downward_range_degradation_applied_s report{};
		if (_downward_range_degradation_applied_subs[index].update(&report)) {
			process_downward_range_degradation_applied_report(index, report);
		}
	}
}

void FaultManager::process_downward_range_degradation_applied_report(
	std::size_t subscription_index,
	const fault_downward_range_degradation_applied_s &report)
{
	if (subscription_index >= _downward_range_degradation_applied_baselines.size()) {
		return;
	}

	fault_injection::DownwardRangeDegradationFault &fault =
		_fault_registry.downward_range_degradation();

	if (!fault.has_definition() || (report.estimator_instance == UINT8_MAX)) {
		return;
	}

	const fault_injection::FaultSpec &spec = fault.spec();
	const fault_injection::CumulativeFeedbackResult result =
		fault_injection::CumulativeFeedbackAggregator::consume(
			fault_injection::make_definition_identity(
				spec.scenario_id, spec.sequence, spec.fault_id),
			fault_injection::make_definition_identity(
				report.scenario_id, report.definition_sequence, report.fault_id),
			report.cumulative_applied_count,
			_downward_range_degradation_applied_baselines[subscription_index]);

	if (result.accepted && (result.delta > 0)) {
		fault.record_applied(result.delta);
		publish_lifecycle_status(fault);
	}
}

void FaultManager::process_baro_altitude_bias_applied()
{
	for (std::size_t index = 0; index < _baro_altitude_bias_applied_subs.size(); ++index) {
		fault_baro_altitude_bias_applied_s report{};
		if (_baro_altitude_bias_applied_subs[index].update(&report)) {
			process_baro_altitude_bias_applied_report(index, report);
		}
	}
}

void FaultManager::process_baro_altitude_bias_applied_report(
	std::size_t subscription_index,
	const fault_baro_altitude_bias_applied_s &report)
{
	if (subscription_index >= _baro_altitude_bias_applied_baselines.size()) { return; }
	fault_injection::BaroAltitudeBiasFault &fault = _fault_registry.baro_altitude_bias();
	if (!fault.has_definition() || (report.estimator_instance == UINT8_MAX)) { return; }
	const fault_injection::FaultSpec &spec = fault.spec();
	const fault_injection::CumulativeFeedbackResult result =
		fault_injection::CumulativeFeedbackAggregator::consume(
			fault_injection::make_definition_identity(spec.scenario_id, spec.sequence, spec.fault_id),
			fault_injection::make_definition_identity(report.scenario_id, report.definition_sequence, report.fault_id),
			report.cumulative_applied_count,
			_baro_altitude_bias_applied_baselines[subscription_index]);
	if (result.accepted && (result.delta > 0)) {
		fault.record_applied(result.delta);
		publish_lifecycle_status(fault);
	}
}

void FaultManager::process_baro_altitude_noise_applied()
{
	for (std::size_t index = 0; index < _baro_altitude_noise_applied_subs.size(); ++index) {
		fault_baro_altitude_noise_applied_s report{};
		if (_baro_altitude_noise_applied_subs[index].update(&report)) {
			process_baro_altitude_noise_applied_report(index, report);
		}
	}
}

void FaultManager::process_baro_altitude_noise_applied_report(
	std::size_t subscription_index,
	const fault_baro_altitude_noise_applied_s &report)
{
	if (subscription_index >= _baro_altitude_noise_applied_baselines.size()) { return; }
	fault_injection::BaroAltitudeNoiseFault &fault = _fault_registry.baro_altitude_noise();
	if (!fault.has_definition() || (report.estimator_instance == UINT8_MAX)) { return; }
	const fault_injection::FaultSpec &spec = fault.spec();
	const fault_injection::CumulativeFeedbackResult result =
		fault_injection::CumulativeFeedbackAggregator::consume(
			fault_injection::make_definition_identity(spec.scenario_id, spec.sequence, spec.fault_id),
			fault_injection::make_definition_identity(report.scenario_id, report.definition_sequence, report.fault_id),
			report.cumulative_applied_count,
			_baro_altitude_noise_applied_baselines[subscription_index]);
	if (result.accepted && (result.delta > 0)) {
		fault.record_applied(result.delta);
		publish_lifecycle_status(fault);
	}
}


void FaultManager::process_motor_output_bias_applied()
{
	for (std::size_t index = 0; index < _motor_output_bias_applied_subs.size(); ++index) {
		fault_motor_output_bias_applied_s report{};

		if (_motor_output_bias_applied_subs[index].update(&report)) {
			process_motor_output_bias_applied_report(index, report);
		}
	}
}

void FaultManager::process_motor_output_bias_applied_report(
	std::size_t subscription_index,
	const fault_motor_output_bias_applied_s &report)
{
	if (subscription_index >= _motor_output_bias_applied_baselines.size()) {
		return;
	}

	fault_injection::MotorOutputBiasFault &fault = _fault_registry.motor_output_bias();

	if (!fault.has_definition() || (report.output_instance == UINT8_MAX)) {
		return;
	}

	const fault_injection::FaultSpec &spec = fault.spec();
	const fault_injection::CumulativeFeedbackResult result =
		fault_injection::CumulativeFeedbackAggregator::consume(
			fault_injection::make_definition_identity(
				spec.scenario_id, spec.sequence, spec.fault_id),
			fault_injection::make_definition_identity(
				report.scenario_id, report.definition_sequence, report.fault_id),
			report.cumulative_applied_count,
			_motor_output_bias_applied_baselines[subscription_index]);

	if (result.accepted && (result.delta > 0)) {
		fault.record_applied(result.delta);
		publish_lifecycle_status(fault);
	}
}

void FaultManager::process_motor_output_delay_applied()
{
	for (std::size_t index = 0; index < _motor_output_delay_applied_subs.size(); ++index) {
		fault_motor_output_delay_applied_s report{};

		if (_motor_output_delay_applied_subs[index].update(&report)) {
			process_motor_output_delay_applied_report(index, report);
		}
	}
}

void FaultManager::process_motor_output_delay_applied_report(
	std::size_t subscription_index,
	const fault_motor_output_delay_applied_s &report)
{
	if (subscription_index >= _motor_output_delay_applied_baselines.size()) {
		return;
	}

	fault_injection::MotorOutputDelayFault &fault = _fault_registry.motor_output_delay();

	if (!fault.has_definition() || (report.output_instance == UINT8_MAX)) {
		return;
	}

	const fault_injection::FaultSpec &spec = fault.spec();
	const fault_injection::CumulativeFeedbackResult result =
		fault_injection::CumulativeFeedbackAggregator::consume(
			fault_injection::make_definition_identity(
				spec.scenario_id, spec.sequence, spec.fault_id),
			fault_injection::make_definition_identity(
				report.scenario_id, report.definition_sequence, report.fault_id),
			report.cumulative_applied_count,
			_motor_output_delay_applied_baselines[subscription_index]);

	if (result.accepted && (result.delta > 0)) {
		fault.record_applied(result.delta);
		publish_lifecycle_status(fault);
	}
}

void FaultManager::process_motor_max_output_degradation_applied()
{
	for (std::size_t index = 0;
	     index < _motor_max_output_degradation_applied_subs.size();
	     ++index) {
		fault_motor_max_output_degradation_applied_s report{};

		if (_motor_max_output_degradation_applied_subs[index].update(&report)) {
			process_motor_max_output_degradation_applied_report(index, report);
		}
	}
}

void FaultManager::process_motor_max_output_degradation_applied_report(
	std::size_t subscription_index,
	const fault_motor_max_output_degradation_applied_s &report)
{
	if (subscription_index
	    >= _motor_max_output_degradation_applied_baselines.size()) {
		return;
	}

	fault_injection::MotorMaxOutputDegradationFault &fault =
		_fault_registry.motor_max_output_degradation();

	if (!fault.has_definition() || (report.output_instance == UINT8_MAX)) {
		return;
	}

	const fault_injection::FaultSpec &spec = fault.spec();
	const fault_injection::CumulativeFeedbackResult result =
		fault_injection::CumulativeFeedbackAggregator::consume(
			fault_injection::make_definition_identity(
				spec.scenario_id, spec.sequence, spec.fault_id),
			fault_injection::make_definition_identity(
				report.scenario_id, report.definition_sequence, report.fault_id),
			report.cumulative_applied_count,
			_motor_max_output_degradation_applied_baselines[subscription_index]);

	if (result.accepted && (result.delta > 0)) {
		fault.record_applied(result.delta);
		publish_lifecycle_status(fault);
	}
}

void FaultManager::publish_ack(
	const fault_command_s &command,
	uint8_t result,
	uint8_t reason,
	uint8_t state)
{
	fault_ack_s ack{};
	ack.timestamp = hrt_absolute_time();

	ack.scenario_id = command.scenario_id;
	ack.sequence = command.sequence;
	ack.fault_id = command.fault_id;

	ack.target_system = command.source_system;
	ack.target_component = command.source_component;

	ack.protocol_version = fault_ack_s::PROTOCOL_VERSION;
	ack.result = result;
	ack.reason = reason;
	ack.state = state;

	_fault_ack_pub.publish(ack);

	_last_ack = ack;
	_has_last_ack = true;
	++_published_ack_count;
}

void FaultManager::publish_status(
	const fault_injection::FaultInstance &fault,
	uint8_t target_system,
	uint8_t target_component,
	uint32_t sequence)
{
	if (!fault.has_definition()) {
		return;
	}

	const fault_injection::FaultSpec &spec = fault.spec();

	fault_status_s status{};
	status.timestamp = hrt_absolute_time();
	status.activation_time_us = fault.activation_time_us();
	status.expiration_time_us = fault.expiration_time_us();

	status.scenario_id = spec.scenario_id;
	status.sequence = sequence;
	status.applied_count = fault.applied_count();
	status.instance_mask = spec.instance_mask;
	status.random_seed = spec.random_seed;
	status.delay_us = spec.delay_us;

	status.vector[0] = spec.vector[0];
	status.vector[1] = spec.vector[1];
	status.vector[2] = spec.vector[2];
	status.scalar = spec.scalar;

	status.fault_id = spec.fault_id;

	status.target_system = target_system;
	status.target_component = target_component;
	status.protocol_version = fault_status_s::PROTOCOL_VERSION;
	status.target = spec.target;
	status.mode = spec.mode;
	status.state = fault.state_code();
	status.axis_mask = spec.axis_mask;

	_fault_status_pub.publish(status);

	_last_status = status;
	_has_last_status = true;
	++_published_status_count;
}

void FaultManager::publish_lifecycle_status(
	const fault_injection::FaultInstance &fault)
{
	const fault_injection::FaultSpec &spec = fault.spec();

	publish_status(
		fault,
		spec.source_system,
		spec.source_component,
		spec.sequence);
}

void FaultManager::publish_runtime_effect(
	const fault_injection::FaultInstance &fault,
	uint32_t sequence)
{
	const fault_injection::FaultInstance *imu_accel_bias_fault =
		static_cast<const fault_injection::FaultInstance *>(
			&_fault_registry.imu_accel_bias());

	if (&fault == imu_accel_bias_fault) {
		publish_imu_accel_bias_effect(
			_fault_registry.imu_accel_bias(),
			sequence);
		return;
	}

	const fault_injection::FaultInstance *imu_accel_noise_fault =
		static_cast<const fault_injection::FaultInstance *>(
			&_fault_registry.imu_accel_noise());

	if (&fault == imu_accel_noise_fault) {
		publish_imu_accel_noise_effect(
			_fault_registry.imu_accel_noise(),
			sequence);
		return;
	}
	const fault_injection::FaultInstance *imu_gyro_noise_fault =
		static_cast<const fault_injection::FaultInstance *>(
			&_fault_registry.imu_gyro_noise());

	if (&fault == imu_gyro_noise_fault) {
		publish_imu_gyro_noise_effect(
			_fault_registry.imu_gyro_noise(),
			sequence);
		return;
	}

	const fault_injection::FaultInstance *imu_gyro_bias_fault =
		static_cast<const fault_injection::FaultInstance *>(
			&_fault_registry.imu_gyro_bias());

	if (&fault == imu_gyro_bias_fault) {
		publish_imu_gyro_bias_effect(
			_fault_registry.imu_gyro_bias(),
			sequence);
	}
	const fault_injection::FaultInstance *mag_bias_fault =
		static_cast<const fault_injection::FaultInstance *>(
			&_fault_registry.mag_bias());

	if (&fault == mag_bias_fault) {
		publish_mag_bias_effect(
			_fault_registry.mag_bias(),
			sequence);
	}
	const fault_injection::FaultInstance *mag_noise_fault =
		static_cast<const fault_injection::FaultInstance *>(&_fault_registry.mag_noise());
	if (&fault == mag_noise_fault) {
		publish_mag_noise_effect(_fault_registry.mag_noise(), sequence);
		return;
	}

	const fault_injection::FaultInstance *gnss_position_bias_fault =
		static_cast<const fault_injection::FaultInstance *>(
			&_fault_registry.gnss_position_bias());

	if (&fault == gnss_position_bias_fault) {
		publish_gnss_position_bias_effect(
			_fault_registry.gnss_position_bias(),
			sequence);
		return;
	}

	const fault_injection::FaultInstance *gnss_position_noise_fault =
		static_cast<const fault_injection::FaultInstance *>(
			&_fault_registry.gnss_position_noise());

	if (&fault == gnss_position_noise_fault) {
		publish_gnss_position_noise_effect(
			_fault_registry.gnss_position_noise(),
			sequence);
	}

	const fault_injection::FaultInstance *gnss_velocity_bias_fault =
		static_cast<const fault_injection::FaultInstance *>(
			&_fault_registry.gnss_velocity_bias());

	if (&fault == gnss_velocity_bias_fault) {
		publish_gnss_velocity_bias_effect(
			_fault_registry.gnss_velocity_bias(),
			sequence);
		return;
	}

	const fault_injection::FaultInstance *gnss_velocity_noise_fault =
		static_cast<const fault_injection::FaultInstance *>(
			&_fault_registry.gnss_velocity_noise());

	if (&fault == gnss_velocity_noise_fault) {
		publish_gnss_velocity_noise_effect(
			_fault_registry.gnss_velocity_noise(),
			sequence);
		return;
	}

	if (&fault == static_cast<const fault_injection::FaultInstance *>(
			&_fault_registry.downward_range_bias())) {
		publish_downward_range_bias_effect(
			_fault_registry.downward_range_bias(), sequence);
		return;
	}

	if (&fault == static_cast<const fault_injection::FaultInstance *>(
			&_fault_registry.downward_range_noise())) {
		publish_downward_range_noise_effect(
			_fault_registry.downward_range_noise(), sequence);
		return;
	}

	if (&fault == static_cast<const fault_injection::FaultInstance *>(
			&_fault_registry.downward_range_degradation())) {
		publish_downward_range_degradation_effect(
			_fault_registry.downward_range_degradation(), sequence);
		return;
	}

	if (&fault == static_cast<const fault_injection::FaultInstance *>(
			&_fault_registry.baro_altitude_bias())) {
		publish_baro_altitude_bias_effect(
			_fault_registry.baro_altitude_bias(), sequence);
		return;
	}

	if (&fault == static_cast<const fault_injection::FaultInstance *>(
			&_fault_registry.baro_altitude_noise())) {
		publish_baro_altitude_noise_effect(
			_fault_registry.baro_altitude_noise(), sequence);
		return;
	}

	if (&fault == static_cast<const fault_injection::FaultInstance *>(
			&_fault_registry.motor_output_bias())) {
		publish_motor_output_bias_effect(
			_fault_registry.motor_output_bias(), sequence);
		return;
	}

	if (&fault == static_cast<const fault_injection::FaultInstance *>(
			&_fault_registry.motor_output_delay())) {
		publish_motor_output_delay_effect(
			_fault_registry.motor_output_delay(), sequence);
		return;
	}

	if (&fault == static_cast<const fault_injection::FaultInstance *>(
			&_fault_registry.motor_max_output_degradation())) {
		publish_motor_max_output_degradation_effect(
			_fault_registry.motor_max_output_degradation(), sequence);
	}
}

void FaultManager::publish_imu_accel_bias_effect(
	const fault_injection::ImuAccelBiasFault &fault,
	uint32_t sequence)
{
	fault_imu_accel_bias_s effect{};
	effect.timestamp = hrt_absolute_time();

	if (fault.has_definition()) {
		const fault_injection::FaultSpec &spec = fault.spec();

		effect.scenario_id = spec.scenario_id;
		effect.sequence = sequence;
		effect.fault_id = spec.fault_id;

		if (fault.is_active()) {
			effect.active = true;
			effect.axis_mask = spec.axis_mask;
			effect.bias_m_s2[0] = fault.bias_m_s2(0);
			effect.bias_m_s2[1] = fault.bias_m_s2(1);
			effect.bias_m_s2[2] = fault.bias_m_s2(2);
		}
	}

	/*
	 * Value initialization deliberately keeps EMPTY, PENDING, CLEARED, and
	 * EXPIRED snapshots neutral:
	 *
	 * active = false
	 * axis_mask = 0
	 * bias_m_s2 = [0, 0, 0]
	 */
	_imu_accel_bias_pub.publish(effect);

	_last_imu_accel_bias = effect;
	_has_last_imu_accel_bias = true;
	++_published_imu_accel_bias_count;
}

void FaultManager::publish_imu_accel_noise_effect(
	const fault_injection::ImuAccelNoiseFault &fault,
	uint32_t sequence)
{
	fault_imu_accel_noise_s effect{};
	effect.timestamp = hrt_absolute_time();

	if (fault.has_definition()) {
		const fault_injection::FaultSpec &spec = fault.spec();

		effect.scenario_id = spec.scenario_id;
		effect.sequence = sequence;
		effect.fault_id = spec.fault_id;

		if (fault.is_active()) {
			effect.random_seed = spec.random_seed;
			effect.active = true;
			effect.axis_mask = spec.axis_mask;
			effect.sigma_m_s2[0] = fault.sigma_m_s2(0);
			effect.sigma_m_s2[1] = fault.sigma_m_s2(1);
			effect.sigma_m_s2[2] = fault.sigma_m_s2(2);
		}
	}

	/*
	 * Value initialization keeps EMPTY, PENDING, CLEARED, and EXPIRED
	 * snapshots explicitly neutral.
	 */
	_imu_accel_noise_pub.publish(effect);

	_last_imu_accel_noise = effect;
	_has_last_imu_accel_noise = true;
	++_published_imu_accel_noise_count;
}

void FaultManager::publish_imu_gyro_bias_effect(
	const fault_injection::ImuGyroBiasFault &fault,
	uint32_t sequence)
{
	fault_imu_gyro_bias_s effect{};
	effect.timestamp = hrt_absolute_time();

	if (fault.has_definition()) {
		const fault_injection::FaultSpec &spec = fault.spec();
		effect.scenario_id = spec.scenario_id;
		effect.sequence = sequence;
		effect.fault_id = spec.fault_id;

		if (fault.is_active()) {
			effect.active = true;
			effect.axis_mask = spec.axis_mask;
			effect.bias_rad_s[0] = fault.bias_rad_s(0);
			effect.bias_rad_s[1] = fault.bias_rad_s(1);
			effect.bias_rad_s[2] = fault.bias_rad_s(2);
		}
	}

	_imu_gyro_bias_pub.publish(effect);
	_last_imu_gyro_bias = effect;
	_has_last_imu_gyro_bias = true;
	++_published_imu_gyro_bias_count;
}

void FaultManager::publish_imu_gyro_noise_effect(
	const fault_injection::ImuGyroNoiseFault &fault,
	uint32_t sequence)
{
	fault_imu_gyro_noise_s effect{};
	effect.timestamp = hrt_absolute_time();

	if (fault.has_definition()) {
		const fault_injection::FaultSpec &spec = fault.spec();

		effect.scenario_id = spec.scenario_id;
		effect.sequence = sequence;
		effect.fault_id = spec.fault_id;

		if (fault.is_active()) {
			effect.random_seed = spec.random_seed;
			effect.active = true;
			effect.axis_mask = spec.axis_mask;
			effect.sigma_rad_s[0] = fault.sigma_rad_s(0);
			effect.sigma_rad_s[1] = fault.sigma_rad_s(1);
			effect.sigma_rad_s[2] = fault.sigma_rad_s(2);
		}
	}

	/*
	 * Value initialization keeps EMPTY, PENDING, CLEARED, and EXPIRED
	 * snapshots explicitly neutral.
	 */
	_imu_gyro_noise_pub.publish(effect);

	_last_imu_gyro_noise = effect;
	_has_last_imu_gyro_noise = true;
	++_published_imu_gyro_noise_count;
}

void FaultManager::publish_mag_bias_effect(
	const fault_injection::MagBiasFault &fault,
	uint32_t sequence)
{
	fault_mag_bias_s effect{};
	effect.timestamp = hrt_absolute_time();

	if (fault.has_definition()) {
		const fault_injection::FaultSpec &spec = fault.spec();
		effect.scenario_id = spec.scenario_id;
		effect.sequence = sequence;
		effect.fault_id = spec.fault_id;

		if (fault.is_active()) {
			effect.active = true;
			effect.axis_mask = spec.axis_mask;
			effect.bias_ga[0] = fault.bias_ga(0);
			effect.bias_ga[1] = fault.bias_ga(1);
			effect.bias_ga[2] = fault.bias_ga(2);
		}
	}

	_mag_bias_pub.publish(effect);
	_last_mag_bias = effect;
	_has_last_mag_bias = true;
	++_published_mag_bias_count;
}

void FaultManager::publish_mag_noise_effect(
	const fault_injection::MagNoiseFault &fault,
	uint32_t sequence)
{
	fault_mag_noise_s effect{};
	effect.timestamp = hrt_absolute_time();
	if (fault.has_definition()) {
		const fault_injection::FaultSpec &spec = fault.spec();
		effect.scenario_id = spec.scenario_id;
		effect.sequence = sequence;
		effect.fault_id = spec.fault_id;
		if (fault.is_active()) {
			effect.random_seed = spec.random_seed;
			effect.active = true;
			effect.axis_mask = spec.axis_mask;
			effect.sigma_ga[0] = fault.sigma_ga(0);
			effect.sigma_ga[1] = fault.sigma_ga(1);
			effect.sigma_ga[2] = fault.sigma_ga(2);
		}
	}
	_mag_noise_pub.publish(effect);
	_last_mag_noise = effect;
	_has_last_mag_noise = true;
	++_published_mag_noise_count;
}

void FaultManager::publish_gnss_position_bias_effect(
	const fault_injection::GnssPositionBiasFault &fault,
	uint32_t sequence)
{
	fault_gnss_position_bias_s effect{};
	effect.timestamp = hrt_absolute_time();

	if (fault.has_definition()) {
		const fault_injection::FaultSpec &spec = fault.spec();
		effect.scenario_id = spec.scenario_id;
		effect.sequence = sequence;
		effect.fault_id = spec.fault_id;

		if (fault.is_active()) {
			effect.active = true;
			effect.axis_mask = spec.axis_mask;
			effect.bias_ned_m[0] = fault.bias_ned_m(0);
			effect.bias_ned_m[1] = fault.bias_ned_m(1);
			effect.bias_ned_m[2] = fault.bias_ned_m(2);
		}
	}

	_gnss_position_bias_pub.publish(effect);
	_last_gnss_position_bias = effect;
	_has_last_gnss_position_bias = true;
	++_published_gnss_position_bias_count;
}

void FaultManager::publish_gnss_position_noise_effect(
	const fault_injection::GnssPositionNoiseFault &fault,
	uint32_t sequence)
{
	fault_gnss_position_noise_s effect{};
	effect.timestamp = hrt_absolute_time();

	if (fault.has_definition()) {
		const fault_injection::FaultSpec &spec = fault.spec();
		effect.scenario_id = spec.scenario_id;
		effect.sequence = sequence;
		effect.fault_id = spec.fault_id;

		if (fault.is_active()) {
			effect.random_seed = spec.random_seed;
			effect.active = true;
			effect.axis_mask = spec.axis_mask;
			effect.sigma_ned_m[0] = fault.sigma_ned_m(0);
			effect.sigma_ned_m[1] = fault.sigma_ned_m(1);
			effect.sigma_ned_m[2] = fault.sigma_ned_m(2);
		}
	}

	_gnss_position_noise_pub.publish(effect);
	_last_gnss_position_noise = effect;
	_has_last_gnss_position_noise = true;
	++_published_gnss_position_noise_count;
}

void FaultManager::publish_gnss_velocity_bias_effect(
	const fault_injection::GnssVelocityBiasFault &fault,
	uint32_t sequence)
{
	fault_gnss_velocity_bias_s effect{};
	effect.timestamp = hrt_absolute_time();

	if (fault.has_definition()) {
		const fault_injection::FaultSpec &spec = fault.spec();
		effect.scenario_id = spec.scenario_id;
		effect.sequence = sequence;
		effect.fault_id = spec.fault_id;

		if (fault.is_active()) {
			effect.active = true;
			effect.axis_mask = spec.axis_mask;
			effect.bias_ned_m_s[0] = fault.bias_ned_m_s(0);
			effect.bias_ned_m_s[1] = fault.bias_ned_m_s(1);
			effect.bias_ned_m_s[2] = fault.bias_ned_m_s(2);
		}
	}

	_gnss_velocity_bias_pub.publish(effect);
	_last_gnss_velocity_bias = effect;
	_has_last_gnss_velocity_bias = true;
	++_published_gnss_velocity_bias_count;
}

void FaultManager::publish_gnss_velocity_noise_effect(
	const fault_injection::GnssVelocityNoiseFault &fault,
	uint32_t sequence)
{
	fault_gnss_velocity_noise_s effect{};
	effect.timestamp = hrt_absolute_time();

	if (fault.has_definition()) {
		const fault_injection::FaultSpec &spec = fault.spec();
		effect.scenario_id = spec.scenario_id;
		effect.sequence = sequence;
		effect.fault_id = spec.fault_id;

		if (fault.is_active()) {
			effect.random_seed = spec.random_seed;
			effect.active = true;
			effect.axis_mask = spec.axis_mask;
			effect.sigma_ned_m_s[0] = fault.sigma_ned_m_s(0);
			effect.sigma_ned_m_s[1] = fault.sigma_ned_m_s(1);
			effect.sigma_ned_m_s[2] = fault.sigma_ned_m_s(2);
		}
	}

	_gnss_velocity_noise_pub.publish(effect);
	_last_gnss_velocity_noise = effect;
	_has_last_gnss_velocity_noise = true;
	++_published_gnss_velocity_noise_count;
}

void FaultManager::publish_downward_range_bias_effect(
	const fault_injection::DownwardRangeBiasFault &fault, uint32_t sequence)
{
	fault_downward_range_bias_s effect{};
	effect.timestamp = hrt_absolute_time();
	if (fault.has_definition()) {
		const fault_injection::FaultSpec &spec = fault.spec();
		effect.scenario_id = spec.scenario_id;
		effect.sequence = sequence;
		effect.fault_id = spec.fault_id;
		if (fault.is_active()) {
			effect.active = true;
			effect.bias_m = fault.bias_m();
		}
	}
	_downward_range_bias_pub.publish(effect);
}

void FaultManager::publish_downward_range_noise_effect(
	const fault_injection::DownwardRangeNoiseFault &fault, uint32_t sequence)
{
	fault_downward_range_noise_s effect{};
	effect.timestamp = hrt_absolute_time();
	if (fault.has_definition()) {
		const fault_injection::FaultSpec &spec = fault.spec();
		effect.scenario_id = spec.scenario_id;
		effect.sequence = sequence;
		effect.fault_id = spec.fault_id;
		if (fault.is_active()) {
			effect.active = true;
			effect.random_seed = spec.random_seed;
			effect.sigma_m = fault.sigma_m();
		}
	}
	_downward_range_noise_pub.publish(effect);
}

void FaultManager::publish_downward_range_degradation_effect(
	const fault_injection::DownwardRangeDegradationFault &fault, uint32_t sequence)
{
	fault_downward_range_degradation_s effect{};
	effect.timestamp = hrt_absolute_time();
	if (fault.has_definition()) {
		const fault_injection::FaultSpec &spec = fault.spec();
		effect.scenario_id = spec.scenario_id;
		effect.sequence = sequence;
		effect.fault_id = spec.fault_id;
		if (fault.is_active()) {
			effect.active = true;
			effect.effective_max_distance_m = fault.effective_max_distance_m();
		}
	}
	_downward_range_degradation_pub.publish(effect);
}

int FaultManager::task_spawn(int argc, char *argv[])
{
	FaultManager *instance = new FaultManager();

	if (instance != nullptr) {
		_object.store(instance);
		_task_id = task_id_is_work_queue;

		if (instance->init()) {
			return PX4_OK;
		}

	} else {
		PX4_ERR("allocation failed");
	}

	delete instance;
	_object.store(nullptr);
	_task_id = -1;
	return PX4_ERROR;
}

void FaultManager::publish_baro_altitude_bias_effect(
	const fault_injection::BaroAltitudeBiasFault &fault, uint32_t sequence)
{
	fault_baro_altitude_bias_s effect{};
	effect.timestamp = hrt_absolute_time();
	if (fault.has_definition()) {
		const fault_injection::FaultSpec &spec = fault.spec();
		effect.scenario_id = spec.scenario_id;
		effect.sequence = sequence;
		effect.fault_id = spec.fault_id;
		if (fault.is_active()) {
			effect.active = true;
			effect.bias_m = fault.bias_m();
		}
	}
	_baro_altitude_bias_pub.publish(effect);
}

void FaultManager::publish_baro_altitude_noise_effect(
	const fault_injection::BaroAltitudeNoiseFault &fault, uint32_t sequence)
{
	fault_baro_altitude_noise_s effect{};
	effect.timestamp = hrt_absolute_time();
	if (fault.has_definition()) {
		const fault_injection::FaultSpec &spec = fault.spec();
		effect.scenario_id = spec.scenario_id;
		effect.sequence = sequence;
		effect.fault_id = spec.fault_id;
		if (fault.is_active()) {
			effect.active = true;
			effect.random_seed = spec.random_seed;
			effect.sigma_m = fault.sigma_m();
		}
	}
	_baro_altitude_noise_pub.publish(effect);
}

void FaultManager::publish_motor_output_bias_effect(
	const fault_injection::MotorOutputBiasFault &fault, uint32_t sequence)
{
	fault_motor_output_bias_s effect{};
	effect.timestamp = hrt_absolute_time();

	if (fault.has_definition()) {
		const fault_injection::FaultSpec &spec = fault.spec();
		effect.scenario_id = spec.scenario_id;
		effect.sequence = sequence;
		effect.fault_id = spec.fault_id;

		if (fault.is_active()) {
			effect.active = true;
			effect.motor_mask = fault.motor_mask();
			effect.bias_norm = fault.bias_norm();
		}
	}

	_motor_output_bias_pub.publish(effect);
}

void FaultManager::publish_motor_output_delay_effect(
	const fault_injection::MotorOutputDelayFault &fault, uint32_t sequence)
{
	fault_motor_output_delay_s effect{};
	effect.timestamp = hrt_absolute_time();

	if (fault.has_definition()) {
		const fault_injection::FaultSpec &spec = fault.spec();
		effect.scenario_id = spec.scenario_id;
		effect.sequence = sequence;
		effect.fault_id = spec.fault_id;

		if (fault.is_active()) {
			effect.active = true;
			effect.motor_mask = fault.motor_mask();
			effect.delay_us = fault.delay_us();
		}
	}

	_motor_output_delay_pub.publish(effect);
}

void FaultManager::publish_motor_max_output_degradation_effect(
	const fault_injection::MotorMaxOutputDegradationFault &fault,
	uint32_t sequence)
{
	fault_motor_max_output_degradation_s effect{};
	effect.timestamp = hrt_absolute_time();

	if (fault.has_definition()) {
		const fault_injection::FaultSpec &spec = fault.spec();
		effect.scenario_id = spec.scenario_id;
		effect.sequence = sequence;
		effect.fault_id = spec.fault_id;

		if (fault.is_active()) {
			effect.active = true;
			effect.motor_mask = fault.motor_mask();
			effect.max_output_norm = fault.max_output_norm();
		}
	}

	_motor_max_output_degradation_pub.publish(effect);
}

int FaultManager::print_status()
{
	PX4_INFO("running");
	PX4_INFO("preallocated fault-object status infrastructure");
	PX4_INFO("run count: %" PRIu64, _run_count);
	PX4_INFO(
		"received command count: %" PRIu64,
		_received_command_count);
	PX4_INFO(
		"published ack count: %" PRIu64,
		_published_ack_count);
	PX4_INFO(
		"published status count: %" PRIu64,
		_published_status_count);
	PX4_INFO(
		"published imu accel bias count: %" PRIu64,
		_published_imu_accel_bias_count);
	PX4_INFO(
		"published imu accel noise count: %" PRIu64,
		_published_imu_accel_noise_count);
	PX4_INFO(
		"received imu applied feedback count: %" PRIu64,
		_received_imu_accel_bias_applied_count);
	PX4_INFO(
		"accepted imu applied feedback count: %" PRIu64,
		_accepted_imu_accel_bias_applied_count);
	PX4_INFO(
		"rejected imu applied feedback count: %" PRIu64,
		_rejected_imu_accel_bias_applied_count);
	PX4_INFO(
		"recorded imu applied sample count: %" PRIu64,
		_recorded_imu_accel_bias_sample_count);
	PX4_INFO(
		"received imu noise applied feedback count: %" PRIu64,
		_received_imu_accel_noise_applied_count);
	PX4_INFO(
		"accepted imu noise applied feedback count: %" PRIu64,
		_accepted_imu_accel_noise_applied_count);
	PX4_INFO(
		"rejected imu noise applied feedback count: %" PRIu64,
		_rejected_imu_accel_noise_applied_count);
	PX4_INFO(
		"recorded imu noise applied sample count: %" PRIu64,
		_recorded_imu_accel_noise_sample_count);
	PX4_INFO(
		"published imu gyro bias count: %" PRIu64,
		_published_imu_gyro_bias_count);
	PX4_INFO(
		"received imu gyro bias feedback count: %" PRIu64,
		_received_imu_gyro_bias_applied_count);
	PX4_INFO(
		"accepted imu gyro bias feedback count: %" PRIu64,
		_accepted_imu_gyro_bias_applied_count);
	PX4_INFO(
		"rejected imu gyro bias feedback count: %" PRIu64,
		_rejected_imu_gyro_bias_applied_count);
	PX4_INFO(
		"recorded imu gyro bias sample count: %" PRIu64,
		_recorded_imu_gyro_bias_sample_count);
	PX4_INFO(
		"published mag bias count: %" PRIu64,
		_published_mag_bias_count);
	PX4_INFO(
		"received mag bias feedback count: %" PRIu64,
		_received_mag_bias_applied_count);
	PX4_INFO(
		"accepted mag bias feedback count: %" PRIu64,
		_accepted_mag_bias_applied_count);
	PX4_INFO(
		"rejected mag bias feedback count: %" PRIu64,
		_rejected_mag_bias_applied_count);
	PX4_INFO(
		"recorded mag bias sample count: %" PRIu64,
		_recorded_mag_bias_sample_count);
	PX4_INFO("published mag noise count: %" PRIu64, _published_mag_noise_count);
	PX4_INFO("received mag noise feedback count: %" PRIu64, _received_mag_noise_applied_count);
	PX4_INFO("accepted mag noise feedback count: %" PRIu64, _accepted_mag_noise_applied_count);
	PX4_INFO("rejected mag noise feedback count: %" PRIu64, _rejected_mag_noise_applied_count);
	PX4_INFO("recorded mag noise sample count: %" PRIu64, _recorded_mag_noise_sample_count);
	PX4_INFO("published gnss position bias count: %" PRIu64,
		 _published_gnss_position_bias_count);
	PX4_INFO("received gnss position bias feedback count: %" PRIu64,
		 _received_gnss_position_bias_applied_count);
	PX4_INFO("accepted gnss position bias feedback count: %" PRIu64,
		 _accepted_gnss_position_bias_applied_count);
	PX4_INFO("rejected gnss position bias feedback count: %" PRIu64,
		 _rejected_gnss_position_bias_applied_count);
	PX4_INFO("recorded gnss position bias sample count: %" PRIu64,
		 _recorded_gnss_position_bias_sample_count);
	PX4_INFO("published gnss position noise count: %" PRIu64,
		 _published_gnss_position_noise_count);
	PX4_INFO("received gnss position noise feedback count: %" PRIu64,
		 _received_gnss_position_noise_applied_count);
	PX4_INFO("accepted gnss position noise feedback count: %" PRIu64,
		 _accepted_gnss_position_noise_applied_count);
	PX4_INFO("rejected gnss position noise feedback count: %" PRIu64,
		 _rejected_gnss_position_noise_applied_count);
	PX4_INFO("recorded gnss position noise sample count: %" PRIu64,
		 _recorded_gnss_position_noise_sample_count);
	PX4_INFO("published gnss velocity bias count: %" PRIu64,
		 _published_gnss_velocity_bias_count);
	PX4_INFO("received gnss velocity bias feedback count: %" PRIu64,
		 _received_gnss_velocity_bias_applied_count);
	PX4_INFO("accepted gnss velocity bias feedback count: %" PRIu64,
		 _accepted_gnss_velocity_bias_applied_count);
	PX4_INFO("rejected gnss velocity bias feedback count: %" PRIu64,
		 _rejected_gnss_velocity_bias_applied_count);
	PX4_INFO("recorded gnss velocity bias sample count: %" PRIu64,
		 _recorded_gnss_velocity_bias_sample_count);
	PX4_INFO("published gnss velocity noise count: %" PRIu64,
		 _published_gnss_velocity_noise_count);
	PX4_INFO("received gnss velocity noise feedback count: %" PRIu64,
		 _received_gnss_velocity_noise_applied_count);
	PX4_INFO("accepted gnss velocity noise feedback count: %" PRIu64,
		 _accepted_gnss_velocity_noise_applied_count);
	PX4_INFO("rejected gnss velocity noise feedback count: %" PRIu64,
		 _rejected_gnss_velocity_noise_applied_count);
	PX4_INFO("recorded gnss velocity noise sample count: %" PRIu64,
		 _recorded_gnss_velocity_noise_sample_count);
	PX4_INFO(
		"published imu gyro noise count: %" PRIu64,
		_published_imu_gyro_noise_count);
	PX4_INFO(
		"received imu gyro noise feedback count: %" PRIu64,
		_received_imu_gyro_noise_applied_count);
	PX4_INFO(
		"accepted imu gyro noise feedback count: %" PRIu64,
		_accepted_imu_gyro_noise_applied_count);
	PX4_INFO(
		"rejected imu gyro noise feedback count: %" PRIu64,
		_rejected_imu_gyro_noise_applied_count);
	PX4_INFO(
		"recorded imu gyro noise sample count: %" PRIu64,
		_recorded_imu_gyro_noise_sample_count);

	if (_has_last_command) {
		PX4_INFO(
			"last command: scenario=%" PRIu32
			" sequence=%" PRIu32
			" fault=%" PRIu16
			" action=%" PRIu8
			" target=%" PRIu8
			" mode=%" PRIu8,
			_last_command.scenario_id,
			_last_command.sequence,
			_last_command.fault_id,
			_last_command.action,
			_last_command.target,
			_last_command.mode);

		PX4_INFO(
			"last route: source=%" PRIu8 "/%" PRIu8
			" target=%" PRIu8 "/%" PRIu8
			" protocol=%" PRIu8,
			_last_command.source_system,
			_last_command.source_component,
			_last_command.target_system,
			_last_command.target_component,
			_last_command.protocol_version);

	} else {
		PX4_INFO("last command: none");
	}

	if (_has_last_ack) {
		PX4_INFO(
			"last ack: scenario=%" PRIu32
			" sequence=%" PRIu32
			" fault=%" PRIu16
			" result=%" PRIu8
			" reason=%" PRIu8
			" state=%" PRIu8,
			_last_ack.scenario_id,
			_last_ack.sequence,
			_last_ack.fault_id,
			_last_ack.result,
			_last_ack.reason,
			_last_ack.state);

		PX4_INFO(
			"last ack route: target=%" PRIu8 "/%" PRIu8
			" protocol=%" PRIu8,
			_last_ack.target_system,
			_last_ack.target_component,
			_last_ack.protocol_version);

	} else {
		PX4_INFO("last ack: none");
	}

	if (_has_last_status) {
		PX4_INFO(
			"last status: scenario=%" PRIu32
			" sequence=%" PRIu32
			" fault=%" PRIu16
			" state=%" PRIu8
			" applied=%" PRIu32,
			_last_status.scenario_id,
			_last_status.sequence,
			_last_status.fault_id,
			_last_status.state,
			_last_status.applied_count);

		PX4_INFO(
			"last status route: target=%" PRIu8 "/%" PRIu8
			" protocol=%" PRIu8,
			_last_status.target_system,
			_last_status.target_component,
			_last_status.protocol_version);

	} else {
		PX4_INFO("last status: none");
	}

	if (_has_last_imu_accel_bias) {
		PX4_INFO(
			"last imu effect: scenario=%" PRIu32
			" sequence=%" PRIu32
			" fault=%" PRIu16
			" active=%u"
			" axis_mask=%" PRIu8,
			_last_imu_accel_bias.scenario_id,
			_last_imu_accel_bias.sequence,
			_last_imu_accel_bias.fault_id,
			(unsigned)_last_imu_accel_bias.active,
			_last_imu_accel_bias.axis_mask);

		PX4_INFO(
			"last imu effect bias: [%.6f, %.6f, %.6f] m/s^2",
			(double)_last_imu_accel_bias.bias_m_s2[0],
			(double)_last_imu_accel_bias.bias_m_s2[1],
			(double)_last_imu_accel_bias.bias_m_s2[2]);

	} else {
		PX4_INFO("last imu effect: none");
	}

	if (_has_last_imu_accel_noise) {
		PX4_INFO(
			"last imu noise effect: scenario=%" PRIu32
			" sequence=%" PRIu32
			" fault=%" PRIu16
			" active=%u axis_mask=%" PRIu8
			" seed=%" PRIu32,
			_last_imu_accel_noise.scenario_id,
			_last_imu_accel_noise.sequence,
			_last_imu_accel_noise.fault_id,
			(unsigned)_last_imu_accel_noise.active,
			_last_imu_accel_noise.axis_mask,
			_last_imu_accel_noise.random_seed);

		PX4_INFO(
			"last imu noise sigma: [%.6f, %.6f, %.6f] m/s^2",
			(double)_last_imu_accel_noise.sigma_m_s2[0],
			(double)_last_imu_accel_noise.sigma_m_s2[1],
			(double)_last_imu_accel_noise.sigma_m_s2[2]);

	} else {
		PX4_INFO("last imu noise effect: none");
	}

	if (_has_last_imu_gyro_noise) {
		PX4_INFO(
			"last imu gyro noise effect: scenario=%" PRIu32
			" sequence=%" PRIu32
			" fault=%" PRIu16
			" active=%u axis_mask=%" PRIu8
			" seed=%" PRIu32,
			_last_imu_gyro_noise.scenario_id,
			_last_imu_gyro_noise.sequence,
			_last_imu_gyro_noise.fault_id,
			(unsigned)_last_imu_gyro_noise.active,
			_last_imu_gyro_noise.axis_mask,
			_last_imu_gyro_noise.random_seed);

		PX4_INFO(
			"last imu gyro noise sigma: [%.6f, %.6f, %.6f] rad/s",
			(double)_last_imu_gyro_noise.sigma_rad_s[0],
			(double)_last_imu_gyro_noise.sigma_rad_s[1],
			(double)_last_imu_gyro_noise.sigma_rad_s[2]);

	} else {
		PX4_INFO("last imu gyro noise effect: none");
	}

	if (_has_last_imu_gyro_bias) {
		PX4_INFO(
			"last imu gyro bias effect: scenario=%" PRIu32
			" sequence=%" PRIu32
			" fault=%" PRIu16
			" active=%u axis_mask=%" PRIu8,
			_last_imu_gyro_bias.scenario_id,
			_last_imu_gyro_bias.sequence,
			_last_imu_gyro_bias.fault_id,
			(unsigned)_last_imu_gyro_bias.active,
			_last_imu_gyro_bias.axis_mask);

		PX4_INFO(
			"last imu gyro bias vector: [%.6f, %.6f, %.6f] rad/s",
			(double)_last_imu_gyro_bias.bias_rad_s[0],
			(double)_last_imu_gyro_bias.bias_rad_s[1],
			(double)_last_imu_gyro_bias.bias_rad_s[2]);

	} else {
		PX4_INFO("last imu gyro bias effect: none");
	}

	if (_has_last_imu_accel_bias_applied) {
		PX4_INFO(
			"last imu applied feedback: scenario=%" PRIu32
			" definition_sequence=%" PRIu32
			" fault=%" PRIu16
			" estimator=%" PRIu8
			" cumulative=%" PRIu32
			" delta=%" PRIu32
			" accepted=%u"
			" final=%u",
			_last_imu_accel_bias_applied.scenario_id,
			_last_imu_accel_bias_applied.definition_sequence,
			_last_imu_accel_bias_applied.fault_id,
			_last_imu_accel_bias_applied.estimator_instance,
			_last_imu_accel_bias_applied.cumulative_applied_count,
			_last_imu_accel_bias_applied_delta,
			(unsigned)_last_imu_accel_bias_applied_accepted,
			(unsigned)_last_imu_accel_bias_applied.final_report);

		PX4_INFO(
			"last imu applied timestamp_sample=%" PRIu64,
			_last_imu_accel_bias_applied.timestamp_sample);

	} else {
		PX4_INFO("last imu applied feedback: none");
	}

	if (_has_last_imu_accel_noise_applied) {
		PX4_INFO(
			"last imu noise applied feedback: scenario=%" PRIu32
			" definition_sequence=%" PRIu32
			" fault=%" PRIu16
			" estimator=%" PRIu8
			" cumulative=%" PRIu32
			" delta=%" PRIu32
			" accepted=%u final=%u",
			_last_imu_accel_noise_applied.scenario_id,
			_last_imu_accel_noise_applied.definition_sequence,
			_last_imu_accel_noise_applied.fault_id,
			_last_imu_accel_noise_applied.estimator_instance,
			_last_imu_accel_noise_applied.cumulative_applied_count,
			_last_imu_accel_noise_applied_delta,
			(unsigned)_last_imu_accel_noise_applied_accepted,
			(unsigned)_last_imu_accel_noise_applied.final_report);

		PX4_INFO(
			"last imu noise applied timestamp_sample=%" PRIu64,
			_last_imu_accel_noise_applied.timestamp_sample);

	} else {
		PX4_INFO("last imu noise applied feedback: none");
	}

	if (_has_last_mag_bias) {
		PX4_INFO(
			"last mag bias effect: scenario=%" PRIu32
			" sequence=%" PRIu32
			" fault=%" PRIu16
			" active=%u axis_mask=%" PRIu8,
			_last_mag_bias.scenario_id,
			_last_mag_bias.sequence,
			_last_mag_bias.fault_id,
			(unsigned)_last_mag_bias.active,
			_last_mag_bias.axis_mask);

		PX4_INFO(
			"last mag bias vector: [%.6f, %.6f, %.6f] Gauss",
			(double)_last_mag_bias.bias_ga[0],
			(double)_last_mag_bias.bias_ga[1],
			(double)_last_mag_bias.bias_ga[2]);

	} else {
		PX4_INFO("last mag bias effect: none");
	}

	if (_has_last_imu_accel_bias_applied) {
		PX4_INFO(
			"last imu applied feedback: scenario=%" PRIu32
			" definition_sequence=%" PRIu32
			" fault=%" PRIu16
			" estimator=%" PRIu8
			" cumulative=%" PRIu32
			" delta=%" PRIu32
			" accepted=%u"
			" final=%u",
			_last_imu_accel_bias_applied.scenario_id,
			_last_imu_accel_bias_applied.definition_sequence,
			_last_imu_accel_bias_applied.fault_id,
			_last_imu_accel_bias_applied.estimator_instance,
			_last_imu_accel_bias_applied.cumulative_applied_count,
			_last_imu_accel_bias_applied_delta,
			(unsigned)_last_imu_accel_bias_applied_accepted,
			(unsigned)_last_imu_accel_bias_applied.final_report);

		PX4_INFO(
			"last imu applied timestamp_sample=%" PRIu64,
			_last_imu_accel_bias_applied.timestamp_sample);

	} else {
		PX4_INFO("last imu applied feedback: none");
	}

	if (_has_last_imu_accel_noise_applied) {
		PX4_INFO(
			"last imu noise applied feedback: scenario=%" PRIu32
			" definition_sequence=%" PRIu32
			" fault=%" PRIu16
			" estimator=%" PRIu8
			" cumulative=%" PRIu32
			" delta=%" PRIu32
			" accepted=%u final=%u",
			_last_imu_accel_noise_applied.scenario_id,
			_last_imu_accel_noise_applied.definition_sequence,
			_last_imu_accel_noise_applied.fault_id,
			_last_imu_accel_noise_applied.estimator_instance,
			_last_imu_accel_noise_applied.cumulative_applied_count,
			_last_imu_accel_noise_applied_delta,
			(unsigned)_last_imu_accel_noise_applied_accepted,
			(unsigned)_last_imu_accel_noise_applied.final_report);

		PX4_INFO(
			"last imu noise applied timestamp_sample=%" PRIu64,
			_last_imu_accel_noise_applied.timestamp_sample);

	} else {
		PX4_INFO("last imu noise applied feedback: none");
	}

	if (_has_last_imu_gyro_noise_applied) {
		PX4_INFO(
			"last imu gyro noise applied feedback: scenario=%" PRIu32
			" definition_sequence=%" PRIu32
			" fault=%" PRIu16
			" estimator=%" PRIu8
			" cumulative=%" PRIu32
			" delta=%" PRIu32
			" accepted=%u final=%u",
			_last_imu_gyro_noise_applied.scenario_id,
			_last_imu_gyro_noise_applied.definition_sequence,
			_last_imu_gyro_noise_applied.fault_id,
			_last_imu_gyro_noise_applied.estimator_instance,
			_last_imu_gyro_noise_applied.cumulative_applied_count,
			_last_imu_gyro_noise_applied_delta,
			(unsigned)_last_imu_gyro_noise_applied_accepted,
			(unsigned)_last_imu_gyro_noise_applied.final_report);

		PX4_INFO(
			"last imu gyro noise applied timestamp_sample=%" PRIu64,
			_last_imu_gyro_noise_applied.timestamp_sample);

	} else {
		PX4_INFO("last imu gyro noise applied feedback: none");
	}

	if (_has_last_imu_gyro_bias_applied) {
		PX4_INFO(
			"last imu gyro bias feedback: scenario=%" PRIu32
			" definition_sequence=%" PRIu32
			" fault=%" PRIu16
			" estimator=%" PRIu8
			" cumulative=%" PRIu32
			" delta=%" PRIu32
			" accepted=%u final=%u",
			_last_imu_gyro_bias_applied.scenario_id,
			_last_imu_gyro_bias_applied.definition_sequence,
			_last_imu_gyro_bias_applied.fault_id,
			_last_imu_gyro_bias_applied.estimator_instance,
			_last_imu_gyro_bias_applied.cumulative_applied_count,
			_last_imu_gyro_bias_applied_delta,
			(unsigned)_last_imu_gyro_bias_applied_accepted,
			(unsigned)_last_imu_gyro_bias_applied.final_report);

		PX4_INFO(
			"last imu gyro bias timestamp_sample=%" PRIu64,
			_last_imu_gyro_bias_applied.timestamp_sample);

	} else {
		PX4_INFO("last imu gyro bias feedback: none");
	}

	if (_has_last_mag_bias_applied) {
		PX4_INFO(
			"last mag bias feedback: scenario=%" PRIu32
			" definition_sequence=%" PRIu32
			" fault=%" PRIu16
			" estimator=%" PRIu8
			" cumulative=%" PRIu32
			" delta=%" PRIu32
			" accepted=%u final=%u",
			_last_mag_bias_applied.scenario_id,
			_last_mag_bias_applied.definition_sequence,
			_last_mag_bias_applied.fault_id,
			_last_mag_bias_applied.estimator_instance,
			_last_mag_bias_applied.cumulative_applied_count,
			_last_mag_bias_applied_delta,
			(unsigned)_last_mag_bias_applied_accepted,
			(unsigned)_last_mag_bias_applied.final_report);

		PX4_INFO(
			"last mag bias timestamp_sample=%" PRIu64,
			_last_mag_bias_applied.timestamp_sample);

	} else {
		PX4_INFO("last mag bias feedback: none");
	}

	if (_has_last_gnss_position_bias) {
		PX4_INFO(
			"last gnss position bias effect: scenario=%" PRIu32
			" sequence=%" PRIu32 " fault=%" PRIu16
			" active=%u axis_mask=%" PRIu8,
			_last_gnss_position_bias.scenario_id,
			_last_gnss_position_bias.sequence,
			_last_gnss_position_bias.fault_id,
			(unsigned)_last_gnss_position_bias.active,
			_last_gnss_position_bias.axis_mask);
		PX4_INFO("last gnss position bias: [%.6f, %.6f, %.6f] m NED",
			(double)_last_gnss_position_bias.bias_ned_m[0],
			(double)_last_gnss_position_bias.bias_ned_m[1],
			(double)_last_gnss_position_bias.bias_ned_m[2]);
	}

	if (_has_last_gnss_position_noise) {
		PX4_INFO(
			"last gnss position noise effect: scenario=%" PRIu32
			" sequence=%" PRIu32 " fault=%" PRIu16
			" active=%u axis_mask=%" PRIu8 " seed=%" PRIu32,
			_last_gnss_position_noise.scenario_id,
			_last_gnss_position_noise.sequence,
			_last_gnss_position_noise.fault_id,
			(unsigned)_last_gnss_position_noise.active,
			_last_gnss_position_noise.axis_mask,
			_last_gnss_position_noise.random_seed);
		PX4_INFO("last gnss position noise sigma: [%.6f, %.6f, %.6f] m NED",
			(double)_last_gnss_position_noise.sigma_ned_m[0],
			(double)_last_gnss_position_noise.sigma_ned_m[1],
			(double)_last_gnss_position_noise.sigma_ned_m[2]);
	}

	if (_has_last_gnss_position_bias_applied) {
		PX4_INFO(
			"last gnss position bias feedback: scenario=%" PRIu32
			" definition_sequence=%" PRIu32 " fault=%" PRIu16
			" estimator=%" PRIu8 " cumulative=%" PRIu32
			" delta=%" PRIu32 " accepted=%u final=%u",
			_last_gnss_position_bias_applied.scenario_id,
			_last_gnss_position_bias_applied.definition_sequence,
			_last_gnss_position_bias_applied.fault_id,
			_last_gnss_position_bias_applied.estimator_instance,
			_last_gnss_position_bias_applied.cumulative_applied_count,
			_last_gnss_position_bias_applied_delta,
			(unsigned)_last_gnss_position_bias_applied_accepted,
			(unsigned)_last_gnss_position_bias_applied.final_report);
	}

	if (_has_last_gnss_position_noise_applied) {
		PX4_INFO(
			"last gnss position noise feedback: scenario=%" PRIu32
			" definition_sequence=%" PRIu32 " fault=%" PRIu16
			" estimator=%" PRIu8 " cumulative=%" PRIu32
			" delta=%" PRIu32 " accepted=%u final=%u",
			_last_gnss_position_noise_applied.scenario_id,
			_last_gnss_position_noise_applied.definition_sequence,
			_last_gnss_position_noise_applied.fault_id,
			_last_gnss_position_noise_applied.estimator_instance,
			_last_gnss_position_noise_applied.cumulative_applied_count,
			_last_gnss_position_noise_applied_delta,
			(unsigned)_last_gnss_position_noise_applied_accepted,
			(unsigned)_last_gnss_position_noise_applied.final_report);
	}

	if (_has_last_gnss_velocity_bias) {
		PX4_INFO(
			"last gnss velocity bias effect: scenario=%" PRIu32
			" sequence=%" PRIu32 " fault=%" PRIu16
			" active=%u axis_mask=%" PRIu8,
			_last_gnss_velocity_bias.scenario_id,
			_last_gnss_velocity_bias.sequence,
			_last_gnss_velocity_bias.fault_id,
			(unsigned)_last_gnss_velocity_bias.active,
			_last_gnss_velocity_bias.axis_mask);
		PX4_INFO("last gnss velocity bias: [%.6f, %.6f, %.6f] m/s NED",
			(double)_last_gnss_velocity_bias.bias_ned_m_s[0],
			(double)_last_gnss_velocity_bias.bias_ned_m_s[1],
			(double)_last_gnss_velocity_bias.bias_ned_m_s[2]);
	}

	if (_has_last_gnss_velocity_noise) {
		PX4_INFO(
			"last gnss velocity noise effect: scenario=%" PRIu32
			" sequence=%" PRIu32 " fault=%" PRIu16
			" active=%u axis_mask=%" PRIu8 " seed=%" PRIu32,
			_last_gnss_velocity_noise.scenario_id,
			_last_gnss_velocity_noise.sequence,
			_last_gnss_velocity_noise.fault_id,
			(unsigned)_last_gnss_velocity_noise.active,
			_last_gnss_velocity_noise.axis_mask,
			_last_gnss_velocity_noise.random_seed);
		PX4_INFO("last gnss velocity noise sigma: [%.6f, %.6f, %.6f] m/s NED",
			(double)_last_gnss_velocity_noise.sigma_ned_m_s[0],
			(double)_last_gnss_velocity_noise.sigma_ned_m_s[1],
			(double)_last_gnss_velocity_noise.sigma_ned_m_s[2]);
	}

	if (_has_last_gnss_velocity_bias_applied) {
		PX4_INFO(
			"last gnss velocity bias feedback: scenario=%" PRIu32
			" definition_sequence=%" PRIu32 " fault=%" PRIu16
			" estimator=%" PRIu8 " cumulative=%" PRIu32
			" delta=%" PRIu32 " accepted=%u final=%u",
			_last_gnss_velocity_bias_applied.scenario_id,
			_last_gnss_velocity_bias_applied.definition_sequence,
			_last_gnss_velocity_bias_applied.fault_id,
			_last_gnss_velocity_bias_applied.estimator_instance,
			_last_gnss_velocity_bias_applied.cumulative_applied_count,
			_last_gnss_velocity_bias_applied_delta,
			(unsigned)_last_gnss_velocity_bias_applied_accepted,
			(unsigned)_last_gnss_velocity_bias_applied.final_report);
	}

	if (_has_last_gnss_velocity_noise_applied) {
		PX4_INFO(
			"last gnss velocity noise feedback: scenario=%" PRIu32
			" definition_sequence=%" PRIu32 " fault=%" PRIu16
			" estimator=%" PRIu8 " cumulative=%" PRIu32
			" delta=%" PRIu32 " accepted=%u final=%u",
			_last_gnss_velocity_noise_applied.scenario_id,
			_last_gnss_velocity_noise_applied.definition_sequence,
			_last_gnss_velocity_noise_applied.fault_id,
			_last_gnss_velocity_noise_applied.estimator_instance,
			_last_gnss_velocity_noise_applied.cumulative_applied_count,
			_last_gnss_velocity_noise_applied_delta,
			(unsigned)_last_gnss_velocity_noise_applied_accepted,
			(unsigned)_last_gnss_velocity_noise_applied.final_report);
	}

	const fault_injection::NoEffectFault &no_effect =
		_fault_registry.no_effect();

	if (no_effect.has_definition()) {
		const fault_injection::FaultSpec &spec = no_effect.spec();

		PX4_INFO(
			"no-effect object: scenario=%" PRIu32
			" sequence=%" PRIu32
			" fault=%" PRIu16
			" state=%" PRIu8,
			spec.scenario_id,
			spec.sequence,
			spec.fault_id,
			no_effect.state_code());

		PX4_INFO(
			"no-effect timing: activation=%" PRIu64
			" expiration=%" PRIu64
			" applied=%" PRIu32,
			no_effect.activation_time_us(),
			no_effect.expiration_time_us(),
			no_effect.applied_count());

	} else {
		PX4_INFO("no-effect object: empty");
	}

	const fault_injection::ImuAccelBiasFault &imu_accel_bias =
		_fault_registry.imu_accel_bias();

	if (imu_accel_bias.has_definition()) {
		const fault_injection::FaultSpec &spec = imu_accel_bias.spec();

		PX4_INFO(
			"imu-accel-bias object: scenario=%" PRIu32
			" sequence=%" PRIu32
			" fault=%" PRIu16
			" state=%" PRIu8,
			spec.scenario_id,
			spec.sequence,
			spec.fault_id,
			imu_accel_bias.state_code());

		PX4_INFO(
			"imu-accel-bias vector: [%.6f, %.6f, %.6f] m/s^2",
			(double)imu_accel_bias.bias_m_s2(0),
			(double)imu_accel_bias.bias_m_s2(1),
			(double)imu_accel_bias.bias_m_s2(2));

		PX4_INFO(
			"imu-accel-bias timing: activation=%" PRIu64
			" expiration=%" PRIu64
			" applied=%" PRIu32,
			imu_accel_bias.activation_time_us(),
			imu_accel_bias.expiration_time_us(),
			imu_accel_bias.applied_count());

	} else {
		PX4_INFO("imu-accel-bias object: empty");
	}

	const fault_injection::ImuAccelNoiseFault &imu_accel_noise =
		_fault_registry.imu_accel_noise();

	if (imu_accel_noise.has_definition()) {
		const fault_injection::FaultSpec &spec = imu_accel_noise.spec();

		PX4_INFO(
			"imu-accel-noise object: scenario=%" PRIu32
			" sequence=%" PRIu32
			" fault=%" PRIu16
			" state=%" PRIu8
			" axis_mask=%" PRIu8
			" seed=%" PRIu32,
			spec.scenario_id,
			spec.sequence,
			spec.fault_id,
			imu_accel_noise.state_code(),
			spec.axis_mask,
			spec.random_seed);

		PX4_INFO(
			"imu-accel-noise sigma: [%.6f, %.6f, %.6f] m/s^2",
			(double)imu_accel_noise.sigma_m_s2(0),
			(double)imu_accel_noise.sigma_m_s2(1),
			(double)imu_accel_noise.sigma_m_s2(2));

		PX4_INFO(
			"imu-accel-noise timing: activation=%" PRIu64
			" expiration=%" PRIu64
			" applied=%" PRIu32,
			imu_accel_noise.activation_time_us(),
			imu_accel_noise.expiration_time_us(),
			imu_accel_noise.applied_count());

	} else {
		PX4_INFO("imu-accel-noise object: empty");
	}

	const fault_injection::ImuGyroNoiseFault &imu_gyro_noise =
		_fault_registry.imu_gyro_noise();

	if (imu_gyro_noise.has_definition()) {
		const fault_injection::FaultSpec &spec = imu_gyro_noise.spec();

		PX4_INFO(
			"imu-gyro-noise object: scenario=%" PRIu32
			" sequence=%" PRIu32
			" fault=%" PRIu16
			" state=%" PRIu8
			" axis_mask=%" PRIu8
			" seed=%" PRIu32,
			spec.scenario_id,
			spec.sequence,
			spec.fault_id,
			imu_gyro_noise.state_code(),
			spec.axis_mask,
			spec.random_seed);

		PX4_INFO(
			"imu-gyro-noise sigma: [%.6f, %.6f, %.6f] rad/s",
			(double)imu_gyro_noise.sigma_rad_s(0),
			(double)imu_gyro_noise.sigma_rad_s(1),
			(double)imu_gyro_noise.sigma_rad_s(2));

		PX4_INFO(
			"imu-gyro-noise timing: activation=%" PRIu64
			" expiration=%" PRIu64
			" applied=%" PRIu32,
			imu_gyro_noise.activation_time_us(),
			imu_gyro_noise.expiration_time_us(),
			imu_gyro_noise.applied_count());

	} else {
		PX4_INFO("imu-gyro-noise object: empty");
	}

	const fault_injection::ImuGyroBiasFault &imu_gyro_bias =
		_fault_registry.imu_gyro_bias();

	if (imu_gyro_bias.has_definition()) {
		const fault_injection::FaultSpec &spec = imu_gyro_bias.spec();

		PX4_INFO(
			"imu-gyro-bias object: scenario=%" PRIu32
			" sequence=%" PRIu32
			" fault=%" PRIu16
			" state=%" PRIu8
			" axis_mask=%" PRIu8,
			spec.scenario_id,
			spec.sequence,
			spec.fault_id,
			imu_gyro_bias.state_code(),
			spec.axis_mask);

		PX4_INFO(
			"imu-gyro-bias vector: [%.6f, %.6f, %.6f] rad/s",
			(double)imu_gyro_bias.bias_rad_s(0),
			(double)imu_gyro_bias.bias_rad_s(1),
			(double)imu_gyro_bias.bias_rad_s(2));

		PX4_INFO(
			"imu-gyro-bias timing: activation=%" PRIu64
			" expiration=%" PRIu64
			" applied=%" PRIu32,
			imu_gyro_bias.activation_time_us(),
			imu_gyro_bias.expiration_time_us(),
			imu_gyro_bias.applied_count());

	} else {
		PX4_INFO("imu-gyro-bias object: empty");
	}

	const fault_injection::MagBiasFault &mag_bias =
		_fault_registry.mag_bias();

	if (mag_bias.has_definition()) {
		const fault_injection::FaultSpec &spec = mag_bias.spec();

		PX4_INFO(
			"mag-bias object: scenario=%" PRIu32
			" sequence=%" PRIu32
			" fault=%" PRIu16
			" state=%" PRIu8
			" axis_mask=%" PRIu8,
			spec.scenario_id,
			spec.sequence,
			spec.fault_id,
			mag_bias.state_code(),
			spec.axis_mask);

		PX4_INFO(
			"mag-bias vector: [%.6f, %.6f, %.6f] Gauss",
			(double)mag_bias.bias_ga(0),
			(double)mag_bias.bias_ga(1),
			(double)mag_bias.bias_ga(2));

		PX4_INFO(
			"mag-bias timing: activation=%" PRIu64
			" expiration=%" PRIu64
			" applied=%" PRIu32,
			mag_bias.activation_time_us(),
			mag_bias.expiration_time_us(),
			mag_bias.applied_count());

	} else {
		PX4_INFO("mag-bias object: empty");
	}

	const fault_injection::GnssPositionBiasFault &gnss_position_bias =
		_fault_registry.gnss_position_bias();

	if (gnss_position_bias.has_definition()) {
		const fault_injection::FaultSpec &spec = gnss_position_bias.spec();
		PX4_INFO(
			"gnss-position-bias object: scenario=%" PRIu32
			" sequence=%" PRIu32 " fault=%" PRIu16
			" state=%" PRIu8 " axis_mask=%" PRIu8,
			spec.scenario_id, spec.sequence, spec.fault_id,
			gnss_position_bias.state_code(), spec.axis_mask);
		PX4_INFO("gnss-position-bias vector: [%.6f, %.6f, %.6f] m NED",
			(double)gnss_position_bias.bias_ned_m(0),
			(double)gnss_position_bias.bias_ned_m(1),
			(double)gnss_position_bias.bias_ned_m(2));
		PX4_INFO(
			"gnss-position-bias timing: activation=%" PRIu64
			" expiration=%" PRIu64 " applied=%" PRIu32,
			gnss_position_bias.activation_time_us(),
			gnss_position_bias.expiration_time_us(),
			gnss_position_bias.applied_count());
	} else {
		PX4_INFO("gnss-position-bias object: empty");
	}

	const fault_injection::GnssPositionNoiseFault &gnss_position_noise =
		_fault_registry.gnss_position_noise();

	if (gnss_position_noise.has_definition()) {
		const fault_injection::FaultSpec &spec = gnss_position_noise.spec();
		PX4_INFO(
			"gnss-position-noise object: scenario=%" PRIu32
			" sequence=%" PRIu32 " fault=%" PRIu16
			" state=%" PRIu8 " axis_mask=%" PRIu8
			" seed=%" PRIu32,
			spec.scenario_id, spec.sequence, spec.fault_id,
			gnss_position_noise.state_code(), spec.axis_mask,
			spec.random_seed);
		PX4_INFO("gnss-position-noise sigma: [%.6f, %.6f, %.6f] m NED",
			(double)gnss_position_noise.sigma_ned_m(0),
			(double)gnss_position_noise.sigma_ned_m(1),
			(double)gnss_position_noise.sigma_ned_m(2));
		PX4_INFO(
			"gnss-position-noise timing: activation=%" PRIu64
			" expiration=%" PRIu64 " applied=%" PRIu32,
			gnss_position_noise.activation_time_us(),
			gnss_position_noise.expiration_time_us(),
			gnss_position_noise.applied_count());
	} else {
		PX4_INFO("gnss-position-noise object: empty");
	}

	const fault_injection::GnssVelocityBiasFault &gnss_velocity_bias =
		_fault_registry.gnss_velocity_bias();

	if (gnss_velocity_bias.has_definition()) {
		const fault_injection::FaultSpec &spec = gnss_velocity_bias.spec();
		PX4_INFO(
			"gnss-velocity-bias object: scenario=%" PRIu32
			" sequence=%" PRIu32 " fault=%" PRIu16
			" state=%" PRIu8 " axis_mask=%" PRIu8,
			spec.scenario_id, spec.sequence, spec.fault_id,
			gnss_velocity_bias.state_code(), spec.axis_mask);
		PX4_INFO("gnss-velocity-bias vector: [%.6f, %.6f, %.6f] m/s NED",
			(double)gnss_velocity_bias.bias_ned_m_s(0),
			(double)gnss_velocity_bias.bias_ned_m_s(1),
			(double)gnss_velocity_bias.bias_ned_m_s(2));
		PX4_INFO(
			"gnss-velocity-bias timing: activation=%" PRIu64
			" expiration=%" PRIu64 " applied=%" PRIu32,
			gnss_velocity_bias.activation_time_us(),
			gnss_velocity_bias.expiration_time_us(),
			gnss_velocity_bias.applied_count());
	} else {
		PX4_INFO("gnss-velocity-bias object: empty");
	}

	const fault_injection::GnssVelocityNoiseFault &gnss_velocity_noise =
		_fault_registry.gnss_velocity_noise();

	if (gnss_velocity_noise.has_definition()) {
		const fault_injection::FaultSpec &spec = gnss_velocity_noise.spec();
		PX4_INFO(
			"gnss-velocity-noise object: scenario=%" PRIu32
			" sequence=%" PRIu32 " fault=%" PRIu16
			" state=%" PRIu8 " axis_mask=%" PRIu8
			" seed=%" PRIu32,
			spec.scenario_id, spec.sequence, spec.fault_id,
			gnss_velocity_noise.state_code(), spec.axis_mask,
			spec.random_seed);
		PX4_INFO("gnss-velocity-noise sigma: [%.6f, %.6f, %.6f] m/s NED",
			(double)gnss_velocity_noise.sigma_ned_m_s(0),
			(double)gnss_velocity_noise.sigma_ned_m_s(1),
			(double)gnss_velocity_noise.sigma_ned_m_s(2));
		PX4_INFO(
			"gnss-velocity-noise timing: activation=%" PRIu64
			" expiration=%" PRIu64 " applied=%" PRIu32,
			gnss_velocity_noise.activation_time_us(),
			gnss_velocity_noise.expiration_time_us(),
			gnss_velocity_noise.applied_count());
	} else {
		PX4_INFO("gnss-velocity-noise object: empty");
	}

	const fault_injection::DownwardRangeBiasFault &downward_range_bias =
		_fault_registry.downward_range_bias();

	if (downward_range_bias.has_definition()) {
		const fault_injection::FaultSpec &spec = downward_range_bias.spec();
		PX4_INFO(
			"downward-range-bias object: scenario=%" PRIu32
			" sequence=%" PRIu32 " fault=%" PRIu16
			" state=%" PRIu8 " bias=%.6f m applied=%" PRIu32,
			spec.scenario_id, spec.sequence, spec.fault_id,
			downward_range_bias.state_code(),
			(double)downward_range_bias.bias_m(),
			downward_range_bias.applied_count());
	} else {
		PX4_INFO("downward-range-bias object: empty");
	}

	const fault_injection::DownwardRangeNoiseFault &downward_range_noise =
		_fault_registry.downward_range_noise();

	if (downward_range_noise.has_definition()) {
		const fault_injection::FaultSpec &spec = downward_range_noise.spec();
		PX4_INFO(
			"downward-range-noise object: scenario=%" PRIu32
			" sequence=%" PRIu32 " fault=%" PRIu16
			" state=%" PRIu8 " seed=%" PRIu32
			" sigma=%.6f m applied=%" PRIu32,
			spec.scenario_id, spec.sequence, spec.fault_id,
			downward_range_noise.state_code(), spec.random_seed,
			(double)downward_range_noise.sigma_m(),
			downward_range_noise.applied_count());
	} else {
		PX4_INFO("downward-range-noise object: empty");
	}


	const fault_injection::DownwardRangeDegradationFault &downward_range_degradation =
		_fault_registry.downward_range_degradation();

	if (downward_range_degradation.has_definition()) {
		const fault_injection::FaultSpec &spec = downward_range_degradation.spec();
		PX4_INFO(
			"downward-range-degradation object: scenario=%" PRIu32
			" sequence=%" PRIu32 " fault=%" PRIu16
			" state=%" PRIu8 " effective_max=%.6f m applied=%" PRIu32,
			spec.scenario_id, spec.sequence, spec.fault_id,
			downward_range_degradation.state_code(),
			(double)downward_range_degradation.effective_max_distance_m(),
			downward_range_degradation.applied_count());
	} else {
		PX4_INFO("downward-range-degradation object: empty");
	}

	const fault_injection::BaroAltitudeBiasFault &baro_altitude_bias =
		_fault_registry.baro_altitude_bias();
	if (baro_altitude_bias.has_definition()) {
		const fault_injection::FaultSpec &spec = baro_altitude_bias.spec();
		PX4_INFO(
			"baro-altitude-bias object: scenario=%" PRIu32
			" sequence=%" PRIu32 " fault=%" PRIu16
			" state=%" PRIu8 " bias=%.6f m applied=%" PRIu32,
			spec.scenario_id, spec.sequence, spec.fault_id,
			baro_altitude_bias.state_code(),
			(double)baro_altitude_bias.bias_m(),
			baro_altitude_bias.applied_count());
	} else {
		PX4_INFO("baro-altitude-bias object: empty");
	}

	const fault_injection::BaroAltitudeNoiseFault &baro_altitude_noise =
		_fault_registry.baro_altitude_noise();
	if (baro_altitude_noise.has_definition()) {
		const fault_injection::FaultSpec &spec = baro_altitude_noise.spec();
		PX4_INFO(
			"baro-altitude-noise object: scenario=%" PRIu32
			" sequence=%" PRIu32 " fault=%" PRIu16
			" state=%" PRIu8 " seed=%" PRIu32
			" sigma=%.6f m applied=%" PRIu32,
			spec.scenario_id, spec.sequence, spec.fault_id,
			baro_altitude_noise.state_code(), spec.random_seed,
			(double)baro_altitude_noise.sigma_m(),
			baro_altitude_noise.applied_count());
	} else {
		PX4_INFO("baro-altitude-noise object: empty");
	}

	const fault_injection::MotorOutputBiasFault &motor_output_bias =
		_fault_registry.motor_output_bias();
	if (motor_output_bias.has_definition()) {
		const fault_injection::FaultSpec &spec = motor_output_bias.spec();
		PX4_INFO(
			"motor-output-bias object: scenario=%" PRIu32
			" sequence=%" PRIu32 " fault=%" PRIu16
			" state=%" PRIu8 " mask=0x%02" PRIx8
			" bias=%.6f applied=%" PRIu32,
			spec.scenario_id, spec.sequence, spec.fault_id,
			motor_output_bias.state_code(), spec.axis_mask,
			(double)motor_output_bias.bias_norm(),
			motor_output_bias.applied_count());
	} else {
		PX4_INFO("motor-output-bias object: empty");
	}

	const fault_injection::MotorOutputDelayFault &motor_output_delay =
		_fault_registry.motor_output_delay();
	if (motor_output_delay.has_definition()) {
		const fault_injection::FaultSpec &spec = motor_output_delay.spec();
		PX4_INFO(
			"motor-output-delay object: scenario=%" PRIu32
			" sequence=%" PRIu32 " fault=%" PRIu16
			" state=%" PRIu8 " mask=0x%02" PRIx8
			" delay_us=%" PRIu32 " applied=%" PRIu32,
			spec.scenario_id, spec.sequence, spec.fault_id,
			motor_output_delay.state_code(), spec.axis_mask,
			motor_output_delay.delay_us(),
			motor_output_delay.applied_count());
	} else {
		PX4_INFO("motor-output-delay object: empty");
	}

	const fault_injection::MotorMaxOutputDegradationFault &motor_max_output =
		_fault_registry.motor_max_output_degradation();
	if (motor_max_output.has_definition()) {
		const fault_injection::FaultSpec &spec = motor_max_output.spec();
		PX4_INFO(
			"motor-max-output object: scenario=%" PRIu32
			" sequence=%" PRIu32 " fault=%" PRIu16
			" state=%" PRIu8 " mask=0x%02" PRIx8
			" cap=%.6f applied=%" PRIu32,
			spec.scenario_id, spec.sequence, spec.fault_id,
			motor_max_output.state_code(), spec.axis_mask,
			(double)motor_max_output.max_output_norm(),
			motor_max_output.applied_count());
	} else {
		PX4_INFO("motor-max-output object: empty");
	}

	return 0;
}

int FaultManager::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int FaultManager::print_usage(const char *reason)
{
	if (reason != nullptr) {
		PX4_WARN("%s", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
Preallocated FAULT fault-object control and lifecycle infrastructure.

The registry contains preallocated IMU, magnetometer, GNSS, downward-range,
barometer and motor-output concrete fault objects. GNSS position bias/noise are expressed in local NED metres and GNSS
velocity bias/noise in local NED metres per second. They modify only the
corresponding position or velocity fields of the gnssSample consumed by EKF2.
Receiver health, availability, validity and accuracy metadata remain unchanged.
Downward-range bias/noise use the scalar command parameter in metres. Range
degradation uses scalar as the effective maximum detection distance. They modify
only the selected downward-facing range supplied to EKF2. Barometric-altitude
bias/noise use scalar metres and transform only vehicle_air_data.baro_alt_meter
immediately before EKF2 setBaroData; density and reset metadata are preserved.
Motor faults transform selected actuator_motors commands after allocation and
before the common SITL/real output conversion. Their order is delay, bias, then
maximum-output degradation. Current NaN/stop commands are never revived.

Fault identities are unique by scenario_id + fault_id across concrete object
types. No runtime fault-object allocation or deletion occurs.
)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("fault_manager", "system");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

extern "C" __EXPORT int fault_manager_main(
	int argc,
	char *argv[])
{
	return FaultManager::main(argc, argv);
}
