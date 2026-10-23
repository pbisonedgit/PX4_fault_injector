#include "FaultRegistry.hpp"

#include <uORB/topics/fault_command.h>

namespace fault_injection
{

FaultRegistry::FaultRegistry() :
	_faults{&_no_effect, &_imu_accel_bias, &_imu_accel_noise, &_imu_gyro_bias, &_imu_gyro_noise, &_mag_bias, &_mag_noise, &_gnss_position_bias, &_gnss_position_noise, &_gnss_velocity_bias, &_gnss_velocity_noise, &_downward_range_bias, &_downward_range_noise, &_downward_range_degradation, &_baro_altitude_bias, &_baro_altitude_noise, &_motor_output_bias, &_motor_output_delay, &_motor_max_output_degradation}
{
}

FaultInstance *FaultRegistry::find(uint8_t target, uint8_t mode)
{
	if ((target == fault_command_s::TARGET_NONE)
	    && (mode == fault_command_s::MODE_NO_EFFECT)) {
		return &_no_effect;
	}

	if ((target == fault_command_s::TARGET_IMU)
	    && (mode == fault_command_s::MODE_ACCEL_BIAS)) {
		return &_imu_accel_bias;
	}

	if ((target == fault_command_s::TARGET_IMU)
	    && (mode == fault_command_s::MODE_ACCEL_NOISE)) {
		return &_imu_accel_noise;
	}

	if ((target == fault_command_s::TARGET_IMU)
	    && (mode == fault_command_s::MODE_GYRO_BIAS)) {
		return &_imu_gyro_bias;
	}

	if ((target == fault_command_s::TARGET_IMU)
	    && (mode == fault_command_s::MODE_GYRO_NOISE)) {
		return &_imu_gyro_noise;
	}

	if ((target == fault_command_s::TARGET_MAG)
	    && (mode == fault_command_s::MODE_MAG_BIAS)) {
		return &_mag_bias;
	}
	if ((target == fault_command_s::TARGET_MAG)
	    && (mode == fault_command_s::MODE_MAG_NOISE)) {
		return &_mag_noise;
	}

	if ((target == fault_command_s::TARGET_GNSS)
	    && (mode == fault_command_s::MODE_GNSS_POSITION_BIAS)) {
		return &_gnss_position_bias;
	}

	if ((target == fault_command_s::TARGET_GNSS)
	    && (mode == fault_command_s::MODE_GNSS_POSITION_NOISE)) {
		return &_gnss_position_noise;
	}

	if ((target == fault_command_s::TARGET_GNSS)
	    && (mode == fault_command_s::MODE_GNSS_VELOCITY_BIAS)) {
		return &_gnss_velocity_bias;
	}

	if ((target == fault_command_s::TARGET_GNSS)
	    && (mode == fault_command_s::MODE_GNSS_VELOCITY_NOISE)) {
		return &_gnss_velocity_noise;
	}

	if ((target == fault_command_s::TARGET_DOWNWARD_RANGE)
	    && (mode == fault_command_s::MODE_DOWNWARD_RANGE_BIAS)) {
		return &_downward_range_bias;
	}

	if ((target == fault_command_s::TARGET_DOWNWARD_RANGE)
	    && (mode == fault_command_s::MODE_DOWNWARD_RANGE_NOISE)) {
		return &_downward_range_noise;
	}

	if ((target == fault_command_s::TARGET_DOWNWARD_RANGE)
	    && (mode == fault_command_s::MODE_DOWNWARD_RANGE_DEGRADATION)) {
		return &_downward_range_degradation;
	}

	if ((target == fault_command_s::TARGET_BAROMETER)
	    && (mode == fault_command_s::MODE_BARO_ALTITUDE_BIAS)) {
		return &_baro_altitude_bias;
	}

	if ((target == fault_command_s::TARGET_BAROMETER)
	    && (mode == fault_command_s::MODE_BARO_ALTITUDE_NOISE)) {
		return &_baro_altitude_noise;
	}

	if ((target == fault_command_s::TARGET_MOTOR)
	    && (mode == fault_command_s::MODE_MOTOR_OUTPUT_BIAS)) {
		return &_motor_output_bias;
	}

	if ((target == fault_command_s::TARGET_MOTOR)
	    && (mode == fault_command_s::MODE_MOTOR_OUTPUT_DELAY)) {
		return &_motor_output_delay;
	}

	if ((target == fault_command_s::TARGET_MOTOR)
	    && (mode == fault_command_s::MODE_MOTOR_MAX_OUTPUT_DEGRADATION)) {
		return &_motor_max_output_degradation;
	}

	return nullptr;
}

const FaultInstance *FaultRegistry::find(
	uint8_t target,
	uint8_t mode) const
{
	if ((target == fault_command_s::TARGET_NONE)
	    && (mode == fault_command_s::MODE_NO_EFFECT)) {
		return &_no_effect;
	}

	if ((target == fault_command_s::TARGET_IMU)
	    && (mode == fault_command_s::MODE_ACCEL_BIAS)) {
		return &_imu_accel_bias;
	}

	if ((target == fault_command_s::TARGET_IMU)
	    && (mode == fault_command_s::MODE_ACCEL_NOISE)) {
		return &_imu_accel_noise;
	}

	if ((target == fault_command_s::TARGET_IMU)
	    && (mode == fault_command_s::MODE_GYRO_BIAS)) {
		return &_imu_gyro_bias;
	}

	if ((target == fault_command_s::TARGET_IMU)
	    && (mode == fault_command_s::MODE_GYRO_NOISE)) {
		return &_imu_gyro_noise;
	}

	if ((target == fault_command_s::TARGET_MAG)
	    && (mode == fault_command_s::MODE_MAG_BIAS)) {
		return &_mag_bias;
	}
	if ((target == fault_command_s::TARGET_MAG)
	    && (mode == fault_command_s::MODE_MAG_NOISE)) {
		return &_mag_noise;
	}

	if ((target == fault_command_s::TARGET_GNSS)
	    && (mode == fault_command_s::MODE_GNSS_POSITION_BIAS)) {
		return &_gnss_position_bias;
	}

	if ((target == fault_command_s::TARGET_GNSS)
	    && (mode == fault_command_s::MODE_GNSS_POSITION_NOISE)) {
		return &_gnss_position_noise;
	}

	if ((target == fault_command_s::TARGET_GNSS)
	    && (mode == fault_command_s::MODE_GNSS_VELOCITY_BIAS)) {
		return &_gnss_velocity_bias;
	}

	if ((target == fault_command_s::TARGET_GNSS)
	    && (mode == fault_command_s::MODE_GNSS_VELOCITY_NOISE)) {
		return &_gnss_velocity_noise;
	}

	if ((target == fault_command_s::TARGET_DOWNWARD_RANGE)
	    && (mode == fault_command_s::MODE_DOWNWARD_RANGE_BIAS)) {
		return &_downward_range_bias;
	}

	if ((target == fault_command_s::TARGET_DOWNWARD_RANGE)
	    && (mode == fault_command_s::MODE_DOWNWARD_RANGE_NOISE)) {
		return &_downward_range_noise;
	}

	if ((target == fault_command_s::TARGET_DOWNWARD_RANGE)
	    && (mode == fault_command_s::MODE_DOWNWARD_RANGE_DEGRADATION)) {
		return &_downward_range_degradation;
	}

	if ((target == fault_command_s::TARGET_BAROMETER)
	    && (mode == fault_command_s::MODE_BARO_ALTITUDE_BIAS)) {
		return &_baro_altitude_bias;
	}

	if ((target == fault_command_s::TARGET_BAROMETER)
	    && (mode == fault_command_s::MODE_BARO_ALTITUDE_NOISE)) {
		return &_baro_altitude_noise;
	}

	if ((target == fault_command_s::TARGET_MOTOR)
	    && (mode == fault_command_s::MODE_MOTOR_OUTPUT_BIAS)) {
		return &_motor_output_bias;
	}

	if ((target == fault_command_s::TARGET_MOTOR)
	    && (mode == fault_command_s::MODE_MOTOR_OUTPUT_DELAY)) {
		return &_motor_output_delay;
	}

	if ((target == fault_command_s::TARGET_MOTOR)
	    && (mode == fault_command_s::MODE_MOTOR_MAX_OUTPUT_DEGRADATION)) {
		return &_motor_max_output_degradation;
	}

	return nullptr;
}

bool FaultRegistry::supports_target(uint8_t target) const
{
	return (target == fault_command_s::TARGET_NONE)
	       || (target == fault_command_s::TARGET_IMU)
	       || (target == fault_command_s::TARGET_MAG)
	       || (target == fault_command_s::TARGET_GNSS)
	       || (target == fault_command_s::TARGET_DOWNWARD_RANGE)
	       || (target == fault_command_s::TARGET_BAROMETER)
	       || (target == fault_command_s::TARGET_MOTOR);
}

FaultInstance *FaultRegistry::find_by_identity(
	uint32_t scenario_id,
	uint16_t fault_id)
{
	for (FaultInstance *fault : _faults) {
		if ((fault != nullptr)
		    && fault->matches(scenario_id, fault_id)) {
			return fault;
		}
	}

	return nullptr;
}

const FaultInstance *FaultRegistry::find_by_identity(
	uint32_t scenario_id,
	uint16_t fault_id) const
{
	for (const FaultInstance *fault : _faults) {
		if ((fault != nullptr)
		    && fault->matches(scenario_id, fault_id)) {
			return fault;
		}
	}

	return nullptr;
}

std::size_t FaultRegistry::update(
	uint64_t now_us,
	FaultInstance *changed_faults[],
	std::size_t capacity)
{
	std::size_t changed_count = 0;

	for (FaultInstance *fault : _faults) {
		if ((fault != nullptr) && fault->update(now_us)) {
			if ((changed_faults != nullptr)
			    && (changed_count < capacity)) {
				changed_faults[changed_count] = fault;
			}

			++changed_count;
		}
	}

	return changed_count;
}

std::size_t FaultRegistry::clear_all(
	uint64_t now_us,
	FaultInstance *cleared_faults[],
	std::size_t capacity)
{
	std::size_t cleared_count = 0;

	for (FaultInstance *fault : _faults) {
		if ((fault != nullptr) && fault->clear(now_us)) {
			if ((cleared_faults != nullptr)
			    && (cleared_count < capacity)) {
				cleared_faults[cleared_count] = fault;
			}

			++cleared_count;
		}
	}

	return cleared_count;
}

} // namespace fault_injection
