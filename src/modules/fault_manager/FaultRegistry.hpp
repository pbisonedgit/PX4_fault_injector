#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

#include "FaultInstance.hpp"
#include "ImuAccelBiasFault.hpp"
#include "ImuAccelNoiseFault.hpp"
#include "ImuGyroBiasFault.hpp"
#include "ImuGyroNoiseFault.hpp"
#include "MagBiasFault.hpp"
#include "MagNoiseFault.hpp"
#include "GnssPositionBiasFault.hpp"
#include "GnssPositionNoiseFault.hpp"
#include "GnssVelocityBiasFault.hpp"
#include "GnssVelocityNoiseFault.hpp"
#include "DownwardRangeBiasFault.hpp"
#include "DownwardRangeNoiseFault.hpp"
#include "DownwardRangeDegradationFault.hpp"
#include "BaroAltitudeBiasFault.hpp"
#include "BaroAltitudeNoiseFault.hpp"
#include "MotorOutputBiasFault.hpp"
#include "MotorOutputDelayFault.hpp"
#include "MotorMaxOutputDegradationFault.hpp"
#include "NoEffectFault.hpp"

namespace fault_injection
{

class FaultRegistry
{
public:
	static constexpr std::size_t FaultCount = 19;

	FaultRegistry();
	~FaultRegistry() = default;

	FaultRegistry(const FaultRegistry &) = delete;
	FaultRegistry &operator=(const FaultRegistry &) = delete;
	FaultRegistry(FaultRegistry &&) = delete;
	FaultRegistry &operator=(FaultRegistry &&) = delete;

	FaultInstance *find(uint8_t target, uint8_t mode);
	const FaultInstance *find(uint8_t target, uint8_t mode) const;

	bool supports_target(uint8_t target) const;

	FaultInstance *find_by_identity(
		uint32_t scenario_id,
		uint16_t fault_id);

	const FaultInstance *find_by_identity(
		uint32_t scenario_id,
		uint16_t fault_id) const;

	std::size_t update(
		uint64_t now_us,
		FaultInstance *changed_faults[],
		std::size_t capacity);

	std::size_t clear_all(
		uint64_t now_us,
		FaultInstance *cleared_faults[],
		std::size_t capacity);

	NoEffectFault &no_effect()
	{
		return _no_effect;
	}

	const NoEffectFault &no_effect() const
	{
		return _no_effect;
	}

	ImuAccelBiasFault &imu_accel_bias()
	{
		return _imu_accel_bias;
	}

	const ImuAccelBiasFault &imu_accel_bias() const
	{
		return _imu_accel_bias;
	}

	ImuAccelNoiseFault &imu_accel_noise()
	{
		return _imu_accel_noise;
	}

	const ImuAccelNoiseFault &imu_accel_noise() const
	{
		return _imu_accel_noise;
	}

	ImuGyroBiasFault &imu_gyro_bias()
	{
		return _imu_gyro_bias;
	}

	const ImuGyroBiasFault &imu_gyro_bias() const
	{
		return _imu_gyro_bias;
	}

	ImuGyroNoiseFault &imu_gyro_noise()
	{
		return _imu_gyro_noise;
	}

	const ImuGyroNoiseFault &imu_gyro_noise() const
	{
		return _imu_gyro_noise;
	}

	MagBiasFault &mag_bias()
	{
		return _mag_bias;
	}

	const MagBiasFault &mag_bias() const
	{
		return _mag_bias;
	}

	MagNoiseFault &mag_noise() { return _mag_noise; }
	const MagNoiseFault &mag_noise() const { return _mag_noise; }

	GnssPositionBiasFault &gnss_position_bias()
	{
		return _gnss_position_bias;
	}

	const GnssPositionBiasFault &gnss_position_bias() const
	{
		return _gnss_position_bias;
	}

	GnssPositionNoiseFault &gnss_position_noise()
	{
		return _gnss_position_noise;
	}

	const GnssPositionNoiseFault &gnss_position_noise() const
	{
		return _gnss_position_noise;
	}

	GnssVelocityBiasFault &gnss_velocity_bias()
	{
		return _gnss_velocity_bias;
	}

	const GnssVelocityBiasFault &gnss_velocity_bias() const
	{
		return _gnss_velocity_bias;
	}

	GnssVelocityNoiseFault &gnss_velocity_noise()
	{
		return _gnss_velocity_noise;
	}

	const GnssVelocityNoiseFault &gnss_velocity_noise() const
	{
		return _gnss_velocity_noise;
	}

	DownwardRangeBiasFault &downward_range_bias() { return _downward_range_bias; }
	const DownwardRangeBiasFault &downward_range_bias() const { return _downward_range_bias; }
	DownwardRangeNoiseFault &downward_range_noise() { return _downward_range_noise; }
	const DownwardRangeNoiseFault &downward_range_noise() const { return _downward_range_noise; }
	DownwardRangeDegradationFault &downward_range_degradation() { return _downward_range_degradation; }
	const DownwardRangeDegradationFault &downward_range_degradation() const { return _downward_range_degradation; }
	BaroAltitudeBiasFault &baro_altitude_bias() { return _baro_altitude_bias; }
	const BaroAltitudeBiasFault &baro_altitude_bias() const { return _baro_altitude_bias; }
	BaroAltitudeNoiseFault &baro_altitude_noise() { return _baro_altitude_noise; }
	const BaroAltitudeNoiseFault &baro_altitude_noise() const { return _baro_altitude_noise; }
	MotorOutputBiasFault &motor_output_bias() { return _motor_output_bias; }
	const MotorOutputBiasFault &motor_output_bias() const { return _motor_output_bias; }
	MotorOutputDelayFault &motor_output_delay() { return _motor_output_delay; }
	const MotorOutputDelayFault &motor_output_delay() const { return _motor_output_delay; }
	MotorMaxOutputDegradationFault &motor_max_output_degradation()
	{
		return _motor_max_output_degradation;
	}
	const MotorMaxOutputDegradationFault &motor_max_output_degradation() const
	{
		return _motor_max_output_degradation;
	}

private:
	/*
	 * Every concrete fault object is a registry member. Runtime commands only
	 * reconfigure these existing objects; they do not allocate or delete fault
	 * instances.
	 */
	NoEffectFault _no_effect{};
	ImuAccelBiasFault _imu_accel_bias{};
	ImuAccelNoiseFault _imu_accel_noise{};
	ImuGyroBiasFault _imu_gyro_bias{};
	ImuGyroNoiseFault _imu_gyro_noise{};
	MagBiasFault _mag_bias{};
	MagNoiseFault _mag_noise{};
	GnssPositionBiasFault _gnss_position_bias{};
	GnssPositionNoiseFault _gnss_position_noise{};
	GnssVelocityBiasFault _gnss_velocity_bias{};
	GnssVelocityNoiseFault _gnss_velocity_noise{};
	DownwardRangeBiasFault _downward_range_bias{};
	DownwardRangeNoiseFault _downward_range_noise{};
	DownwardRangeDegradationFault _downward_range_degradation{};
	BaroAltitudeBiasFault _baro_altitude_bias{};
	BaroAltitudeNoiseFault _baro_altitude_noise{};
	MotorOutputBiasFault _motor_output_bias{};
	MotorOutputDelayFault _motor_output_delay{};
	MotorMaxOutputDegradationFault _motor_max_output_degradation{};

	std::array<FaultInstance *, FaultCount> _faults{};
};

} // namespace fault_injection
