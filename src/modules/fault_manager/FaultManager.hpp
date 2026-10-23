#pragma once

#include "FaultRegistry.hpp"

#include <lib/fault_injection/FixedArray.hpp>
#include <cstddef>
#include <cstdint>

#include <lib/fault_injection/CumulativeFeedbackAggregator.hpp>
#include <px4_platform_common/module.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <px4_platform_common/time.h>

#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionMultiArray.hpp>
#include <uORB/topics/fault_ack.h>
#include <uORB/topics/fault_command.h>
#include <uORB/topics/fault_status.h>
#include <uORB/topics/fault_imu_accel_bias.h>
#include <uORB/topics/fault_imu_accel_bias_applied.h>
#include <uORB/topics/fault_imu_accel_noise.h>
#include <uORB/topics/fault_imu_accel_noise_applied.h>
#include <uORB/topics/fault_imu_gyro_bias.h>
#include <uORB/topics/fault_imu_gyro_bias_applied.h>
#include <uORB/topics/fault_imu_gyro_noise.h>
#include <uORB/topics/fault_imu_gyro_noise_applied.h>
#include <uORB/topics/fault_mag_bias.h>
#include <uORB/topics/fault_mag_bias_applied.h>
#include <uORB/topics/fault_mag_noise.h>
#include <uORB/topics/fault_mag_noise_applied.h>
#include <uORB/topics/fault_gnss_position_bias.h>
#include <uORB/topics/fault_gnss_position_bias_applied.h>
#include <uORB/topics/fault_gnss_position_noise.h>
#include <uORB/topics/fault_gnss_position_noise_applied.h>
#include <uORB/topics/fault_gnss_velocity_bias.h>
#include <uORB/topics/fault_gnss_velocity_bias_applied.h>
#include <uORB/topics/fault_gnss_velocity_noise.h>
#include <uORB/topics/fault_gnss_velocity_noise_applied.h>
#include <uORB/topics/fault_downward_range_noise_applied.h>
#include <uORB/topics/fault_downward_range_noise.h>
#include <uORB/topics/fault_downward_range_bias_applied.h>
#include <uORB/topics/fault_downward_range_bias.h>
#include <uORB/topics/fault_downward_range_degradation.h>
#include <uORB/topics/fault_downward_range_degradation_applied.h>
#include <uORB/topics/fault_baro_altitude_bias.h>
#include <uORB/topics/fault_baro_altitude_bias_applied.h>
#include <uORB/topics/fault_baro_altitude_noise.h>
#include <uORB/topics/fault_baro_altitude_noise_applied.h>
#include <uORB/topics/fault_motor_output_bias.h>
#include <uORB/topics/fault_motor_output_bias_applied.h>
#include <uORB/topics/fault_motor_output_delay.h>
#include <uORB/topics/fault_motor_output_delay_applied.h>
#include <uORB/topics/fault_motor_max_output_degradation.h>
#include <uORB/topics/fault_motor_max_output_degradation_applied.h>

using namespace time_literals;

class FaultManager final :
	public ModuleBase<FaultManager>,
	public px4::ScheduledWorkItem
{
public:
	FaultManager();
	~FaultManager() override = default;

	static int task_spawn(int argc, char *argv[]);
	static int custom_command(int argc, char *argv[]);
	static int print_usage(const char *reason = nullptr);

	bool init();
	int print_status() override;

private:
	void Run() override;
	void process_command(const fault_command_s &command);

	void process_imu_accel_bias_applied();
	void process_imu_accel_bias_applied_report(
		std::size_t subscription_index,
		const fault_imu_accel_bias_applied_s &report);

	void process_imu_accel_noise_applied();
	void process_imu_accel_noise_applied_report(
		std::size_t subscription_index,
		const fault_imu_accel_noise_applied_s &report);
	void process_imu_gyro_noise_applied();
	void process_imu_gyro_noise_applied_report(
		std::size_t subscription_index,
		const fault_imu_gyro_noise_applied_s &report);

	void process_imu_gyro_bias_applied();
	void process_imu_gyro_bias_applied_report(
		std::size_t subscription_index,
		const fault_imu_gyro_bias_applied_s &report);
	void process_mag_bias_applied();
	void process_mag_bias_applied_report(
		std::size_t subscription_index,
		const fault_mag_bias_applied_s &report);
	void process_mag_noise_applied();
	void process_mag_noise_applied_report(
		std::size_t subscription_index,
		const fault_mag_noise_applied_s &report);

	void process_gnss_position_bias_applied();
	void process_gnss_position_bias_applied_report(
		std::size_t subscription_index,
		const fault_gnss_position_bias_applied_s &report);
	void process_gnss_position_noise_applied();
	void process_gnss_position_noise_applied_report(
		std::size_t subscription_index,
		const fault_gnss_position_noise_applied_s &report);
	void process_gnss_velocity_bias_applied();
	void process_gnss_velocity_bias_applied_report(
		std::size_t subscription_index,
		const fault_gnss_velocity_bias_applied_s &report);
	void process_gnss_velocity_noise_applied();
	void process_gnss_velocity_noise_applied_report(
		std::size_t subscription_index,
		const fault_gnss_velocity_noise_applied_s &report);
	void process_downward_range_bias_applied();
	void process_downward_range_bias_applied_report(
		std::size_t subscription_index,
		const fault_downward_range_bias_applied_s &report);
	void process_downward_range_noise_applied();
	void process_downward_range_noise_applied_report(
		std::size_t subscription_index,
		const fault_downward_range_noise_applied_s &report);
	void process_downward_range_degradation_applied();
	void process_downward_range_degradation_applied_report(
		std::size_t subscription_index,
		const fault_downward_range_degradation_applied_s &report);
	void process_baro_altitude_bias_applied();
	void process_baro_altitude_bias_applied_report(
		std::size_t subscription_index,
		const fault_baro_altitude_bias_applied_s &report);
	void process_baro_altitude_noise_applied();
	void process_baro_altitude_noise_applied_report(
		std::size_t subscription_index,
		const fault_baro_altitude_noise_applied_s &report);
	void process_motor_output_bias_applied();
	void process_motor_output_bias_applied_report(
		std::size_t subscription_index,
		const fault_motor_output_bias_applied_s &report);
	void process_motor_output_delay_applied();
	void process_motor_output_delay_applied_report(
		std::size_t subscription_index,
		const fault_motor_output_delay_applied_s &report);
	void process_motor_max_output_degradation_applied();
	void process_motor_max_output_degradation_applied_report(
		std::size_t subscription_index,
		const fault_motor_max_output_degradation_applied_s &report);

	void publish_ack(const fault_command_s &command,
			 uint8_t result,
			 uint8_t reason,
			 uint8_t state);

	void publish_status(const fault_injection::FaultInstance &fault,
			    uint8_t target_system,
			    uint8_t target_component,
			    uint32_t sequence);

	void publish_lifecycle_status(
		const fault_injection::FaultInstance &fault);

	void publish_runtime_effect(
		const fault_injection::FaultInstance &fault,
		uint32_t sequence);

	void publish_imu_accel_bias_effect(
		const fault_injection::ImuAccelBiasFault &fault,
		uint32_t sequence);

	void publish_imu_accel_noise_effect(
		const fault_injection::ImuAccelNoiseFault &fault,
		uint32_t sequence);
	void publish_imu_gyro_noise_effect(
		const fault_injection::ImuGyroNoiseFault &fault,
		uint32_t sequence);

	void publish_imu_gyro_bias_effect(
		const fault_injection::ImuGyroBiasFault &fault,
		uint32_t sequence);
	void publish_mag_bias_effect(
		const fault_injection::MagBiasFault &fault,
		uint32_t sequence);
	void publish_mag_noise_effect(
		const fault_injection::MagNoiseFault &fault,
		uint32_t sequence);
	void publish_gnss_position_bias_effect(
		const fault_injection::GnssPositionBiasFault &fault,
		uint32_t sequence);
	void publish_gnss_position_noise_effect(
		const fault_injection::GnssPositionNoiseFault &fault,
		uint32_t sequence);
	void publish_gnss_velocity_bias_effect(
		const fault_injection::GnssVelocityBiasFault &fault,
		uint32_t sequence);
	void publish_gnss_velocity_noise_effect(
		const fault_injection::GnssVelocityNoiseFault &fault,
		uint32_t sequence);
	void publish_downward_range_bias_effect(
		const fault_injection::DownwardRangeBiasFault &fault, uint32_t sequence);
	void publish_downward_range_noise_effect(
		const fault_injection::DownwardRangeNoiseFault &fault, uint32_t sequence);
	void publish_downward_range_degradation_effect(
		const fault_injection::DownwardRangeDegradationFault &fault, uint32_t sequence);
	void publish_baro_altitude_bias_effect(
		const fault_injection::BaroAltitudeBiasFault &fault, uint32_t sequence);
	void publish_baro_altitude_noise_effect(
		const fault_injection::BaroAltitudeNoiseFault &fault, uint32_t sequence);
	void publish_motor_output_bias_effect(
		const fault_injection::MotorOutputBiasFault &fault, uint32_t sequence);
	void publish_motor_output_delay_effect(
		const fault_injection::MotorOutputDelayFault &fault, uint32_t sequence);
	void publish_motor_max_output_degradation_effect(
		const fault_injection::MotorMaxOutputDegradationFault &fault, uint32_t sequence);

	using ImuAccelBiasAppliedBaseline =
		fault_injection::CumulativeFeedbackBaseline;

	using ImuAccelBiasAppliedSubscriptions =
		uORB::SubscriptionMultiArray<
			fault_imu_accel_bias_applied_s>;

	static constexpr std::size_t
		ImuAccelBiasAppliedSubscriptionCount =
			ImuAccelBiasAppliedSubscriptions::size();

	using ImuAccelNoiseAppliedBaseline =
		fault_injection::CumulativeFeedbackBaseline;

	using ImuAccelNoiseAppliedSubscriptions =
		uORB::SubscriptionMultiArray<
			fault_imu_accel_noise_applied_s>;

	static constexpr std::size_t
		ImuAccelNoiseAppliedSubscriptionCount =
			ImuAccelNoiseAppliedSubscriptions::size();
	using ImuGyroNoiseAppliedBaseline =
		fault_injection::CumulativeFeedbackBaseline;

	using ImuGyroNoiseAppliedSubscriptions =
		uORB::SubscriptionMultiArray<
			fault_imu_gyro_noise_applied_s>;

	static constexpr std::size_t
		ImuGyroNoiseAppliedSubscriptionCount =
			ImuGyroNoiseAppliedSubscriptions::size();

	using ImuGyroBiasAppliedBaseline =
		fault_injection::CumulativeFeedbackBaseline;

	using ImuGyroBiasAppliedSubscriptions =
		uORB::SubscriptionMultiArray<
			fault_imu_gyro_bias_applied_s>;

	static constexpr std::size_t
		ImuGyroBiasAppliedSubscriptionCount =
			ImuGyroBiasAppliedSubscriptions::size();
	using MagBiasAppliedBaseline =
		fault_injection::CumulativeFeedbackBaseline;

	using MagBiasAppliedSubscriptions =
		uORB::SubscriptionMultiArray<
			fault_mag_bias_applied_s>;

	static constexpr std::size_t
		MagBiasAppliedSubscriptionCount =
			MagBiasAppliedSubscriptions::size();
	using MagNoiseAppliedBaseline = fault_injection::CumulativeFeedbackBaseline;
	using MagNoiseAppliedSubscriptions =
		uORB::SubscriptionMultiArray<fault_mag_noise_applied_s>;
	static constexpr std::size_t MagNoiseAppliedSubscriptionCount =
		MagNoiseAppliedSubscriptions::size();

	using GnssPositionBiasAppliedBaseline =
		fault_injection::CumulativeFeedbackBaseline;
	using GnssPositionBiasAppliedSubscriptions =
		uORB::SubscriptionMultiArray<
			fault_gnss_position_bias_applied_s>;
	static constexpr std::size_t
		GnssPositionBiasAppliedSubscriptionCount =
			GnssPositionBiasAppliedSubscriptions::size();

	using GnssPositionNoiseAppliedBaseline =
		fault_injection::CumulativeFeedbackBaseline;
	using GnssPositionNoiseAppliedSubscriptions =
		uORB::SubscriptionMultiArray<
			fault_gnss_position_noise_applied_s>;
	static constexpr std::size_t
		GnssPositionNoiseAppliedSubscriptionCount =
			GnssPositionNoiseAppliedSubscriptions::size();

	using GnssVelocityBiasAppliedBaseline =
		fault_injection::CumulativeFeedbackBaseline;
	using GnssVelocityBiasAppliedSubscriptions =
		uORB::SubscriptionMultiArray<
			fault_gnss_velocity_bias_applied_s>;
	static constexpr std::size_t
		GnssVelocityBiasAppliedSubscriptionCount =
			GnssVelocityBiasAppliedSubscriptions::size();

	using GnssVelocityNoiseAppliedBaseline =
		fault_injection::CumulativeFeedbackBaseline;
	using GnssVelocityNoiseAppliedSubscriptions =
		uORB::SubscriptionMultiArray<
			fault_gnss_velocity_noise_applied_s>;
	static constexpr std::size_t
		GnssVelocityNoiseAppliedSubscriptionCount =
			GnssVelocityNoiseAppliedSubscriptions::size();

	using DownwardRangeBiasAppliedBaseline = fault_injection::CumulativeFeedbackBaseline;
	using DownwardRangeBiasAppliedSubscriptions =
		uORB::SubscriptionMultiArray<fault_downward_range_bias_applied_s>;
	static constexpr std::size_t DownwardRangeBiasAppliedSubscriptionCount =
		DownwardRangeBiasAppliedSubscriptions::size();
	using DownwardRangeNoiseAppliedBaseline = fault_injection::CumulativeFeedbackBaseline;
	using DownwardRangeNoiseAppliedSubscriptions =
		uORB::SubscriptionMultiArray<fault_downward_range_noise_applied_s>;
	static constexpr std::size_t DownwardRangeNoiseAppliedSubscriptionCount =
		DownwardRangeNoiseAppliedSubscriptions::size();
	using DownwardRangeDegradationAppliedBaseline = fault_injection::CumulativeFeedbackBaseline;
	using DownwardRangeDegradationAppliedSubscriptions =
		uORB::SubscriptionMultiArray<fault_downward_range_degradation_applied_s>;
	static constexpr std::size_t DownwardRangeDegradationAppliedSubscriptionCount =
		DownwardRangeDegradationAppliedSubscriptions::size();
	using BaroAltitudeBiasAppliedBaseline = fault_injection::CumulativeFeedbackBaseline;
	using BaroAltitudeBiasAppliedSubscriptions =
		uORB::SubscriptionMultiArray<fault_baro_altitude_bias_applied_s>;
	static constexpr std::size_t BaroAltitudeBiasAppliedSubscriptionCount =
		BaroAltitudeBiasAppliedSubscriptions::size();
	using BaroAltitudeNoiseAppliedBaseline = fault_injection::CumulativeFeedbackBaseline;
	using BaroAltitudeNoiseAppliedSubscriptions =
		uORB::SubscriptionMultiArray<fault_baro_altitude_noise_applied_s>;
	static constexpr std::size_t BaroAltitudeNoiseAppliedSubscriptionCount =
		BaroAltitudeNoiseAppliedSubscriptions::size();
	using MotorOutputBiasAppliedBaseline = fault_injection::CumulativeFeedbackBaseline;
	using MotorOutputBiasAppliedSubscriptions =
		uORB::SubscriptionMultiArray<fault_motor_output_bias_applied_s>;
	static constexpr std::size_t MotorOutputBiasAppliedSubscriptionCount =
		MotorOutputBiasAppliedSubscriptions::size();
	using MotorOutputDelayAppliedBaseline = fault_injection::CumulativeFeedbackBaseline;
	using MotorOutputDelayAppliedSubscriptions =
		uORB::SubscriptionMultiArray<fault_motor_output_delay_applied_s>;
	static constexpr std::size_t MotorOutputDelayAppliedSubscriptionCount =
		MotorOutputDelayAppliedSubscriptions::size();
	using MotorMaxOutputDegradationAppliedBaseline =
		fault_injection::CumulativeFeedbackBaseline;
	using MotorMaxOutputDegradationAppliedSubscriptions =
		uORB::SubscriptionMultiArray<
			fault_motor_max_output_degradation_applied_s>;
	static constexpr std::size_t
		MotorMaxOutputDegradationAppliedSubscriptionCount =
			MotorMaxOutputDegradationAppliedSubscriptions::size();

	fault_injection::FaultRegistry _fault_registry{};

	uORB::Subscription _fault_command_sub{ORB_ID(fault_command)};

	ImuAccelBiasAppliedSubscriptions
		_imu_accel_bias_applied_subs{
			ORB_ID::fault_imu_accel_bias_applied
		};

	ImuAccelNoiseAppliedSubscriptions
		_imu_accel_noise_applied_subs{
			ORB_ID::fault_imu_accel_noise_applied
		};
	ImuGyroNoiseAppliedSubscriptions
		_imu_gyro_noise_applied_subs{
			ORB_ID::fault_imu_gyro_noise_applied
		};

	ImuGyroBiasAppliedSubscriptions
		_imu_gyro_bias_applied_subs{
			ORB_ID::fault_imu_gyro_bias_applied
		};
	MagBiasAppliedSubscriptions
		_mag_bias_applied_subs{
			ORB_ID::fault_mag_bias_applied
		};
	MagNoiseAppliedSubscriptions _mag_noise_applied_subs{
		ORB_ID::fault_mag_noise_applied};
	GnssPositionBiasAppliedSubscriptions
		_gnss_position_bias_applied_subs{
			ORB_ID::fault_gnss_position_bias_applied};
	GnssPositionNoiseAppliedSubscriptions
		_gnss_position_noise_applied_subs{
			ORB_ID::fault_gnss_position_noise_applied};
	GnssVelocityBiasAppliedSubscriptions
		_gnss_velocity_bias_applied_subs{
			ORB_ID::fault_gnss_velocity_bias_applied};
	GnssVelocityNoiseAppliedSubscriptions
		_gnss_velocity_noise_applied_subs{
			ORB_ID::fault_gnss_velocity_noise_applied};
	DownwardRangeBiasAppliedSubscriptions _downward_range_bias_applied_subs{
		ORB_ID::fault_downward_range_bias_applied};
	DownwardRangeNoiseAppliedSubscriptions _downward_range_noise_applied_subs{
		ORB_ID::fault_downward_range_noise_applied};
	DownwardRangeDegradationAppliedSubscriptions _downward_range_degradation_applied_subs{
		ORB_ID::fault_downward_range_degradation_applied};
	BaroAltitudeBiasAppliedSubscriptions _baro_altitude_bias_applied_subs{
		ORB_ID::fault_baro_altitude_bias_applied};
	BaroAltitudeNoiseAppliedSubscriptions _baro_altitude_noise_applied_subs{
		ORB_ID::fault_baro_altitude_noise_applied};
	MotorOutputBiasAppliedSubscriptions _motor_output_bias_applied_subs{
		ORB_ID::fault_motor_output_bias_applied};
	MotorOutputDelayAppliedSubscriptions _motor_output_delay_applied_subs{
		ORB_ID::fault_motor_output_delay_applied};
	MotorMaxOutputDegradationAppliedSubscriptions
		_motor_max_output_degradation_applied_subs{
			ORB_ID::fault_motor_max_output_degradation_applied};

	uORB::Publication<fault_ack_s> _fault_ack_pub{
		ORB_ID(fault_ack)
	};

	uORB::Publication<fault_status_s> _fault_status_pub{
		ORB_ID(fault_status)
	};

	uORB::Publication<fault_imu_accel_bias_s> _imu_accel_bias_pub{
		ORB_ID(fault_imu_accel_bias)
	};

	uORB::Publication<fault_imu_accel_noise_s> _imu_accel_noise_pub{
		ORB_ID(fault_imu_accel_noise)
	};
	uORB::Publication<fault_imu_gyro_noise_s> _imu_gyro_noise_pub{
		ORB_ID(fault_imu_gyro_noise)
	};

	uORB::Publication<fault_imu_gyro_bias_s> _imu_gyro_bias_pub{
		ORB_ID(fault_imu_gyro_bias)
	};
	uORB::Publication<fault_mag_bias_s> _mag_bias_pub{
		ORB_ID(fault_mag_bias)
	};
	uORB::Publication<fault_mag_noise_s> _mag_noise_pub{
		ORB_ID(fault_mag_noise)};
	uORB::Publication<fault_gnss_position_bias_s>
		_gnss_position_bias_pub{ORB_ID(fault_gnss_position_bias)};
	uORB::Publication<fault_gnss_position_noise_s>
		_gnss_position_noise_pub{ORB_ID(fault_gnss_position_noise)};
	uORB::Publication<fault_gnss_velocity_bias_s>
		_gnss_velocity_bias_pub{ORB_ID(fault_gnss_velocity_bias)};
	uORB::Publication<fault_gnss_velocity_noise_s>
		_gnss_velocity_noise_pub{ORB_ID(fault_gnss_velocity_noise)};
	uORB::Publication<fault_downward_range_bias_s> _downward_range_bias_pub{
		ORB_ID(fault_downward_range_bias)};
	uORB::Publication<fault_downward_range_noise_s> _downward_range_noise_pub{
		ORB_ID(fault_downward_range_noise)};
	uORB::Publication<fault_downward_range_degradation_s> _downward_range_degradation_pub{
		ORB_ID(fault_downward_range_degradation)};
	uORB::Publication<fault_baro_altitude_bias_s> _baro_altitude_bias_pub{
		ORB_ID(fault_baro_altitude_bias)};
	uORB::Publication<fault_baro_altitude_noise_s> _baro_altitude_noise_pub{
		ORB_ID(fault_baro_altitude_noise)};
	uORB::Publication<fault_motor_output_bias_s> _motor_output_bias_pub{
		ORB_ID(fault_motor_output_bias)};
	uORB::Publication<fault_motor_output_delay_s> _motor_output_delay_pub{
		ORB_ID(fault_motor_output_delay)};
	uORB::Publication<fault_motor_max_output_degradation_s>
		_motor_max_output_degradation_pub{
			ORB_ID(fault_motor_max_output_degradation)};

	fault_command_s _last_command{};
	fault_ack_s _last_ack{};
	fault_status_s _last_status{};
	fault_imu_accel_bias_s _last_imu_accel_bias{};
	fault_imu_accel_noise_s _last_imu_accel_noise{};
	fault_imu_gyro_noise_s _last_imu_gyro_noise{};
	fault_imu_gyro_bias_s _last_imu_gyro_bias{};
	fault_mag_bias_s _last_mag_bias{};
	fault_mag_noise_s _last_mag_noise{};
	fault_gnss_position_bias_s _last_gnss_position_bias{};
	fault_gnss_position_noise_s _last_gnss_position_noise{};
	fault_gnss_velocity_bias_s _last_gnss_velocity_bias{};
	fault_gnss_velocity_noise_s _last_gnss_velocity_noise{};
	fault_imu_accel_bias_applied_s
		_last_imu_accel_bias_applied{};
	fault_imu_accel_noise_applied_s
		_last_imu_accel_noise_applied{};
	fault_imu_gyro_noise_applied_s
		_last_imu_gyro_noise_applied{};
	fault_imu_gyro_bias_applied_s
		_last_imu_gyro_bias_applied{};
	fault_mag_bias_applied_s
		_last_mag_bias_applied{};
	fault_mag_noise_applied_s _last_mag_noise_applied{};
	fault_gnss_position_bias_applied_s
		_last_gnss_position_bias_applied{};
	fault_gnss_position_noise_applied_s
		_last_gnss_position_noise_applied{};
	fault_gnss_velocity_bias_applied_s
		_last_gnss_velocity_bias_applied{};
	fault_gnss_velocity_noise_applied_s
		_last_gnss_velocity_noise_applied{};

	fault_injection::FixedArray<
		ImuAccelBiasAppliedBaseline,
		ImuAccelBiasAppliedSubscriptionCount>
		_imu_accel_bias_applied_baselines{};

	fault_injection::FixedArray<
		ImuAccelNoiseAppliedBaseline,
		ImuAccelNoiseAppliedSubscriptionCount>
		_imu_accel_noise_applied_baselines{};
	fault_injection::FixedArray<
		ImuGyroNoiseAppliedBaseline,
		ImuGyroNoiseAppliedSubscriptionCount>
		_imu_gyro_noise_applied_baselines{};

	fault_injection::FixedArray<
		ImuGyroBiasAppliedBaseline,
		ImuGyroBiasAppliedSubscriptionCount>
		_imu_gyro_bias_applied_baselines{};
	fault_injection::FixedArray<
		MagBiasAppliedBaseline,
		MagBiasAppliedSubscriptionCount>
		_mag_bias_applied_baselines{};
	fault_injection::FixedArray<MagNoiseAppliedBaseline, MagNoiseAppliedSubscriptionCount>
		_mag_noise_applied_baselines{};
	fault_injection::FixedArray<
		GnssPositionBiasAppliedBaseline,
		GnssPositionBiasAppliedSubscriptionCount>
		_gnss_position_bias_applied_baselines{};
	fault_injection::FixedArray<
		GnssPositionNoiseAppliedBaseline,
		GnssPositionNoiseAppliedSubscriptionCount>
		_gnss_position_noise_applied_baselines{};
	fault_injection::FixedArray<
		GnssVelocityBiasAppliedBaseline,
		GnssVelocityBiasAppliedSubscriptionCount>
		_gnss_velocity_bias_applied_baselines{};
	fault_injection::FixedArray<
		GnssVelocityNoiseAppliedBaseline,
		GnssVelocityNoiseAppliedSubscriptionCount>
		_gnss_velocity_noise_applied_baselines{};
	fault_injection::FixedArray<DownwardRangeBiasAppliedBaseline,
		DownwardRangeBiasAppliedSubscriptionCount>
		_downward_range_bias_applied_baselines{};
	fault_injection::FixedArray<DownwardRangeNoiseAppliedBaseline,
		DownwardRangeNoiseAppliedSubscriptionCount>
		_downward_range_noise_applied_baselines{};
	fault_injection::FixedArray<DownwardRangeDegradationAppliedBaseline,
		DownwardRangeDegradationAppliedSubscriptionCount>
		_downward_range_degradation_applied_baselines{};
	fault_injection::FixedArray<BaroAltitudeBiasAppliedBaseline,
		BaroAltitudeBiasAppliedSubscriptionCount>
		_baro_altitude_bias_applied_baselines{};
	fault_injection::FixedArray<BaroAltitudeNoiseAppliedBaseline,
		BaroAltitudeNoiseAppliedSubscriptionCount>
		_baro_altitude_noise_applied_baselines{};
	fault_injection::FixedArray<MotorOutputBiasAppliedBaseline,
		MotorOutputBiasAppliedSubscriptionCount>
		_motor_output_bias_applied_baselines{};
	fault_injection::FixedArray<MotorOutputDelayAppliedBaseline,
		MotorOutputDelayAppliedSubscriptionCount>
		_motor_output_delay_applied_baselines{};
	fault_injection::FixedArray<MotorMaxOutputDegradationAppliedBaseline,
		MotorMaxOutputDegradationAppliedSubscriptionCount>
		_motor_max_output_degradation_applied_baselines{};

	uint64_t _run_count{0};
	uint64_t _received_command_count{0};
	uint64_t _published_ack_count{0};
	uint64_t _published_status_count{0};
	uint64_t _published_imu_accel_bias_count{0};
	uint64_t _published_imu_accel_noise_count{0};
	uint64_t _published_imu_gyro_noise_count{0};
	uint64_t _received_imu_accel_bias_applied_count{0};
	uint64_t _accepted_imu_accel_bias_applied_count{0};
	uint64_t _rejected_imu_accel_bias_applied_count{0};
	uint64_t _recorded_imu_accel_bias_sample_count{0};
	uint64_t _received_imu_accel_noise_applied_count{0};
	uint64_t _accepted_imu_accel_noise_applied_count{0};
	uint64_t _rejected_imu_accel_noise_applied_count{0};
	uint64_t _recorded_imu_accel_noise_sample_count{0};
	uint64_t _received_imu_gyro_noise_applied_count{0};
	uint64_t _accepted_imu_gyro_noise_applied_count{0};
	uint64_t _rejected_imu_gyro_noise_applied_count{0};
	uint64_t _recorded_imu_gyro_noise_sample_count{0};
	uint64_t _published_imu_gyro_bias_count{0};
	uint64_t _published_mag_bias_count{0};
	uint64_t _received_imu_gyro_bias_applied_count{0};
	uint64_t _accepted_imu_gyro_bias_applied_count{0};
	uint64_t _rejected_imu_gyro_bias_applied_count{0};
	uint64_t _recorded_imu_gyro_bias_sample_count{0};
	uint64_t _received_mag_bias_applied_count{0};
	uint64_t _accepted_mag_bias_applied_count{0};
	uint64_t _rejected_mag_bias_applied_count{0};
	uint64_t _recorded_mag_bias_sample_count{0};
	uint64_t _published_mag_noise_count{0};
	uint64_t _received_mag_noise_applied_count{0};
	uint64_t _accepted_mag_noise_applied_count{0};
	uint64_t _rejected_mag_noise_applied_count{0};
	uint64_t _recorded_mag_noise_sample_count{0};
	uint64_t _published_gnss_position_bias_count{0};
	uint64_t _received_gnss_position_bias_applied_count{0};
	uint64_t _accepted_gnss_position_bias_applied_count{0};
	uint64_t _rejected_gnss_position_bias_applied_count{0};
	uint64_t _recorded_gnss_position_bias_sample_count{0};
	uint64_t _published_gnss_position_noise_count{0};
	uint64_t _received_gnss_position_noise_applied_count{0};
	uint64_t _accepted_gnss_position_noise_applied_count{0};
	uint64_t _rejected_gnss_position_noise_applied_count{0};
	uint64_t _recorded_gnss_position_noise_sample_count{0};
	uint64_t _published_gnss_velocity_bias_count{0};
	uint64_t _received_gnss_velocity_bias_applied_count{0};
	uint64_t _accepted_gnss_velocity_bias_applied_count{0};
	uint64_t _rejected_gnss_velocity_bias_applied_count{0};
	uint64_t _recorded_gnss_velocity_bias_sample_count{0};
	uint64_t _published_gnss_velocity_noise_count{0};
	uint64_t _received_gnss_velocity_noise_applied_count{0};
	uint64_t _accepted_gnss_velocity_noise_applied_count{0};
	uint64_t _rejected_gnss_velocity_noise_applied_count{0};
	uint64_t _recorded_gnss_velocity_noise_sample_count{0};

	uint32_t _last_imu_accel_bias_applied_delta{0};
	uint32_t _last_imu_accel_noise_applied_delta{0};
	uint32_t _last_imu_gyro_noise_applied_delta{0};
	uint32_t _last_imu_gyro_bias_applied_delta{0};
	uint32_t _last_mag_bias_applied_delta{0};
	uint32_t _last_mag_noise_applied_delta{0};
	uint32_t _last_gnss_position_bias_applied_delta{0};
	uint32_t _last_gnss_position_noise_applied_delta{0};
	uint32_t _last_gnss_velocity_bias_applied_delta{0};
	uint32_t _last_gnss_velocity_noise_applied_delta{0};

	bool _has_last_command{false};
	bool _has_last_ack{false};
	bool _has_last_status{false};
	bool _has_last_imu_accel_bias{false};
	bool _has_last_imu_accel_noise{false};
	bool _has_last_imu_gyro_noise{false};
	bool _has_last_imu_gyro_bias{false};
	bool _has_last_mag_bias{false};
	bool _has_last_mag_noise{false};
	bool _has_last_gnss_position_bias{false};
	bool _has_last_gnss_position_noise{false};
	bool _has_last_gnss_velocity_bias{false};
	bool _has_last_gnss_velocity_noise{false};
	bool _has_last_imu_accel_bias_applied{false};
	bool _has_last_imu_accel_noise_applied{false};
	bool _has_last_imu_gyro_noise_applied{false};
	bool _has_last_imu_gyro_bias_applied{false};
	bool _has_last_mag_bias_applied{false};
	bool _has_last_mag_noise_applied{false};
	bool _has_last_gnss_position_bias_applied{false};
	bool _has_last_gnss_position_noise_applied{false};
	bool _has_last_gnss_velocity_bias_applied{false};
	bool _has_last_gnss_velocity_noise_applied{false};
	bool _last_imu_accel_bias_applied_accepted{false};
	bool _last_imu_accel_noise_applied_accepted{false};
	bool _last_imu_gyro_noise_applied_accepted{false};
	bool _last_imu_gyro_bias_applied_accepted{false};
	bool _last_mag_bias_applied_accepted{false};
	bool _last_mag_noise_applied_accepted{false};
	bool _last_gnss_position_bias_applied_accepted{false};
	bool _last_gnss_position_noise_applied_accepted{false};
	bool _last_gnss_velocity_bias_applied_accepted{false};
	bool _last_gnss_velocity_noise_applied_accepted{false};
};
