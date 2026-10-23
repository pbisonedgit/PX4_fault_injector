/****************************************************************************
 *
 *   FAULT IMU gyroscope-noise runtime snapshot consumer.
 *
 ****************************************************************************/

#pragma once

#include <cstddef>
#include <cstdint>

#include <lib/fault_injection/CumulativeApplicationTracker.hpp>
#include <lib/fault_injection/DeterministicGaussianGenerator.hpp>
#include <matrix/matrix/math.hpp>
#include <px4_platform_common/defines.h>

#include <uORB/PublicationMulti.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/topics/fault_imu_gyro_noise.h>
#include <uORB/topics/fault_imu_gyro_noise_applied.h>

class ImuGyroNoiseInjector final
{
public:
	/**
	 * Copy the newest dedicated runtime-effect snapshot into this EKF2
	 * instance's local cache.
	 *
	 * A new ACTIVE definition also initializes deterministic, independent
	 * per-axis Gaussian angular-rate random streams.
	 */
	bool update(int estimator_instance);

	/**
	 * Apply one deterministic zero-mean Gaussian gyroscope-noise sample to
	 * an integrated body-frame delta-angle sample.
	 *
	 * Returns true only after a finite, non-zero delta-angle increment is
	 * committed and application evidence is recorded.
	 */
	bool apply(
		matrix::Vector3f &delta_angle,
		float delta_angle_dt_s,
		uint64_t timestamp_sample,
		int estimator_instance);

	bool record_application(
		uint64_t timestamp_sample,
		int estimator_instance);

	bool has_snapshot() const
	{
		return _has_snapshot;
	}

	bool active() const
	{
		return _active;
	}

	uint8_t axis_mask() const
	{
		return _axis_mask;
	}

	uint32_t random_seed() const
	{
		return _random_seed;
	}

	float sigma_rad_s(std::size_t axis) const
	{
		return (axis < 3) ? _sigma_rad_s[axis] : 0.0f;
	}

	uint64_t timestamp() const
	{
		return _last_snapshot.timestamp;
	}

	uint32_t scenario_id() const
	{
		return _last_snapshot.scenario_id;
	}

	uint32_t sequence() const
	{
		return _last_snapshot.sequence;
	}

	uint16_t fault_id() const
	{
		return _last_snapshot.fault_id;
	}

	uint32_t update_count() const
	{
		return _update_count;
	}

	uint32_t active_snapshot_count() const
	{
		return _active_snapshot_count;
	}

	uint32_t neutral_snapshot_count() const
	{
		return _neutral_snapshot_count;
	}

	uint32_t cumulative_applied_count() const
	{
		return _application_tracker.cumulative_count();
	}

	uint32_t feedback_published_count() const
	{
		return _feedback_published_count;
	}

	uint32_t feedback_publish_failure_count() const
	{
		return _feedback_publish_failure_count;
	}

	bool generator_initialized() const
	{
		return _gaussian_generator.initialized();
	}

	uint32_t generator_initialization_count() const
	{
		return _gaussian_generator.initialization_count();
	}

	uint32_t generated_sample_count() const
	{
		return _generated_sample_count;
	}

	float first_noise_angular_rate_rad_s(std::size_t axis) const
	{
		return (axis < 3) ? _first_noise_angular_rate_rad_s(axis) : 0.0f;
	}

	float last_noise_angular_rate_rad_s(std::size_t axis) const
	{
		return (axis < 3) ? _last_noise_angular_rate_rad_s(axis) : 0.0f;
	}

	float last_noise_delta_angle(std::size_t axis) const
	{
		return (axis < 3) ? _last_noise_delta_angle(axis) : 0.0f;
	}

private:
	static constexpr std::size_t AxisCount = 3;

	void reset_effect_to_neutral();
	void reset_generator_diagnostics();
	void set_estimator_instance(int estimator_instance);

	bool publish_feedback(bool final_report);
	bool same_active_definition(const fault_imu_gyro_noise_s &snapshot) const;

	uORB::Subscription _snapshot_sub{
		ORB_ID(fault_imu_gyro_noise)
	};

	uORB::PublicationMulti<fault_imu_gyro_noise_applied_s>
		_applied_feedback_pub{
			ORB_ID(fault_imu_gyro_noise_applied)
		};

	fault_imu_gyro_noise_s _last_snapshot{};

	matrix::Vector3f _first_noise_angular_rate_rad_s{};
	matrix::Vector3f _last_noise_angular_rate_rad_s{};
	matrix::Vector3f _last_noise_delta_angle{};

	float _sigma_rad_s[AxisCount]{0.0f, 0.0f, 0.0f};

	fault_injection::CumulativeApplicationTracker
		_application_tracker{100000ULL};

	fault_injection::DeterministicGaussianGenerator<AxisCount>
		_gaussian_generator{};

	uint32_t _random_seed{0};
	uint32_t _generated_sample_count{0};
	uint32_t _update_count{0};
	uint32_t _active_snapshot_count{0};
	uint32_t _neutral_snapshot_count{0};
	uint32_t _feedback_published_count{0};
	uint32_t _feedback_publish_failure_count{0};

	uint8_t _estimator_instance{UINT8_MAX};
	uint8_t _axis_mask{0};

	bool _has_snapshot{false};
	bool _active{false};
};
