/****************************************************************************
 *
 *   FAULT IMU gyroscope-bias runtime snapshot consumer.
 *
 ****************************************************************************/

#pragma once

#include <cstddef>
#include <cstdint>

#include <lib/fault_injection/CumulativeApplicationTracker.hpp>
#include <matrix/matrix/math.hpp>
#include <px4_platform_common/defines.h>

#include <uORB/PublicationMulti.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/topics/fault_imu_gyro_bias.h>
#include <uORB/topics/fault_imu_gyro_bias_applied.h>

class ImuGyroBiasInjector final
{
public:
	/**
	 * Copy the newest dedicated runtime-effect snapshot, when available.
	 *
	 * The cache transition also publishes zero-count baseline or final
	 * cumulative feedback reports.
	 */
	bool update(int estimator_instance);

	/**
	 * Record one successful IMU-sample modification.
	 *
	 * The data-application path calls this only after delta_angle is
	 * actually modified.
	 */
	bool record_application(
		uint64_t timestamp_sample,
		int estimator_instance);

	/**
	 * Apply the cached body-frame gyroscope bias to one integrated
	 * delta-angle sample.
	 *
	 * Returns true only when a finite, non-zero correction is committed.
	 * Application evidence is recorded only after the sample is modified.
	 */
	bool apply(
		matrix::Vector3f &delta_angle,
		float delta_angle_dt_s,
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

	float bias_rad_s(std::size_t axis) const
	{
		return (axis < 3) ? _bias_rad_s[axis] : 0.0f;
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

private:
	void reset_effect_to_neutral();
	void set_estimator_instance(int estimator_instance);
	bool publish_feedback(bool final_report);

	bool same_active_definition(
		const fault_imu_gyro_bias_s &snapshot) const;

	uORB::Subscription _snapshot_sub{
		ORB_ID(fault_imu_gyro_bias)
	};

	uORB::PublicationMulti<fault_imu_gyro_bias_applied_s>
		_applied_feedback_pub{
			ORB_ID(fault_imu_gyro_bias_applied)
		};

	fault_imu_gyro_bias_s _last_snapshot{};

	float _bias_rad_s[3]{0.0f, 0.0f, 0.0f};

	fault_injection::CumulativeApplicationTracker
		_application_tracker{100000ULL};

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
