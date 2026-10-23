/****************************************************************************
 *
 *   FAULT downward-range effective-maximum-distance degradation consumer.
 *
 ****************************************************************************/
#pragma once

#include <cstdint>

#include <lib/fault_injection/CumulativeApplicationTracker.hpp>

#include <uORB/PublicationMulti.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/topics/fault_downward_range_degradation.h>
#include <uORB/topics/fault_downward_range_degradation_applied.h>

class DownwardRangeDegradationInjector final
{
public:
	bool update(int estimator_instance);
	bool gate(float raw_distance_m, float nominal_max_distance_m,
		  uint64_t timestamp_sample, int estimator_instance,
		  float &effective_max_distance_m);

	bool has_snapshot() const { return _has_snapshot; }
	bool active() const { return _active; }
	float configured_max_distance_m() const { return _configured_max_distance_m; }
	float effective_max_distance_m(float nominal_max_distance_m) const;
	uint32_t scenario_id() const { return _last_snapshot.scenario_id; }
	uint32_t sequence() const { return _last_snapshot.sequence; }
	uint16_t fault_id() const { return _last_snapshot.fault_id; }
	uint32_t update_count() const { return _update_count; }
	uint32_t active_snapshot_count() const { return _active_snapshot_count; }
	uint32_t neutral_snapshot_count() const { return _neutral_snapshot_count; }
	uint32_t cumulative_applied_count() const { return _application_tracker.cumulative_count(); }

private:
	void reset_effect_to_neutral();
	void set_estimator_instance(int estimator_instance);
	bool record_application(uint64_t timestamp_sample, int estimator_instance);
	bool publish_feedback(bool final_report);
	bool same_active_definition(const fault_downward_range_degradation_s &snapshot) const;

	uORB::Subscription _snapshot_sub{ORB_ID(fault_downward_range_degradation)};
	uORB::PublicationMulti<fault_downward_range_degradation_applied_s>
		_applied_feedback_pub{ORB_ID(fault_downward_range_degradation_applied)};

	fault_downward_range_degradation_s _last_snapshot{};
	float _configured_max_distance_m{0.0f};
	fault_injection::CumulativeApplicationTracker _application_tracker{100000ULL};
	uint32_t _update_count{0};
	uint32_t _active_snapshot_count{0};
	uint32_t _neutral_snapshot_count{0};
	uint8_t _estimator_instance{UINT8_MAX};
	bool _has_snapshot{false};
	bool _active{false};
};
