/****************************************************************************
 *
 *   FAULT deterministic barometric-altitude noise runtime snapshot consumer.
 *
 ****************************************************************************/
#pragma once

#include <cstdint>

#include <lib/fault_injection/CumulativeApplicationTracker.hpp>
#include <lib/fault_injection/DeterministicGaussianGenerator.hpp>

#include <uORB/PublicationMulti.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/topics/fault_baro_altitude_noise.h>
#include <uORB/topics/fault_baro_altitude_noise_applied.h>

class BaroAltitudeNoiseInjector final
{
public:
	bool update(int estimator_instance);
	bool apply(float &altitude_m, uint64_t timestamp_sample,
		   int estimator_instance, float &applied_noise_m);

	bool has_snapshot() const { return _has_snapshot; }
	bool active() const { return _active; }
	float sigma_m() const { return _sigma_m; }
	uint32_t random_seed() const { return _random_seed; }
	uint32_t scenario_id() const { return _last_snapshot.scenario_id; }
	uint32_t sequence() const { return _last_snapshot.sequence; }
	uint16_t fault_id() const { return _last_snapshot.fault_id; }
	uint32_t update_count() const { return _update_count; }
	uint32_t active_snapshot_count() const { return _active_snapshot_count; }
	uint32_t neutral_snapshot_count() const { return _neutral_snapshot_count; }
	uint32_t cumulative_applied_count() const { return _application_tracker.cumulative_count(); }
	uint32_t feedback_published_count() const { return _feedback_published_count; }
	uint32_t feedback_publish_failure_count() const { return _feedback_publish_failure_count; }
	bool generator_initialized() const { return _gaussian_generator.initialized(); }
	uint32_t generator_initialization_count() const { return _gaussian_generator.initialization_count(); }
	uint32_t generated_sample_count() const { return _generated_sample_count; }
	float first_noise_m() const { return _first_noise_m; }
	float last_noise_m() const { return _last_noise_m; }

private:
	static constexpr uint32_t StreamDomain = 0x4241524FU; // "BARO"
	void reset_effect_to_neutral();
	void reset_generator_diagnostics();
	void set_estimator_instance(int estimator_instance);
	bool record_application(uint64_t timestamp_sample, int estimator_instance);
	bool publish_feedback(bool final_report);
	bool same_active_definition(const fault_baro_altitude_noise_s &snapshot) const;

	uORB::Subscription _snapshot_sub{ORB_ID(fault_baro_altitude_noise)};
	uORB::PublicationMulti<fault_baro_altitude_noise_applied_s>
		_applied_feedback_pub{ORB_ID(fault_baro_altitude_noise_applied)};

	fault_baro_altitude_noise_s _last_snapshot{};
	fault_injection::DeterministicGaussianGenerator<1> _gaussian_generator{};
	fault_injection::CumulativeApplicationTracker _application_tracker{100000ULL};
	float _sigma_m{0.0f};
	float _first_noise_m{0.0f};
	float _last_noise_m{0.0f};
	uint32_t _random_seed{0};
	uint32_t _generated_sample_count{0};
	uint32_t _update_count{0};
	uint32_t _active_snapshot_count{0};
	uint32_t _neutral_snapshot_count{0};
	uint32_t _feedback_published_count{0};
	uint32_t _feedback_publish_failure_count{0};
	uint8_t _estimator_instance{UINT8_MAX};
	bool _has_snapshot{false};
	bool _active{false};
};
