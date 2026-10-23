/****************************************************************************
 *
 *   Shared GNSS position-fault conversion helpers.
 *
 ****************************************************************************/

#pragma once

#include "EKF/common.h"

#include <float.h>
#include <cmath>

#include <lib/geo/geo.h>
#include <matrix/matrix/math.hpp>

namespace fault_gnss
{

/**
 * Apply a finite local-NED position displacement to one EKF GNSS sample.
 *
 * X/North and Y/East are converted from metres to WGS-84 latitude/longitude
 * using PX4's geo library. Z is positive Down, so it is subtracted from the
 * MSL altitude. No health, accuracy, fix, velocity, yaw or timing field is
 * modified.
 */
inline bool apply_ned_position_offset(
	estimator::gnssSample &sample,
	const matrix::Vector3f &offset_ned_m)
{
	if (!std::isfinite(sample.lat)
	    || !std::isfinite(sample.lon)
	    || !std::isfinite(sample.alt)
	    || !offset_ned_m.isAllFinite()
	    || !offset_ned_m.longerThan(0.0f)
	    || (sample.lat < -90.0)
	    || (sample.lat > 90.0)
	    || (sample.lon < -180.0)
	    || (sample.lon > 180.0)) {
		return false;
	}

	double faulted_lat = sample.lat;
	double faulted_lon = sample.lon;

	add_vector_to_global_position(
		sample.lat,
		sample.lon,
		offset_ned_m(0),
		offset_ned_m(1),
		&faulted_lat,
		&faulted_lon);

	const double faulted_alt =
		static_cast<double>(sample.alt)
		- static_cast<double>(offset_ned_m(2));

	if (!std::isfinite(faulted_lat)
	    || !std::isfinite(faulted_lon)
	    || !std::isfinite(faulted_alt)
	    || (faulted_lat < -90.0)
	    || (faulted_lat > 90.0)
	    || (faulted_lon < -180.0)
	    || (faulted_lon > 180.0)
	    || (faulted_alt
		< -static_cast<double>(
			FLT_MAX))
	    || (faulted_alt
		> static_cast<double>(
			FLT_MAX))) {
		return false;
	}

	sample.lat = faulted_lat;
	sample.lon = faulted_lon;
	sample.alt = static_cast<float>(faulted_alt);
	return true;
}

} // namespace fault_gnss
