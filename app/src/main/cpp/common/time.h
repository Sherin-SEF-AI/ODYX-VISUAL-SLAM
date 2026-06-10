// ===========================================================================
// ODYX :: time-base conventions (PRIORITY ZERO).
//
// Every timestamp that crosses the JNI boundary has ALREADY been reduced by the
// Kotlin time/ layer to Android elapsedRealtimeNanos (a single monotonic
// nanosecond clock that keeps counting in deep sleep). The native side NEVER
// re-bases time; it only:
//   - looks up IMU intervals between keyframe times,
//   - estimates the residual camera<->IMU offset td online (a state variable),
//   - applies per-row rolling-shutter offsets relative to the frame timestamp.
//
// The residual td (seconds) is defined so that:
//     t_imu_aligned = t_cam + td
// i.e. the true IMU time of a camera observation taken at frame time t_cam is
// (t_cam + td). td is small (hardware latency / clock skew not removed by the
// elapsedRealtimeNanos reduction) and is refined inside the smoother.
// ===========================================================================
#pragma once
#include "common/odyx_types.h"

namespace odyx {

// ns_to_s / s_to_ns live in common/odyx_types.h (included everywhere).

// Rolling-shutter: time at which image row r (0..H-1) was exposed.
inline TimeNs rowTime(const Frame& f, double row) {
    if (f.height <= 1 || f.rolling_shutter_skew_ns == 0) return f.t_ns;
    // f.t_ns is the exposure midpoint of the frame; the skew spans top->bottom.
    // Center the readout window on the midpoint so the mean row matches t_ns.
    const double frac = (row / static_cast<double>(f.height - 1)) - 0.5;
    return f.t_ns + static_cast<TimeNs>(frac * static_cast<double>(f.rolling_shutter_skew_ns));
}

}  // namespace odyx
