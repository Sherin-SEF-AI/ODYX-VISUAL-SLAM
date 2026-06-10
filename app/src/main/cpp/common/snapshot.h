// ODYX :: snapshots pulled up across JNI (copied out, never shared by ref).
#pragma once
#include "common/odyx_types.h"
#include <string>

namespace odyx {

struct Telemetry {
    int tracking_state = 0;          // TrackingState
    int init_state = 0;              // InitState
    int gnss_mode = 0;               // GnssMode

    int n_tracked = 0;
    int n_keyframes = 0;
    int window_size = 0;
    int n_landmarks = 0;

    double frontend_ms = 0;
    double optimize_ms = 0;
    double imu_rate = 0, frame_rate = 0, gnss_rate = 0;

    double td = 0;                   // estimated cam<->imu offset (s)
    Vec3 bg = Vec3::Zero();
    Vec3 ba = Vec3::Zero();
    Vec3 vel = Vec3::Zero();
    double gravity_align_resid = 0;

    int gnss_n_sats = 0;
    int gnss_fix_type = 0;
    double gnss_acc_m = 0;
    bool enu_aligned = false;
    double enu_resid_m = 0;

    int loop_count = 0;
    double drift_proxy_m = 0;        // |VIO - GNSS| position delta
    int thermal_status = 0;          // 0 none .. higher = throttling
    int frame_stride = 1;
    int target_features = 0;
    bool calib_default = false;      // running on default (uncalibrated) values
};

struct LandmarkOut {
    uint64_t id = 0;
    Vec3 p_w = Vec3::Zero();
    float cov_trace = 0;
};

struct LoopEdge {
    uint64_t kf_a = 0, kf_b = 0;
};

struct MapSnapshot {
    std::vector<NavStateOut> trajectory_local;   // per keyframe
    std::vector<Vec3> trajectory_enu;            // ENU-anchored, if aligned
    std::vector<LandmarkOut> landmarks;
    std::vector<LoopEdge> loop_edges;
    std::vector<Vec3> gnss_markers_enu;
    NavStateOut current;
    bool enu_valid = false;
};

}  // namespace odyx
