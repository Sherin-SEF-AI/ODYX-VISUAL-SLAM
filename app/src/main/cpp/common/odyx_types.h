// ===========================================================================
// ODYX :: core POD types shared across native modules.
//
// All timestamps are int64 nanoseconds on ONE monotonic base:
// Android elapsedRealtimeNanos (see common/time.h and the Kotlin time/ layer).
// Nothing downstream of the JNI boundary is allowed to use a different clock.
// ===========================================================================
#pragma once

#include <cstdint>
#include <vector>
#include <array>
#include <Eigen/Core>
#include <Eigen/Geometry>

namespace odyx {

using Vec2 = Eigen::Vector2d;
using Vec3 = Eigen::Vector3d;
using Vec4 = Eigen::Vector4d;
using Mat3 = Eigen::Matrix3d;
using Mat4 = Eigen::Matrix4d;
using Quat = Eigen::Quaterniond;

// Nanosecond timestamp on the unified elapsedRealtimeNanos base.
using TimeNs = int64_t;
constexpr double kNsToS = 1e-9;
constexpr double kSToNs = 1e9;

inline double ns_to_s(TimeNs ns) { return static_cast<double>(ns) * kNsToS; }
inline TimeNs s_to_ns(double s)  { return static_cast<TimeNs>(s * kSToNs); }

// ---- IMU sample (uncalibrated accel + gyro, body frame) -------------------
struct ImuSample {
    TimeNs t_ns = 0;
    Vec3 acc = Vec3::Zero();   // m/s^2  (raw, gravity included)
    Vec3 gyr = Vec3::Zero();   // rad/s
    // Factory-reported bias estimates for the *_UNCALIBRATED sensors, if any.
    Vec3 acc_bias_hint = Vec3::Zero();
    Vec3 gyr_bias_hint = Vec3::Zero();
};

// ---- Camera frame (grayscale, already converted from YUV) -----------------
struct Frame {
    TimeNs t_ns = 0;           // exposure-midpoint timestamp, unified base
    int width = 0;
    int height = 0;
    // Per-row readout time (rolling shutter): t_row(r) = t_ns + skew_ns*(r/H).
    int64_t rolling_shutter_skew_ns = 0;
    double exposure_s = 0.0;
    // Tightly-packed 8-bit luma. Owned by the frame.
    std::vector<uint8_t> gray;
    uint64_t seq = 0;
};

// ---- GNSS loosely-coupled fix --------------------------------------------
struct GnssFix {
    TimeNs t_ns = 0;
    double lat_deg = 0, lon_deg = 0, alt_m = 0;
    double hAcc_m = 0;         // reported horizontal accuracy (std, m)
    double vAcc_m = 0;         // reported vertical accuracy
    double speed_mps = 0;
    double bearing_deg = 0;
    int n_sats = 0;
    bool has_alt = false;
    bool valid = false;
};

// ---- GNSS raw measurement (tightly-coupled) -------------------------------
// Mirrors the useful subset of Android GnssMeasurement, reduced to the
// unified time base. RTKLIB consumes these + ephemeris for pseudorange.
struct GnssRawMeas {
    TimeNs t_ns = 0;               // measurement time, unified base
    int constellation = 0;         // GnssStatus.CONSTELLATION_*
    int svid = 0;
    double cn0_dbhz = 0;
    int state = 0;                  // measurement state bitmask
    int64_t received_sv_time_ns = 0;
    int64_t received_sv_time_uncertainty_ns = 0;
    double pseudorange_rate_mps = 0;            // ~ -lambda * Doppler
    double pseudorange_rate_uncertainty_mps = 0;
    double carrier_frequency_hz = 0;
    int64_t time_offset_ns = 0;
    bool full_biases_valid = false;
};

// Receiver clock snapshot, paired with a batch of GnssRawMeas.
struct GnssClock {
    TimeNs t_ns = 0;
    int64_t time_ns = 0;
    int64_t full_bias_ns = 0;
    double bias_ns = 0;
    double drift_nsps = 0;
    int leap_second = 0;
    bool full_bias_valid = false;
    int hw_clock_discontinuity_count = 0;
};

// Broadcast ephemeris record (decoded from GnssNavigationMessage by RTKLIB).
struct GnssNavMsg {
    int constellation = 0;
    int svid = 0;
    int type = 0;
    int submessage_id = 0;
    int message_id = 0;
    std::vector<uint8_t> data;   // raw nav bytes
};

// ---- Camera intrinsics (pinhole + radial-tangential distortion) -----------
struct CameraModel {
    int width = 0, height = 0;
    double fx = 0, fy = 0, cx = 0, cy = 0;
    // radtan: k1 k2 p1 p2 k3
    std::array<double, 5> dist = {0, 0, 0, 0, 0};
    bool valid = false;
};

// ---- 6-DoF pose (T_wb: body in world) + velocity + biases -----------------
struct NavStateOut {
    TimeNs t_ns = 0;
    Quat q_wb = Quat::Identity();   // world<-body rotation
    Vec3 p_wb = Vec3::Zero();       // body position in local world (gravity-aligned)
    Vec3 v_w = Vec3::Zero();        // velocity in world
    Vec3 ba = Vec3::Zero();         // accel bias
    Vec3 bg = Vec3::Zero();         // gyro bias
    Eigen::Matrix<double, 6, 6> pose_cov = Eigen::Matrix<double, 6, 6>::Identity();
    bool valid = false;
};

// Tracking lifecycle.
enum class TrackingState : int {
    kUninitialized = 0,   // collecting bootstrap data
    kInitializing  = 1,   // running VI alignment
    kNominal       = 2,   // healthy VIO
    kUnstable      = 3,   // degraded, still emitting
    kLost          = 4,   // tracking lost, attempting relocalization
};

enum class InitState : int {
    kCollecting = 0,
    kSfm        = 1,
    kVialign    = 2,
    kDone       = 3,
    kFailed     = 4,
};

enum class GnssMode : int {
    kOff   = 0,
    kLoose = 1,
    kTight = 2,
};

}  // namespace odyx
