// ===========================================================================
// ODYX :: VIO backend (GTSAM IncrementalFixedLagSmoother).
//
// Sliding window of keyframe states {Pose3 X, Vector3 V, imuBias B}, plus two
// persistent global states: extrinsics E (T_bc, Pose3) and td D (double).
// Factors: CombinedImuFactor between keyframes, visual ExpressionFactors with
// online td+extrinsics (Huber-robust), bias prior chain via CombinedImuFactor,
// priors on the first state, optional GNSS position factors, optional priors
// pinning E/D when online calibration is disabled.
//
// Globals are re-stamped to the newest keyframe time every update so the
// fixed-lag marginalizer never drops them.
// ===========================================================================
#pragma once
#include "common/odyx_types.h"
#include "common/config.h"
#include "common/snapshot.h"
#include "frontend/feature_tracker.h"
#include "backend/preintegrator.h"
#include "init/initializer.h"
#include <gtsam_unstable/nonlinear/IncrementalFixedLagSmoother.h>
#include <gtsam/navigation/NavState.h>
#include <gtsam/geometry/Pose3.h>
#include <map>
#include <memory>
#include <mutex>

namespace odyx {

struct KeyframeMeta {
    uint64_t idx = 0;
    TimeNs t_ns = 0;
    NavStateOut state;          // latest estimate
    std::vector<FeatureObs> obs;
};

struct LandmarkTrack {
    uint64_t id = 0;
    bool in_graph = false;
    Vec3 p_w = Vec3::Zero();
    float cov_trace = 1.f;
    // observations pending or active: keyframe idx -> (undist obs, image vel)
    std::map<uint64_t, std::pair<Vec2, Vec2>> obs;
    uint64_t last_kf = 0;
};

class VioBackend {
public:
    explicit VioBackend(const Config& cfg);

    // Seed the smoother from a successful initialization. imu_between[i] is the
    // IMU preintegration linking boot_kfs[i] -> boot_kfs[i+1] (size N-1); these
    // CombinedImuFactors constrain the bootstrap velocities/biases (without them
    // the initial system is indeterminate).
    void initializeFromResult(const InitResult& r,
                              const std::vector<KeyframeMeta>& boot_kfs,
                              const std::vector<std::shared_ptr<Preintegrator>>& imu_between);

    // Add a new keyframe. preint = preintegration since previous keyframe.
    // imgvel maps feature id -> normalized image-plane velocity (for td).
    // Returns true if the optimizer step was healthy.
    bool addKeyframe(const TrackResult& tr,
                     std::shared_ptr<Preintegrator> preint,
                     const std::map<uint64_t, Vec2>& imgvel,
                     double& optimize_ms);

    // Loose GNSS: add a position factor on the latest body pose (world frame),
    // given the GNSS antenna position expressed in the local world frame and
    // an isotropic std (m).
    void addGnssPositionFactor(uint64_t kf_idx, const Vec3& p_world, double std_m);

    NavStateOut current() const;
    bool diverged() const { return diverged_; }
    int windowSize() const;
    int landmarkCount() const;
    double tdEstimate() const { return td_est_; }
    Vec3 gyroBias() const { return cur_.bg; }
    Vec3 accelBias() const { return cur_.ba; }
    Vec3 velocity() const { return cur_.v_w; }

    // Snapshot of keyframe poses + landmarks (under lock).
    void snapshot(std::vector<KeyframeMeta>& kfs, std::vector<LandmarkOut>& lms) const;

    // Loop closure correction: apply a SE3 correction & relinearize (called by
    // loop thread through the Estimator under the backend lock).
    void applyPoseGraphCorrection(const std::map<uint64_t, gtsam::Pose3>& corrected);

    void reset();
    uint64_t latestKeyframeIdx() const { return kf_idx_; }

private:
    void triangulatePending();
    void extractCurrent();
    bool optimize(double& ms);

    Config cfg_;
    mutable std::mutex mtx_;

    std::unique_ptr<gtsam::IncrementalFixedLagSmoother> smoother_;
    gtsam::NonlinearFactorGraph new_factors_;
    gtsam::Values new_values_;
    gtsam::FixedLagSmoother::KeyTimestampMap new_stamps_;

    std::map<uint64_t, KeyframeMeta> kfs_;     // idx -> meta
    std::map<uint64_t, LandmarkTrack> lms_;    // id -> track
    uint64_t kf_idx_ = 0;
    TimeNs prev_kf_t_ = 0;

    NavStateOut cur_;
    double td_est_ = 0.0;
    bool initialized_ = false;
    bool diverged_ = false;
    int divergence_streak_ = 0;

    double lag_seconds() const;   // window_keyframes * nominal kf dt
};

}  // namespace odyx
