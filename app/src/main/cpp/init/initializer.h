// ===========================================================================
// ODYX :: visual-inertial initializer (VINS-style, reimplemented).
//
// Pipeline (from published methods, NOT copied code):
//   1. Bootstrap window of keyframes with tracked features.
//   2. Vision-only up-to-scale SfM: relative pose from the two frames with the
//      largest parallax (essential matrix), triangulate, PnP the rest, small BA.
//   3. Gyro bias calibration from rotation constraints (preintegration vs SfM).
//   4. Linear visual-inertial alignment: solve per-frame velocity, gravity
//      vector, and metric SCALE from the preintegration <-> SfM translation
//      constraints.
//   5. Gravity refinement on the 2-DoF tangent (fixed |g|), recover R_wl that
//      aligns the local frame to gravity.
//
// Gated on genuine translational excitation: rejects pure-rotation bootstraps.
// ===========================================================================
#pragma once
#include "common/odyx_types.h"
#include "common/config.h"
#include "frontend/feature_tracker.h"
#include "backend/preintegrator.h"
#include <map>
#include <vector>

namespace odyx {

// One bootstrap keyframe: SfM pose (up-to-scale) + its preintegration from prev.
struct BootKf {
    TimeNs t_ns = 0;
    Mat3 R_c0_ck = Mat3::Identity();    // SfM rotation (cam0 <- camk), no scale on R
    Vec3 t_c0_ck = Vec3::Zero();        // SfM translation, arbitrary scale
    bool pose_set = false;
    std::shared_ptr<Preintegrator> preint;  // from previous kf to this one
    std::vector<FeatureObs> obs;        // undistorted obs at this kf
};

struct InitResult {
    bool ok = false;
    InitState state = InitState::kCollecting;
    double scale = 1.0;
    Vec3 gravity_l = Vec3(0,0,-9.81);   // gravity in the initial local frame
    Mat3 R_wl = Mat3::Identity();       // world(gravity-aligned) <- local
    Vec3 bg = Vec3::Zero();
    Vec3 ba = Vec3::Zero();
    std::vector<Vec3> vel_b;            // per-bootstrap-kf body velocity (world)
    std::vector<NavStateOut> states;    // metric, gravity-aligned states per kf
    std::map<uint64_t, Vec3> landmarks; // triangulated, metric, world frame
    double gravity_resid = 0;
    std::string reason;
};

class Initializer {
public:
    explicit Initializer(const Config& cfg) : cfg_(cfg) {}

    // Feed a keyframe (with the preintegration since the previous keyframe).
    // Returns current InitResult; ok==true when initialization succeeded.
    InitResult addKeyframe(const BootKf& kf);
    void reset() { kfs_.clear(); }
    size_t size() const { return kfs_.size(); }

private:
    bool visualSfm(InitResult& r);
    bool calibrateGyroBias();
    bool linearAlign(InitResult& r);
    void refineGravity(InitResult& r);
    double avgParallax(size_t i, size_t j) const;

    // Parallax + #common features between two bootstrap keyframes.
    void parallaxCommon(size_t i, size_t j, double& par_px, int& common) const;
    // Pick the widest-baseline reference frame vs the latest that still shares
    // enough features AND has enough parallax. Returns false if none (no motion).
    bool selectReference(size_t& ref) const;

    Config cfg_;
    std::vector<BootKf> kfs_;
    Vec3 bg_ = Vec3::Zero();   // gyro bias from calibrateGyroBias()
    size_t ref_idx_ = 0;       // SfM reference frame chosen by selectReference()
    mutable double last_best_par_ = 0;     // diagnostics from selectReference()
    mutable int last_best_common_ = 0;
    static constexpr size_t kMinKf = 6;
    static constexpr int kMinCommon = 12;
    static constexpr double kMinAvgParallaxPx = 8.0;
};

}  // namespace odyx
