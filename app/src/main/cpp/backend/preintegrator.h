// ===========================================================================
// ODYX :: IMU preintegration wrapper (GTSAM CombinedImuFactor).
//
// Wraps gtsam::PreintegratedCombinedMeasurements so the rest of the estimator
// stays GTSAM-agnostic where convenient, and exposes the raw delta quantities
// (dR, dp, dv, dt) the initializer needs for gyro-bias calibration and the
// linear visual-inertial alignment.
// ===========================================================================
#pragma once
#include "common/odyx_types.h"
#include "common/config.h"
#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/navigation/ImuBias.h>
#include <memory>

namespace odyx {

class Preintegrator {
public:
    Preintegrator(const ImuNoise& n, const Vec3& ba0, const Vec3& bg0);

    void integrate(const Vec3& acc, const Vec3& gyr, double dt);
    void integrateSamples(const std::vector<ImuSample>& s);  // uses sample dt

    double dt() const { return pim_->deltaTij(); }
    Mat3 dR() const { return pim_->deltaRij().matrix(); }
    Vec3 dp() const { return pim_->deltaPij(); }
    Vec3 dv() const { return pim_->deltaVij(); }

    // Jacobian of dR wrt gyro bias (for gyro bias calibration).
    Mat3 dR_dbg() const;

    // Re-evaluate delta quantities for a new bias (first-order repropagation).
    Mat3 dR_biased(const Vec3& bg) const;
    Vec3 dp_biased(const Vec3& ba, const Vec3& bg) const;
    Vec3 dv_biased(const Vec3& ba, const Vec3& bg) const;

    void resetWithBias(const Vec3& ba, const Vec3& bg);

    const gtsam::PreintegratedCombinedMeasurements& pim() const { return *pim_; }
    gtsam::imuBias::ConstantBias bias() const { return pim_->biasHat(); }

    // GTSAM 4.2 uses boost::shared_ptr for preintegration params/factors.
    static boost::shared_ptr<gtsam::PreintegrationCombinedParams>
        makeParams(const ImuNoise& n);

private:
    std::shared_ptr<gtsam::PreintegratedCombinedMeasurements> pim_;
    ImuNoise noise_;
};

}  // namespace odyx
