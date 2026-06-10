#include "backend/preintegrator.h"
#include "common/math_util.h"

using gtsam::PreintegrationCombinedParams;
using gtsam::PreintegratedCombinedMeasurements;
using gtsam::imuBias::ConstantBias;

namespace odyx {

boost::shared_ptr<PreintegrationCombinedParams>
Preintegrator::makeParams(const ImuNoise& n) {
    // Z-up gravity convention for the navigation frame (n_gravity = (0,0,-g)).
    auto p = PreintegrationCombinedParams::MakeSharedU(n.g);
    const double a2 = n.acc_n * n.acc_n;
    const double g2 = n.gyr_n * n.gyr_n;
    const double aw2 = n.acc_w * n.acc_w;
    const double gw2 = n.gyr_w * n.gyr_w;
    p->accelerometerCovariance = Mat3::Identity() * a2;
    p->gyroscopeCovariance     = Mat3::Identity() * g2;
    p->biasAccCovariance       = Mat3::Identity() * aw2;
    p->biasOmegaCovariance     = Mat3::Identity() * gw2;
    p->integrationCovariance   = Mat3::Identity() * 1e-8;
    p->biasAccOmegaInt         = Eigen::Matrix<double,6,6>::Identity() * 1e-5;
    return p;
}

Preintegrator::Preintegrator(const ImuNoise& n, const Vec3& ba0, const Vec3& bg0)
    : noise_(n) {
    auto params = makeParams(n);
    ConstantBias bias(ba0, bg0);
    pim_ = std::make_shared<PreintegratedCombinedMeasurements>(params, bias);
}

void Preintegrator::integrate(const Vec3& acc, const Vec3& gyr, double dt) {
    if (dt <= 0) return;
    pim_->integrateMeasurement(acc, gyr, dt);
}

void Preintegrator::integrateSamples(const std::vector<ImuSample>& s) {
    for (size_t i = 1; i < s.size(); ++i) {
        const double dt = ns_to_s(s[i].t_ns - s[i-1].t_ns);
        if (dt <= 0 || dt > 0.5) continue;       // guard against gaps
        // midpoint sample for the interval
        const Vec3 a = 0.5 * (s[i].acc + s[i-1].acc);
        const Vec3 g = 0.5 * (s[i].gyr + s[i-1].gyr);
        pim_->integrateMeasurement(a, g, dt);
    }
}

Mat3 Preintegrator::dR_dbg() const {
    // GTSAM exposes delRdelBiasOmega() (3x3).
    return pim_->preintegrated_H_biasOmega().topRows<3>();
}

Mat3 Preintegrator::dR_biased(const Vec3& bg) const {
    const Vec3 dbg = bg - pim_->biasHat().gyroscope();
    return (pim_->deltaRij().matrix() * expSO3(dR_dbg() * dbg));
}

Vec3 Preintegrator::dp_biased(const Vec3& ba, const Vec3& bg) const {
    const Vec3 dba = ba - pim_->biasHat().accelerometer();
    const Vec3 dbg = bg - pim_->biasHat().gyroscope();
    // first-order correction via stored jacobians
    const auto Hba = pim_->preintegrated_H_biasAcc();   // 9x3
    const auto Hbg = pim_->preintegrated_H_biasOmega();  // 9x3
    Vec3 dp = pim_->deltaPij();
    dp += Hba.block<3,3>(3,0) * dba + Hbg.block<3,3>(3,0) * dbg;
    return dp;
}

Vec3 Preintegrator::dv_biased(const Vec3& ba, const Vec3& bg) const {
    const Vec3 dba = ba - pim_->biasHat().accelerometer();
    const Vec3 dbg = bg - pim_->biasHat().gyroscope();
    const auto Hba = pim_->preintegrated_H_biasAcc();
    const auto Hbg = pim_->preintegrated_H_biasOmega();
    Vec3 dv = pim_->deltaVij();
    dv += Hba.block<3,3>(6,0) * dba + Hbg.block<3,3>(6,0) * dbg;
    return dv;
}

void Preintegrator::resetWithBias(const Vec3& ba, const Vec3& bg) {
    pim_->resetIntegrationAndSetBias(ConstantBias(ba, bg));
}

}  // namespace odyx
