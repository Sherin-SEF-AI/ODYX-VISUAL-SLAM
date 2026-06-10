#include "gnss/gnss_fusion.h"
#include "common/math_util.h"
#include "common/log.h"
#include <Eigen/Dense>

namespace odyx {

Mat3 EnuTransform::R() const {
    const double c = std::cos(yaw), s = std::sin(yaw);
    Mat3 m; m << c,-s,0, s,c,0, 0,0,1; return m;
}
Vec3 EnuTransform::localToEnu(const Vec3& p) const { return R()*p + t; }
Vec3 EnuTransform::enuToLocal(const Vec3& p) const { return R().transpose()*(p - t); }

void EnuAligner::setOrigin(const GnssFix& fix) {
    lat0_ = fix.lat_deg; lon0_ = fix.lon_deg; alt0_ = fix.alt_m;
    ecef0_ = lla2ecef(lat0_, lon0_, alt0_);
    has_origin_ = true;
    ODYXI("gnss", "ENU origin set @ %.7f,%.7f,%.1f", lat0_, lon0_, alt0_);
}

bool EnuAligner::originLla(double& lat, double& lon, double& alt) const {
    if (!has_origin_) return false;
    lat = lat0_; lon = lon0_; alt = alt0_; return true;
}

Vec3 EnuAligner::fixToEnu(const GnssFix& fix) const {
    if (!has_origin_) return Vec3::Zero();
    Vec3 ecef = lla2ecef(fix.lat_deg, fix.lon_deg, fix.has_alt ? fix.alt_m : alt0_);
    return ecef2enu(ecef, lat0_, lon0_, ecef0_);
}

void EnuAligner::addCorrespondence(const Vec3& p_local, const Vec3& p_enu, double w) {
    if (holding_) return;
    corr_.push_back({p_local, p_enu, w});
    while (corr_.size() > 60) corr_.pop_front();
    if (corr_.size() >= 4) { coarse(); fine(); }
}

// Closed-form: with z aligned by gravity, solve planar yaw + 3D translation by
// Procrustes on the (x,y) components (weighted), z by weighted mean offset.
void EnuAligner::coarse() {
    double W = 0; Vec3 ml = Vec3::Zero(), me = Vec3::Zero();
    for (auto& c : corr_) { W += c.w; ml += c.w*c.l; me += c.w*c.e; }
    if (W < 1e-9) return;
    ml /= W; me /= W;
    double Sxx=0, Sxy=0;     // 2D cross-covariance for yaw
    for (auto& c : corr_) {
        Vec2 l(c.l.x()-ml.x(), c.l.y()-ml.y());
        Vec2 e(c.e.x()-me.x(), c.e.y()-me.y());
        Sxx += c.w*(l.x()*e.x() + l.y()*e.y());
        Sxy += c.w*(l.x()*e.y() - l.y()*e.x());
    }
    double yaw = std::atan2(Sxy, Sxx);
    EnuTransform T; T.yaw = yaw;
    T.t = me - T.R()*ml;
    T.valid = true;
    T_ = T;
}

// Gauss-Newton refine over [yaw, tx, ty, tz] minimizing weighted residuals.
void EnuAligner::fine() {
    if (corr_.size() < 4) return;
    double yaw = T_.yaw; Vec3 t = T_.t;
    for (int it = 0; it < 8; ++it) {
        Eigen::Matrix4d H = Eigen::Matrix4d::Zero();
        Eigen::Vector4d g = Eigen::Vector4d::Zero();
        double c = std::cos(yaw), s = std::sin(yaw);
        Mat3 R; R << c,-s,0, s,c,0, 0,0,1;
        Mat3 dR; dR << -s,-c,0, c,-s,0, 0,0,0;   // dR/dyaw
        double cost = 0;
        for (auto& cc : corr_) {
            Vec3 r = R*cc.l + t - cc.e;
            Eigen::Matrix<double,3,4> J;
            J.col(0) = dR*cc.l;       // d/dyaw
            J.block<3,3>(0,1) = Mat3::Identity();   // d/dt
            H += cc.w * J.transpose()*J;
            g += cc.w * J.transpose()*r;
            cost += cc.w * r.squaredNorm();
        }
        H += 1e-9*Eigen::Matrix4d::Identity();
        Eigen::Vector4d dx = H.ldlt().solve(-g);
        yaw += dx(0); t += dx.tail<3>();
        if (dx.norm() < 1e-7) break;
        (void)cost;
    }
    // residual rms
    double c = std::cos(yaw), s = std::sin(yaw);
    Mat3 R; R << c,-s,0, s,c,0, 0,0,1;
    double sum=0; double W=0;
    for (auto& cc : corr_) { sum += cc.w*(R*cc.l + t - cc.e).squaredNorm(); W+=cc.w; }
    T_.yaw = yaw; T_.t = t; T_.valid = true;
    T_.resid_m = W>0 ? std::sqrt(sum/W) : 0;
}

void EnuAligner::reset() {
    has_origin_ = false; holding_ = false; corr_.clear(); T_ = EnuTransform();
}

}  // namespace odyx
