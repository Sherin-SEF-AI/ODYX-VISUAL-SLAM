// ===========================================================================
// ODYX :: visual factor with ONLINE td + extrinsic calibration.
//
// Built with GTSAM Expressions so Jacobians are auto-derived and the code is
// robust across the GTSAM 4.2/4.3 evaluateError signature change.
//
// Model (normalized/undistorted image coords, identity calibration):
//   p_body   = transformTo(T_wb, L)          // world point into body
//   p_cam    = transformTo(T_bc, p_body)     // body into camera
//   pred     = projN(p_cam)                  // (x/z, y/z), see projN below
//   h(x)     = pred + td * vel               // td shifts obs by image-plane vel
//   residual = h(x) - obs
//
// Keys: X(T_wb : Pose3), L(landmark : Point3), E(T_bc : Pose3), D(td : double).
// When online td / extrinsics are disabled, strong priors pin D and E.
// ===========================================================================
#pragma once
#include <gtsam/nonlinear/ExpressionFactor.h>
#include <gtsam/nonlinear/expressions.h>
#include <gtsam/slam/expressions.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Point3.h>
#include <cmath>

namespace odyx {

// Unary expression: pinhole normalized projection (x/z, y/z) of a camera-frame
// point, with analytic Jacobian. Self-contained so we don't depend on the exact
// name/signature of GTSAM's built-in projection expression helper.
inline gtsam::Point2_ projN(const gtsam::Point3_& pc) {
    auto f = [](const gtsam::Point3& p, gtsam::OptionalJacobian<2,3> H) -> gtsam::Point2 {
        const double z = (std::abs(p.z()) < 1e-6) ? 1e-6 : p.z();
        const double zi = 1.0 / z;
        if (H) {
            (*H) << zi, 0.0, -p.x() * zi * zi,
                    0.0, zi, -p.y() * zi * zi;
        }
        return gtsam::Point2(p.x() * zi, p.y() * zi);
    };
    return gtsam::Point2_(f, pc);
}

// Binary expression: h(pred, td) = pred + td * vel  (vel constant).
// Implemented explicitly (no reliance on an operator+ overload for Point2_).
inline gtsam::Point2_ addTd(const gtsam::Point2_& pred, const gtsam::Double_& td,
                            const gtsam::Point2& vel) {
    auto f = [vel](const gtsam::Point2& p, const double& t,
                   gtsam::OptionalJacobian<2,2> Hp,
                   gtsam::OptionalJacobian<2,1> Ht) -> gtsam::Point2 {
        if (Hp) *Hp = gtsam::Matrix22::Identity();
        if (Ht) { (*Ht)(0,0) = vel.x(); (*Ht)(1,0) = vel.y(); }
        return gtsam::Point2(p.x() + t * vel.x(), p.y() + t * vel.y());
    };
    return gtsam::Point2_(f, pred, td);
}

// Build the visual ExpressionFactor.
//   obs  : undistorted normalized observation (x/z,y/z)
//   vel  : image-plane velocity of the feature in normalized coords (per second)
inline gtsam::ExpressionFactor<gtsam::Point2>
makeVisualFactor(gtsam::Key xKey, gtsam::Key lKey, gtsam::Key eKey, gtsam::Key dKey,
                 const gtsam::Point2& obs, const gtsam::Point2& vel,
                 const gtsam::SharedNoiseModel& noise) {
    using namespace gtsam;
    Pose3_  Twb(xKey);
    Point3_ L(lKey);
    Pose3_  Tbc(eKey);
    Double_ td(dKey);

    Point3_ p_body = transformTo(Twb, L);
    Point3_ p_cam  = transformTo(Tbc, p_body);
    Point2_ pred   = projN(p_cam);
    Point2_ h      = addTd(pred, td, vel);
    return ExpressionFactor<Point2>(noise, obs, h);
}

}  // namespace odyx
