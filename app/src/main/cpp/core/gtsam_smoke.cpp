// ===========================================================================
// M0 smoke test: a tiny GTSAM optimization, callable from Kotlin via JNI.
// Builds a 2-pose factor graph (prior at origin + BetweenFactor of +1m in x)
// and returns the optimized x-translation of pose 1 (expected ~1.0).
// Exercises the GTSAM cross-compile end-to-end before any SLAM runs.
// ===========================================================================
#include <jni.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/inference/Symbol.h>
#include "common/log.h"

using namespace gtsam;
using symbol_shorthand::X;

extern "C"
JNIEXPORT jdouble JNICALL
Java_ai_deepmost_odyx_jni_NativeBridge_nativeGtsamSmoke(JNIEnv*, jobject) {
    try {
        NonlinearFactorGraph g;
        auto pn = noiseModel::Isotropic::Sigma(6, 1e-3);
        auto bn = noiseModel::Isotropic::Sigma(6, 1e-2);
        g.addPrior(X(0), Pose3(), pn);
        g.add(BetweenFactor<Pose3>(X(0), X(1), Pose3(Rot3(), Point3(1,0,0)), bn));
        Values init;
        init.insert(X(0), Pose3());
        init.insert(X(1), Pose3(Rot3(), Point3(0.5, 0.1, -0.1)));
        Values res = LevenbergMarquardtOptimizer(g, init).optimize();
        double x = res.at<Pose3>(X(1)).translation().x();
        ODYXI("smoke", "GTSAM smoke pose1.x = %.4f (expect ~1.0)", x);
        return x;
    } catch (const std::exception& e) {
        ODYXE("smoke", "GTSAM smoke threw: %s", e.what());
        return -999.0;
    }
}
