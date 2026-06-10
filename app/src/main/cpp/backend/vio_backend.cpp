#include "backend/vio_backend.h"
#include "backend/factors.h"
#include "common/math_util.h"
#include "common/log.h"
#include <Eigen/Dense>

#include <gtsam/nonlinear/PriorFactor.h>
#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/navigation/GPSFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/linear/NoiseModel.h>
#include <gtsam/inference/Symbol.h>
#include <chrono>

using gtsam::symbol_shorthand::X;   // pose   T_wb
using gtsam::symbol_shorthand::V;   // velocity
using gtsam::symbol_shorthand::B;   // imu bias
using gtsam::symbol_shorthand::L;   // landmark
using gtsam::Pose3; using gtsam::Rot3; using gtsam::Point3; using gtsam::Point2;
using gtsam::Vector3; using gtsam::NavState; using gtsam::Values;
using gtsam::noiseModel::Isotropic; using gtsam::noiseModel::Diagonal;

namespace odyx {

static const gtsam::Key kE = gtsam::Symbol('e', 0);   // extrinsics T_bc
static const gtsam::Key kD = gtsam::Symbol('d', 0);   // td (double)

static Pose3 toPose3(const NavStateOut& s) {
    return Pose3(Rot3(s.q_wb), Point3(s.p_wb));
}

VioBackend::VioBackend(const Config& cfg) : cfg_(cfg) {}

double VioBackend::lag_seconds() const {
    return cfg_.window_keyframes * cfg_.kf_max_dt_s + 1.0;
}

void VioBackend::reset() {
    std::lock_guard<std::mutex> lk(mtx_);
    smoother_.reset();
    new_factors_ = gtsam::NonlinearFactorGraph();
    new_values_.clear();
    new_stamps_.clear();
    kfs_.clear(); lms_.clear();
    kf_idx_ = 0; prev_kf_t_ = 0;
    cur_ = NavStateOut();
    initialized_ = false; diverged_ = false; divergence_streak_ = 0;
}

void VioBackend::initializeFromResult(const InitResult& r,
                                      const std::vector<KeyframeMeta>& boot_kfs,
                                      const std::vector<std::shared_ptr<Preintegrator>>& imu_between) {
    std::lock_guard<std::mutex> lk(mtx_);
    gtsam::ISAM2Params params;
    params.relinearizeThreshold = 0.01;
    params.relinearizeSkip = 1;
    params.findUnusedFactorSlots = true;
    smoother_ = std::make_unique<gtsam::IncrementalFixedLagSmoother>(lag_seconds(), params);

    new_factors_ = gtsam::NonlinearFactorGraph();
    new_values_.clear(); new_stamps_.clear();

    // Persistent globals: extrinsics + td.
    Pose3 Tbc(Rot3(cfg_.extr.q_bc), Point3(cfg_.extr.t_bc));
    new_values_.insert(kE, Tbc);
    new_values_.insert(kD, cfg_.extr.td);
    td_est_ = cfg_.extr.td;

    // Priors pin extrinsics / td either softly (online) or hard (disabled).
    {
        auto en = cfg_.online_extr
            ? Diagonal::Sigmas((gtsam::Vector(6) << 0.02,0.02,0.02, 0.03,0.03,0.03).finished())
            : Diagonal::Sigmas((gtsam::Vector(6) << 1e-4,1e-4,1e-4, 1e-4,1e-4,1e-4).finished());
        new_factors_.addPrior(kE, Tbc, en);
        // td: camera + IMU share the REALTIME clock, so the residual offset is
        // small (sensor latency, ~10-30 ms). Keep the prior TIGHT so the online
        // estimate can't run away (it was swinging to +-0.27 s and destabilizing
        // the IMU alignment).
        auto dn = Isotropic::Sigma(1, cfg_.online_td ? 0.005 : 1e-5);
        new_factors_.addPrior(kD, cfg_.extr.td, dn);
    }

    // Seed boot keyframes as window states.
    const auto& S = r.states;
    for (size_t i = 0; i < boot_kfs.size() && i < S.size(); ++i) {
        const auto& st = S[i];
        uint64_t idx = boot_kfs[i].idx;
        Pose3 pose = toPose3(st);
        new_values_.insert(X(idx), pose);
        new_values_.insert(V(idx), Vector3(st.v_w));
        new_values_.insert(B(idx), gtsam::imuBias::ConstantBias(st.ba, st.bg));
        new_stamps_[X(idx)] = ns_to_s(st.t_ns);
        new_stamps_[V(idx)] = ns_to_s(st.t_ns);
        new_stamps_[B(idx)] = ns_to_s(st.t_ns);

        KeyframeMeta km; km.idx = idx; km.t_ns = st.t_ns; km.state = st;
        km.obs = boot_kfs[i].obs;
        kfs_[idx] = km;
        kf_idx_ = std::max(kf_idx_, idx);
        prev_kf_t_ = st.t_ns;
    }
    new_stamps_[kE] = ns_to_s(prev_kf_t_);
    new_stamps_[kD] = ns_to_s(prev_kf_t_);

    // IMU factors between consecutive bootstrap keyframes — constrain the
    // velocities/biases (otherwise the initial linear system is indeterminate).
    const size_t nseed = std::min(boot_kfs.size(), S.size());
    for (size_t i = 0; i + 1 < nseed; ++i) {
        if (i >= imu_between.size() || !imu_between[i]) continue;
        new_factors_.add(gtsam::CombinedImuFactor(
            X(boot_kfs[i].idx),   V(boot_kfs[i].idx),
            X(boot_kfs[i+1].idx), V(boot_kfs[i+1].idx),
            B(boot_kfs[i].idx),   B(boot_kfs[i+1].idx),
            imu_between[i]->pim()));
    }

    // Strong prior on the first state to fix the gauge.
    {
        const auto& st0 = S.front();
        uint64_t idx0 = boot_kfs.front().idx;
        auto pn = Diagonal::Sigmas((gtsam::Vector(6) << 1e-3,1e-3,1e-3, 1e-2,1e-2,1e-2).finished());
        new_factors_.addPrior(X(idx0), toPose3(st0), pn);
        new_factors_.addPrior(V(idx0), Vector3(st0.v_w), Isotropic::Sigma(3, 0.1));
        new_factors_.addPrior(B(idx0),
            gtsam::imuBias::ConstantBias(st0.ba, st0.bg), Isotropic::Sigma(6, 0.05));
    }

    // Seed landmarks from init.
    const double sigpx = cfg_.huber_px / std::max(1.0, cfg_.cam.fx);
    auto base = Isotropic::Sigma(2, sigpx);
    auto robust = gtsam::noiseModel::Robust::Create(
        gtsam::noiseModel::mEstimator::Huber::Create(1.345), base);
    for (const auto& kv : r.landmarks) {
        uint64_t lid = kv.first;
        LandmarkTrack t; t.id = lid; t.p_w = kv.second; t.in_graph = true;
        new_values_.insert(L(lid), Point3(kv.second));
        // add observations from boot kfs
        for (const auto& bk : boot_kfs) {
            for (const auto& o : bk.obs) {
                if (o.id != lid) continue;
                Point2 obs(o.undist.x(), o.undist.y());
                new_factors_.add(makeVisualFactor(X(bk.idx), L(lid), kE, kD,
                                                  obs, Point2(0,0), robust));
                t.obs[bk.idx] = {o.undist, Vec2::Zero()};
                t.last_kf = std::max(t.last_kf, bk.idx);
            }
        }
        new_stamps_[L(lid)] = ns_to_s(prev_kf_t_);
        lms_[lid] = t;
    }

    double ms; optimize(ms);
    extractCurrent();
    initialized_ = true;
    ODYXI("backend", "initialized: %zu kfs, %zu landmarks, td=%.4f",
          kfs_.size(), lms_.size(), td_est_);
}

bool VioBackend::optimize(double& ms) {
    using clock = std::chrono::steady_clock;
    auto t0 = clock::now();
    try {
        smoother_->update(new_factors_, new_values_, new_stamps_);
        // a couple of extra iterations for the nonlinear visual factors
        smoother_->update();
    } catch (const std::exception& e) {
        ODYXE("backend", "smoother update threw: %s", e.what());
        new_factors_ = gtsam::NonlinearFactorGraph();
        new_values_.clear(); new_stamps_.clear();
        // Single failures are recovered by rolling back the keyframe (caller);
        // only declare true divergence after several CONSECUTIVE failures.
        ++divergence_streak_;
        if (divergence_streak_ >= 5) diverged_ = true;
        return false;
    }
    new_factors_ = gtsam::NonlinearFactorGraph();
    new_values_.clear(); new_stamps_.clear();
    ms = std::chrono::duration<double, std::milli>(clock::now() - t0).count();
    return true;
}

void VioBackend::extractCurrent() {
    if (!smoother_) return;
    Values est = smoother_->calculateEstimate();
    if (!est.exists(X(kf_idx_))) return;
    Pose3 p = est.at<Pose3>(X(kf_idx_));
    Vector3 v = est.at<Vector3>(V(kf_idx_));
    auto b = est.at<gtsam::imuBias::ConstantBias>(B(kf_idx_));
    cur_.t_ns = kfs_[kf_idx_].t_ns;
    cur_.q_wb = Quat(p.rotation().matrix());
    cur_.p_wb = p.translation();
    cur_.v_w = v;
    cur_.ba = b.accelerometer();
    cur_.bg = b.gyroscope();
    cur_.valid = !diverged_;
    if (est.exists(kD)) td_est_ = std::max(-0.05, std::min(0.05, est.at<double>(kD)));  // clamp ±50 ms
    if (cfg_.online_extr && est.exists(kE)) {
        Pose3 e = est.at<Pose3>(kE);
        cfg_.extr.q_bc = Quat(e.rotation().matrix());
        cfg_.extr.t_bc = e.translation();
    }
    // covariance (via the smoother's underlying ISAM2)
    try {
        auto m = smoother_->getISAM2().marginalCovariance(X(kf_idx_));
        cur_.pose_cov = m;
        if (!m.allFinite()) { diverged_ = true; cur_.valid = false; }
    } catch (...) {}
    // sync keyframe metadata estimate
    for (auto& kv : kfs_) {
        if (!est.exists(X(kv.first))) continue;
        Pose3 pp = est.at<Pose3>(X(kv.first));
        kv.second.state.q_wb = Quat(pp.rotation().matrix());
        kv.second.state.p_wb = pp.translation();
        kv.second.state.valid = true;
    }
    for (auto& kv : lms_) {
        if (est.exists(L(kv.first))) kv.second.p_w = est.at<Point3>(L(kv.first));
    }
    if (cur_.pose_cov.allFinite() && cur_.p_wb.allFinite()) divergence_streak_ = 0;
}

bool VioBackend::addKeyframe(const TrackResult& tr,
                             std::shared_ptr<Preintegrator> preint,
                             const std::map<uint64_t, Vec2>& imgvel,
                             double& optimize_ms) {
    std::lock_guard<std::mutex> lk(mtx_);
    if (!initialized_ || !smoother_) return false;

    const uint64_t prev = kf_idx_;
    const uint64_t idx = ++kf_idx_;
    const TimeNs t = tr.t_ns;

    // --- IMU prediction for the new state ---
    NavState prevNav(toPose3(kfs_[prev].state),
                     Vector3(kfs_[prev].state.v_w));
    gtsam::imuBias::ConstantBias prevBias(kfs_[prev].state.ba, kfs_[prev].state.bg);
    NavState pred = preint->pim().predict(prevNav, prevBias);

    new_values_.insert(X(idx), pred.pose());
    new_values_.insert(V(idx), pred.velocity());
    new_values_.insert(B(idx), prevBias);
    new_stamps_[X(idx)] = ns_to_s(t);
    new_stamps_[V(idx)] = ns_to_s(t);
    new_stamps_[B(idx)] = ns_to_s(t);

    // --- IMU factor between prev and new ---
    new_factors_.add(gtsam::CombinedImuFactor(
        X(prev), V(prev), X(idx), V(idx), B(prev), B(idx), preint->pim()));

    // --- bookkeeping for this keyframe ---
    KeyframeMeta km; km.idx = idx; km.t_ns = t;
    km.state.t_ns = t;
    km.state.q_wb = Quat(pred.pose().rotation().matrix());
    km.state.p_wb = pred.pose().translation();
    km.state.v_w = pred.velocity();
    km.state.ba = prevBias.accelerometer();
    km.state.bg = prevBias.gyroscope();
    km.obs = tr.obs;
    kfs_[idx] = km;

    // --- accumulate observations into landmark tracks ---
    const double lag = lag_seconds();
    const double sigpx = cfg_.huber_px / std::max(1.0, cfg_.cam.fx);
    auto base = Isotropic::Sigma(2, sigpx);
    auto robust = gtsam::noiseModel::Robust::Create(
        gtsam::noiseModel::mEstimator::Huber::Create(1.345), base);

    for (const auto& o : tr.obs) {
        Vec2 vel = Vec2::Zero();
        auto vit = imgvel.find(o.id);
        if (vit != imgvel.end()) vel = vit->second;
        auto& trk = lms_[o.id];
        trk.id = o.id;
        trk.obs[idx] = {o.undist, vel};
        trk.last_kf = idx;
        if (trk.in_graph) {
            new_factors_.add(makeVisualFactor(X(idx), L(o.id), kE, kD,
                Point2(o.undist.x(), o.undist.y()), Point2(vel.x(), vel.y()), robust));
            new_stamps_[L(o.id)] = ns_to_s(t);
        }
    }

    // --- triangulate landmarks that have enough parallax & observations ---
    triangulatePending();

    // re-stamp globals so they survive marginalization
    new_stamps_[kE] = ns_to_s(t);
    new_stamps_[kD] = ns_to_s(t);

    prev_kf_t_ = t;

    bool ok = optimize(optimize_ms);
    if (ok) {
        extractCurrent();
    } else {
        // The smoother update threw: this keyframe's states/factors were
        // discarded by optimize(), but kfs_/lms_ were already updated. Roll the
        // keyframe back so the NEXT keyframe links to the last good state instead
        // of referencing a key the smoother never received (which would cascade).
        kfs_.erase(idx);
        for (auto& kv : lms_) kv.second.obs.erase(idx);
        kf_idx_ = prev;
        prev_kf_t_ = kfs_.count(prev) ? kfs_[prev].t_ns : prev_kf_t_;
        ODYXW("backend", "rolled back keyframe %llu after optimizer failure",
              (unsigned long long)idx);
    }

    // Prune keyframes that have left the smoother window. Their GTSAM states are
    // marginalized, so we must not add new factors referencing them; the
    // kfs_.count() checks in triangulatePending rely on this set being the live
    // window. (The UI trajectory is kept independently in MapStore.)
    for (auto it = kfs_.begin(); it != kfs_.end();) {
        if (it->first != kf_idx_ && ns_to_s(t) - ns_to_s(it->second.t_ns) > lag)
            it = kfs_.erase(it);
        else ++it;
    }

    // drop tracks whose last observation aged out of the window
    for (auto it = lms_.begin(); it != lms_.end();) {
        auto kf = kfs_.find(it->second.last_kf);
        if (kf == kfs_.end() || ns_to_s(t) - ns_to_s(kf->second.t_ns) > lag + 1.0)
            it = lms_.erase(it);
        else ++it;
    }
    // cap total landmarks
    while ((int)lms_.size() > cfg_.max_landmarks) {
        lms_.erase(lms_.begin());
    }
    return ok && !diverged_;
}

void VioBackend::triangulatePending() {
    const double sigpx = cfg_.huber_px / std::max(1.0, cfg_.cam.fx);
    auto base = Isotropic::Sigma(2, sigpx);
    auto robust = gtsam::noiseModel::Robust::Create(
        gtsam::noiseModel::mEstimator::Huber::Create(1.345), base);
    const Pose3 Tbc(Rot3(cfg_.extr.q_bc), Point3(cfg_.extr.t_bc));

    for (auto& kv : lms_) {
        LandmarkTrack& t = kv.second;
        if (t.in_graph || t.obs.size() < 2) continue;

        // pick the two observations with the largest baseline.
        std::vector<uint64_t> idxs;
        for (auto& ob : t.obs) if (kfs_.count(ob.first)) idxs.push_back(ob.first);
        if (idxs.size() < 2) continue;
        uint64_t a = idxs.front(), b = idxs.back();
        const auto& ka = kfs_[a].state; const auto& kb = kfs_[b].state;

        // camera poses
        Pose3 Twa = toPose3(ka) * Tbc;
        Pose3 Twb = toPose3(kb) * Tbc;
        // DLT triangulation in world
        Vec2 ua = t.obs[a].first, ub = t.obs[b].first;
        Eigen::Matrix<double,3,4> Pa, Pb;
        Pa = (Twa.inverse().matrix()).topRows<3>();
        Pb = (Twb.inverse().matrix()).topRows<3>();
        Eigen::Matrix4d A;
        A.row(0) = ua.x()*Pa.row(2) - Pa.row(0);
        A.row(1) = ua.y()*Pa.row(2) - Pa.row(1);
        A.row(2) = ub.x()*Pb.row(2) - Pb.row(0);
        A.row(3) = ub.y()*Pb.row(2) - Pb.row(1);
        Eigen::JacobiSVD<Eigen::Matrix4d> svd(A, Eigen::ComputeFullV);
        Vec4 X4 = svd.matrixV().col(3);
        if (std::abs(X4(3)) < 1e-9) continue;
        Vec3 Pw = X4.head<3>() / X4(3);
        // cheirality + range gate in both cameras
        Vec3 pca = Twa.inverse() * Pw, pcb = Twb.inverse() * Pw;
        if (pca.z() <= 0.2 || pcb.z() <= 0.2 || pca.z() > 80 || pcb.z() > 80) continue;
        // parallax gate
        Vec3 r1 = (Pw - Twa.translation()).normalized();
        Vec3 r2 = (Pw - Twb.translation()).normalized();
        if (r1.dot(r2) > 0.9998) continue;   // < ~1 deg parallax

        t.p_w = Pw; t.in_graph = true;
        new_values_.insert(L(t.id), Point3(Pw));
        for (auto& ob : t.obs) {
            if (!kfs_.count(ob.first)) continue;
            new_factors_.add(makeVisualFactor(X(ob.first), L(t.id), kE, kD,
                Point2(ob.second.first.x(), ob.second.first.y()),
                Point2(ob.second.second.x(), ob.second.second.y()), robust));
        }
        new_stamps_[L(t.id)] = ns_to_s(kfs_[t.last_kf].t_ns);
    }
}

void VioBackend::addGnssPositionFactor(uint64_t kf_idx, const Vec3& p_world, double std_m) {
    std::lock_guard<std::mutex> lk(mtx_);
    if (!smoother_ || !kfs_.count(kf_idx)) return;
    // position-only factor on the BODY translation. The caller (Estimator::
    // feedGnss) has already converted the GNSS antenna position to the body
    // position via the lever arm, so p_world here is the body position.
    auto n = Isotropic::Sigma(3, std::max(0.3, std_m));
    auto robust = gtsam::noiseModel::Robust::Create(
        gtsam::noiseModel::mEstimator::Huber::Create(1.0), n);
    new_factors_.add(gtsam::GPSFactor(X(kf_idx), Point3(p_world), robust));
}

void VioBackend::applyPoseGraphCorrection(const std::map<uint64_t, gtsam::Pose3>& corrected) {
    std::lock_guard<std::mutex> lk(mtx_);
    // Inject strong between-style priors nudging the corrected keyframes; the
    // next smoother update relinearizes the whole window consistently.
    auto n = Diagonal::Sigmas((gtsam::Vector(6) << 0.05,0.05,0.05, 0.1,0.1,0.1).finished());
    for (const auto& kv : corrected) {
        if (!kfs_.count(kv.first)) continue;
        new_factors_.addPrior(X(kv.first), kv.second, n);
        new_stamps_[X(kv.first)] = ns_to_s(kfs_[kv.first].t_ns);
    }
}

NavStateOut VioBackend::current() const {
    std::lock_guard<std::mutex> lk(mtx_);
    return cur_;
}
int VioBackend::windowSize() const {
    std::lock_guard<std::mutex> lk(mtx_);
    return (int)kfs_.size();
}
int VioBackend::landmarkCount() const {
    std::lock_guard<std::mutex> lk(mtx_);
    int n = 0; for (auto& kv : lms_) if (kv.second.in_graph) ++n; return n;
}

void VioBackend::snapshot(std::vector<KeyframeMeta>& kfs, std::vector<LandmarkOut>& lms) const {
    std::lock_guard<std::mutex> lk(mtx_);
    kfs.clear(); lms.clear();
    for (auto& kv : kfs_) kfs.push_back(kv.second);
    for (auto& kv : lms_) {
        if (!kv.second.in_graph) continue;
        LandmarkOut o; o.id = kv.first; o.p_w = kv.second.p_w; o.cov_trace = kv.second.cov_trace;
        lms.push_back(o);
    }
}

}  // namespace odyx
