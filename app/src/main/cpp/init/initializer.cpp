#include "init/initializer.h"
#include "common/math_util.h"
#include "common/log.h"
#include <cstdio>
#include <opencv2/calib3d.hpp>
#include <opencv2/core/eigen.hpp>
#include <Eigen/Dense>
#include <algorithm>

namespace odyx {

double Initializer::avgParallax(size_t i, size_t j) const {
    double par; int n; parallaxCommon(i, j, par, n); return par;
}

void Initializer::parallaxCommon(size_t i, size_t j, double& par_px, int& common) const {
    // mean pixel displacement of features common to kf i and kf j.
    std::map<uint64_t, Vec2> a;
    for (const auto& o : kfs_[i].obs) a[o.id] = o.undist;
    double sum = 0; int n = 0;
    for (const auto& o : kfs_[j].obs) {
        auto it = a.find(o.id);
        if (it != a.end()) { sum += (o.undist - it->second).norm(); ++n; }
    }
    common = n;
    par_px = n > 0 ? (sum / n) * cfg_.cam.fx : 0.0;
}

bool Initializer::selectReference(size_t& ref) const {
    // Scan back over the window for the frame that, paired with the latest, has
    // BOTH enough common features (for the essential matrix) and the most
    // parallax (for triangulation). No early break — feature dropouts on any
    // single frame must not abort the search.
    const size_t latest = kfs_.size() - 1;
    double best_par = 0; int best_common = 0; bool found = false;
    for (int i = (int)latest - 1; i >= 0; --i) {
        double par; int common;
        parallaxCommon((size_t)i, latest, par, common);
        if (common > best_common) best_common = common;   // track for diagnostics
        if (common >= kMinCommon && par >= kMinAvgParallaxPx && par > best_par) {
            best_par = par; ref = (size_t)i; found = true;
        }
    }
    last_best_par_ = best_par; last_best_common_ = best_common;   // for logging
    return found;
}

InitResult Initializer::addKeyframe(const BootKf& kf) {
    kfs_.push_back(kf);
    InitResult r;
    r.state = InitState::kCollecting;
    if (kfs_.size() < kMinKf) { r.reason = "collecting"; return r; }
    if (kfs_.size() > 3 * kMinKf) kfs_.erase(kfs_.begin());  // slide bootstrap window

    // Require genuine translational excitation: a reference frame with enough
    // common features AND parallax must exist (not measured between extremes).
    if (!selectReference(ref_idx_)) {
        char buf[128];
        snprintf(buf, sizeof(buf), "no ref: best_par=%.1fpx best_common=%d (need par>=%.0f common>=%d)",
                 last_best_par_, last_best_common_, kMinAvgParallaxPx, kMinCommon);
        r.reason = buf;
        return r;
    }

    r.state = InitState::kSfm;
    if (!visualSfm(r)) { r.reason = "SfM failed"; return r; }

    r.state = InitState::kVialign;
    if (!calibrateGyroBias()) { r.reason = "gyro-bias calib failed"; return r; }
    if (!linearAlign(r))      { r.reason = "linear VI alignment failed"; return r; }
    refineGravity(r);

    r.state = InitState::kDone;
    r.ok = true;
    return r;
}

// ---- step 2: vision-only up-to-scale SfM ----------------------------------
bool Initializer::visualSfm(InitResult& r) {
    const size_t N = kfs_.size();
    // Reference pair = (selected widest-baseline reference, latest) — NOT the
    // window extremes (which share no features after walking).
    const size_t i0 = ref_idx_, i1 = N - 1;

    // shared features between i0 and i1
    std::map<uint64_t, Vec2> m0;
    for (const auto& o : kfs_[i0].obs) m0[o.id] = o.undist;
    std::vector<cv::Point2f> p0, p1; std::vector<uint64_t> shared;
    for (const auto& o : kfs_[i1].obs) {
        auto it = m0.find(o.id);
        if (it != m0.end()) {
            p0.emplace_back((float)it->second.x(), (float)it->second.y());
            p1.emplace_back((float)o.undist.x(), (float)o.undist.y());
            shared.push_back(o.id);
        }
    }
    if (p0.size() < 12) { ODYXW("init", "SfM: only %zu shared pts", p0.size()); return false; }

    // Essential matrix on normalized coords (identity K).
    cv::Mat E, inl;
    E = cv::findEssentialMat(p0, p1, 1.0, cv::Point2d(0,0), cv::RANSAC, 0.999, 0.004, inl);
    if (E.empty()) { ODYXW("init", "SfM: essential matrix empty"); return false; }
    cv::Mat Rcv, tcv;
    int good = cv::recoverPose(E, p0, p1, Rcv, tcv, 1.0, cv::Point2d(0,0), inl);
    if (good < 8) { ODYXW("init", "SfM: recoverPose good=%d", good); return false; }

    Mat3 R10; Vec3 t10;
    cv::cv2eigen(Rcv, R10); cv::cv2eigen(tcv, t10);   // cam1 <- cam0 (unit t)

    // Set SfM poses: cam0 at identity, cam1 from recoverPose.
    kfs_[i0].R_c0_ck = Mat3::Identity(); kfs_[i0].t_c0_ck = Vec3::Zero(); kfs_[i0].pose_set = true;
    // recoverPose gives R,t such that x1 = R*x0 + t (cam1 frame). We want pose of
    // camk in cam0: R_c0_ck = R10^T, t_c0_ck = -R10^T t10.
    kfs_[i1].R_c0_ck = R10.transpose();
    kfs_[i1].t_c0_ck = -R10.transpose() * t10;
    kfs_[i1].pose_set = true;

    // Triangulate shared landmarks from the reference pair.
    cv::Mat P0 = (cv::Mat_<double>(3,4) << 1,0,0,0, 0,1,0,0, 0,0,1,0);
    cv::Mat P1(3,4,CV_64F);
    cv::eigen2cv(Mat3(R10), P1.colRange(0,3));
    P1.at<double>(0,3)=t10.x(); P1.at<double>(1,3)=t10.y(); P1.at<double>(2,3)=t10.z();
    cv::Mat X;
    cv::triangulatePoints(P0, P1, p0, p1, X);
    for (int c = 0; c < X.cols; ++c) {
        if (!inl.empty() && !inl.at<uchar>(c)) continue;
        Vec4 h(X.at<float>(0,c),X.at<float>(1,c),X.at<float>(2,c),X.at<float>(3,c));
        if (std::abs(h(3)) < 1e-6) continue;
        Vec3 P = h.head<3>() / h(3);
        if (P.z() <= 0.1 || P.z() > 80.0) continue;   // cheirality + range gate
        r.landmarks[shared[c]] = P;                    // in cam0 frame (up-to-scale)
    }
    if (r.landmarks.size() < 8) { ODYXW("init", "SfM: only %zu landmarks triangulated", r.landmarks.size()); return false; }

    // PnP every other keyframe against the up-to-scale landmark cloud (frames
    // before the reference won't see these landmarks and are dropped).
    cv::Mat K = cv::Mat::eye(3,3,CV_64F);  // normalized coords
    for (size_t k = 0; k < N; ++k) {
        if (k == i0 || k == i1) continue;
        std::vector<cv::Point3f> objp; std::vector<cv::Point2f> imgp;
        for (const auto& o : kfs_[k].obs) {
            auto it = r.landmarks.find(o.id);
            if (it != r.landmarks.end()) {
                objp.emplace_back((float)it->second.x(),(float)it->second.y(),(float)it->second.z());
                imgp.emplace_back((float)o.undist.x(),(float)o.undist.y());
            }
        }
        if (objp.size() < 8) { kfs_[k].pose_set = false; continue; }
        cv::Mat rvec, tvec, R;
        if (!cv::solvePnPRansac(objp, imgp, K, cv::Mat(), rvec, tvec, false,
                                100, 0.004f, 0.99)) { kfs_[k].pose_set=false; continue; }
        cv::Rodrigues(rvec, R);
        Mat3 Rk; Vec3 tk; cv::cv2eigen(R,Rk); cv::cv2eigen(tvec,tk);
        kfs_[k].R_c0_ck = Rk.transpose();
        kfs_[k].t_c0_ck = -Rk.transpose()*tk;
        kfs_[k].pose_set = true;
    }
    return true;
}

// ---- step 3: gyro bias calibration ----------------------------------------
bool Initializer::calibrateGyroBias() {
    // Solve dbg minimizing sum || Log( dR_ij^T * R_bi^T R_bj ) - J*dbg ||.
    // Using camera SfM rotations as a proxy for body rotations (extrinsic R_bc
    // applied below).
    Mat3 A = Mat3::Zero(); Vec3 b = Vec3::Zero();
    const Mat3 Rbc = cfg_.extr.q_bc.toRotationMatrix();
    for (size_t k = 1; k < kfs_.size(); ++k) {
        if (!kfs_[k].preint || !kfs_[k-1].pose_set || !kfs_[k].pose_set) continue;
        // body rotation between kf k-1 and k from SfM (cam) + extrinsics
        Mat3 Rci = kfs_[k-1].R_c0_ck, Rcj = kfs_[k].R_c0_ck;
        Mat3 Rb_ij = Rbc * (Rci.transpose()*Rcj) * Rbc.transpose();
        Mat3 dR = kfs_[k].preint->dR();
        Mat3 Jr = kfs_[k].preint->dR_dbg();
        Vec3 resid = logSO3(dR.transpose() * Rb_ij);
        A += Jr.transpose()*Jr;
        b += Jr.transpose()*resid;
    }
    Vec3 dbg = A.ldlt().solve(b);
    if (!dbg.allFinite() || dbg.norm() > 1.0) return false;
    // Do NOT reset the preintegrations here — resetIntegrationAndSetBias() would
    // CLEAR the accumulated IMU (it is never re-integrated), collapsing the VI
    // alignment to scale=0/|g|=0. The gyro-bias correction is instead applied
    // analytically in dp_biased()/dv_biased() via the first-order Jacobians,
    // using bg_ below.
    bg_ = dbg;
    return true;
}

// ---- step 4: linear visual-inertial alignment -----------------------------
// Unknowns: per-kf body velocity v_i (3 each), gravity g (3), scale s (1).
bool Initializer::linearAlign(InitResult& r) {
    const size_t N = kfs_.size();
    // count usable consecutive pairs
    std::vector<size_t> idx;
    for (size_t k = 0; k < N; ++k) if (kfs_[k].pose_set) idx.push_back(k);
    if (idx.size() < 4) return false;   // need a few posed frames for VI alignment
    const size_t n = idx.size();
    const int dim = 3*(int)n + 3 + 1;   // velocities + gravity + scale
    Eigen::MatrixXd H = Eigen::MatrixXd::Zero(dim, dim);
    Eigen::VectorXd bb = Eigen::VectorXd::Zero(dim);

    const Mat3 Rbc = cfg_.extr.q_bc.toRotationMatrix();
    const Vec3 pbc = cfg_.extr.t_bc;

    for (size_t a = 0; a + 1 < n; ++a) {
        size_t i = idx[a], j = idx[a+1];
        auto& pim = kfs_[j].preint;
        if (!pim) continue;
        const double dt = pim->dt();
        if (dt <= 0) continue;

        // SfM gives camera poses (R_c0_ck, t_c0_ck up to scale). Body rotation:
        Mat3 Rwbi = kfs_[i].R_c0_ck * Rbc.transpose();
        Mat3 Rwbj = kfs_[j].R_c0_ck * Rbc.transpose();
        Vec3 pci = kfs_[i].t_c0_ck;   // up-to-scale cam position
        Vec3 pcj = kfs_[j].t_c0_ck;

        // preintegration with calibrated bg
        Vec3 dp = pim->dp_biased(Vec3::Zero(), bg_);
        Vec3 dv = pim->dv_biased(Vec3::Zero(), bg_);

        // Build the two block rows (position & velocity constraints).
        Eigen::MatrixXd Hi = Eigen::MatrixXd::Zero(6, 10);  // [vi(3) vj(3) g(3) s(1)]
        Eigen::VectorXd zi = Eigen::VectorXd::Zero(6);

        // position constraint:
        //  s*(pcj - pci) = Rwbi*(vi*dt) - 0.5*g*dt^2 + Rwbi*dp + (Rwbi-Rwbj)*pbc
        Hi.block<3,3>(0,0) = -Rwbi*dt;                 // vi
        Hi.block<3,3>(0,6) = 0.5*Mat3::Identity()*dt*dt; // g
        Hi.block<3,1>(0,9) = (pcj - pci);              // s
        zi.segment<3>(0) = Rwbi*dp + (Rwbi - Rwbj)*pbc;

        // velocity constraint:
        //  0 = Rwbi*dv - g*dt + Rwbi*vi - Rwbj*vj  -> rearrange
        Hi.block<3,3>(3,0) =  Rwbi;                    // vi
        Hi.block<3,3>(3,3) = -Rwbj;                    // vj
        Hi.block<3,3>(3,6) = -Mat3::Identity()*dt;     // g
        zi.segment<3>(3) = -Rwbi*dv;

        // scatter into the big system
        std::array<int,4> off = {3*(int)a, 3*(int)(a+1), 3*(int)n, 3*(int)n+3};
        std::array<int,4> sz  = {3,3,3,1};
        std::array<int,4> loc = {0,3,6,9};
        for (int bi=0; bi<4; ++bi) for (int bj=0; bj<4; ++bj) {
            H.block(off[bi], off[bj], sz[bi], sz[bj]) +=
                Hi.block(0,loc[bi],6,sz[bi]).transpose() * Hi.block(0,loc[bj],6,sz[bj]);
        }
        for (int bi=0; bi<4; ++bi)
            bb.segment(off[bi], sz[bi]) += Hi.block(0,loc[bi],6,sz[bi]).transpose()*zi;
    }

    H += 1e-9 * Eigen::MatrixXd::Identity(dim, dim);
    Eigen::VectorXd x = H.ldlt().solve(bb);
    if (!x.allFinite()) return false;

    double s = x(dim-1);
    Vec3 g = x.segment<3>(3*(int)n);
    ODYXI("init", "VIalign: n=%zu scale=%.4f |g|=%.3f (want |g|~%.2f)", n, s, g.norm(), cfg_.imu.g);
    if (s < 1e-4 || s > 1e4) { ODYXW("init", "VIalign: scale out of range %.4g", s); return false; }
    if (std::abs(g.norm() - cfg_.imu.g) > 3.5) { ODYXW("init", "VIalign: |g|=%.2f too far from %.2f", g.norm(), cfg_.imu.g); return false; }

    r.scale = s;
    r.gravity_l = g;
    r.bg = bg_;
    r.vel_b.resize(n);
    for (size_t a = 0; a < n; ++a) r.vel_b[a] = x.segment<3>(3*(int)a);

    // build metric, gravity-unaligned states (in local frame) for the backend
    r.states.clear();
    for (size_t a = 0; a < n; ++a) {
        size_t k = idx[a];
        NavStateOut st;
        st.t_ns = kfs_[k].t_ns;
        Mat3 Rwb = kfs_[k].R_c0_ck * Rbc.transpose();
        st.q_wb = Quat(Rwb);
        st.p_wb = s * kfs_[k].t_c0_ck;   // metric body position (local frame)
        st.v_w  = r.vel_b[a];
        st.bg = bg_; st.ba = Vec3::Zero();
        st.valid = true;
        r.states.push_back(st);
    }
    // scale landmarks
    for (auto& kv : r.landmarks) kv.second *= s;
    return true;
}

// ---- step 5: gravity refinement & world alignment -------------------------
void Initializer::refineGravity(InitResult& r) {
    // Meaningful residual = how far the RAW aligned gravity was from |g| before
    // we force the magnitude (linearAlign already gated |g-9.81|<3).
    r.gravity_resid = std::abs(r.gravity_l.norm() - cfg_.imu.g);
    // Rotate the local frame so gravity points to -Z with magnitude |g|=imu.g.
    Vec3 g = r.gravity_l.normalized() * cfg_.imu.g;
    Vec3 zw(0,0,-1);                       // world gravity direction (-Z)
    Vec3 gdir = g.normalized();
    Vec3 axis = gdir.cross(zw);
    double s = axis.norm(), c = gdir.dot(zw);
    Mat3 Rwl = Mat3::Identity();
    if (s > 1e-8) Rwl = expSO3(std::atan2(s,c) * axis.normalized());
    r.R_wl = Rwl;
    r.gravity_l = g;

    // rotate states + landmarks into the gravity-aligned world
    for (auto& st : r.states) {
        st.q_wb = Quat(Rwl * st.q_wb.toRotationMatrix());
        st.p_wb = Rwl * st.p_wb;
        st.v_w  = Rwl * st.v_w;
    }
    for (auto& kv : r.landmarks) kv.second = Rwl * kv.second;
}

}  // namespace odyx
