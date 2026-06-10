#include "loop/loop_closer.h"
#include "common/math_util.h"
#include "common/log.h"
#include <opencv2/calib3d.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/core/eigen.hpp>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/nonlinear/PriorFactor.h>
#include <gtsam/inference/Symbol.h>
#include <algorithm>

using gtsam::Pose3; using gtsam::Rot3; using gtsam::Point3;
using gtsam::symbol_shorthand::X;

namespace odyx {

LoopCloser::LoopCloser(const Config& cfg, MapStore& map) : cfg_(cfg), map_(map) {}
LoopCloser::~LoopCloser() { stop(); }

void LoopCloser::start(CommitFn on_commit) {
    commit_ = std::move(on_commit);
    running_ = true;
    th_ = std::thread(&LoopCloser::run, this);
}
void LoopCloser::stop() {
    running_ = false;
    cv_.notify_all();
    if (th_.joinable()) th_.join();
}
void LoopCloser::enqueue(uint64_t kf_idx) {
    { std::lock_guard<std::mutex> lk(mtx_); queue_.push_back(kf_idx); }
    cv_.notify_one();
}

void LoopCloser::run() {
    while (running_) {
        uint64_t idx;
        {
            std::unique_lock<std::mutex> lk(mtx_);
            cv_.wait(lk, [&]{ return !queue_.empty() || !running_; });
            if (!running_) break;
            idx = queue_.back();        // latest; drop backlog
            queue_.clear();
        }
        if (!enabled_) continue;
        LoopResult res;
        if (detect(idx, res) && res.found) {
            ++loop_count_;
            last_loop_kf_ = idx;
            if (commit_) commit_(res);
            ODYXI("loop", "LOOP %llu<->%llu inliers=%d score=%.3f",
                  (unsigned long long)res.kf_query, (unsigned long long)res.kf_match,
                  res.inliers, res.score);
        }
    }
}

bool LoopCloser::detect(uint64_t kf_idx, LoopResult& out) {
    auto voc = map_.vocab();
    if (!voc) return false;
    if (kf_idx < last_loop_kf_ + (uint64_t)cfg_.loop_min_kf_gap) return false;

    KeyframeRecord q;
    if (!map_.getKeyframe(kf_idx, q) || q.bow.empty()) return false;
    auto kfs = map_.keyframes();

    // best older candidate by BoW score, with min keyframe gap.
    double best = 0; const KeyframeRecord* cand = nullptr;
    for (auto& k : kfs) {
        if (k.idx + (uint64_t)cfg_.loop_min_kf_gap > kf_idx) continue;
        if (k.bow.empty()) continue;
        double s = voc->score(q.bow, k.bow);
        if (s > best) { best = s; cand = &k; }
    }
    if (!cand || best < cfg_.dbow_score_thresh) return false;

    Pose3 T_m_q; int inliers = 0;
    if (!verifyAndSolve(q, *cand, T_m_q, inliers)) return false;
    if (inliers < cfg_.loop_min_inliers) return false;

    out.found = true; out.kf_query = kf_idx; out.kf_match = cand->idx;
    out.inliers = inliers; out.score = best;
    optimizePoseGraph(kfs, kf_idx, cand->idx, T_m_q, out);
    map_.addLoopEdge(cand->idx, kf_idx);
    return true;
}

bool LoopCloser::verifyAndSolve(const KeyframeRecord& q, const KeyframeRecord& m,
                                Pose3& T_m_q, int& inliers) {
    if (q.descriptors.empty() || m.descriptors.empty()) return false;

    // ORB descriptor matching with Lowe's ratio test.
    cv::BFMatcher bf(cv::NORM_HAMMING);
    std::vector<std::vector<cv::DMatch>> knn;
    bf.knnMatch(q.descriptors, m.descriptors, knn, 2);

    std::vector<cv::Point2f> p_q, p_m;
    for (auto& mm : knn) {
        if (mm.size() < 2) continue;
        if (mm[0].distance > 0.75f * mm[1].distance) continue;   // Lowe ratio
        int qi = mm[0].queryIdx, mi = mm[0].trainIdx;
        if (qi >= (int)q.kps.size() || mi >= (int)m.kps.size()) continue;
        p_q.push_back(q.kps[qi].pt);
        p_m.push_back(m.kps[mi].pt);
    }
    if (p_q.size() < (size_t)cfg_.loop_min_inliers) return false;

    // Geometric verification: essential matrix + RANSAC, recover relative pose.
    // The relative translation is up to scale; we anchor it with the VIO
    // baseline between the two keyframes (metric, from the smoother).
    const auto& C = cfg_.cam;
    cv::Mat K = (cv::Mat_<double>(3,3) << C.fx,0,C.cx, 0,C.fy,C.cy, 0,0,1);
    cv::Mat inl, E = cv::findEssentialMat(p_q, p_m, K, cv::RANSAC, 0.999, 1.5, inl);
    if (E.empty()) return false;
    cv::Mat Rcv, tcv;
    int good = cv::recoverPose(E, p_q, p_m, K, Rcv, tcv, inl);
    inliers = good;
    if (good < cfg_.loop_min_inliers) return false;

    Mat3 Rqm; Vec3 tqm; cv::cv2eigen(Rcv, Rqm); cv::cv2eigen(tcv, tqm);
    // scale the unit translation by the VIO inter-keyframe baseline magnitude.
    double baseline = (q.pose.p_wb - m.pose.p_wb).norm();
    if (baseline < 1e-3) baseline = 0.3;
    T_m_q = Pose3(Rot3(Rqm.transpose()), Point3(-Rqm.transpose()*tqm * baseline));
    return true;
}

void LoopCloser::optimizePoseGraph(const std::vector<KeyframeRecord>& kfs,
                                   uint64_t qi, uint64_t mi, const Pose3& T_m_q,
                                   LoopResult& out) {
    gtsam::NonlinearFactorGraph g;
    gtsam::Values v;
    auto odom_n = gtsam::noiseModel::Diagonal::Sigmas(
        (gtsam::Vector(6) << 0.02,0.02,0.02, 0.05,0.05,0.05).finished());
    auto loop_n = gtsam::noiseModel::Diagonal::Sigmas(
        (gtsam::Vector(6) << 0.05,0.05,0.05, 0.1,0.1,0.1).finished());
    auto prior_n = gtsam::noiseModel::Diagonal::Sigmas(
        (gtsam::Vector(6) << 1e-4,1e-4,1e-4, 1e-4,1e-4,1e-4).finished());

    // sequential odometry chain from current poses
    std::vector<KeyframeRecord> sorted = kfs;
    std::sort(sorted.begin(), sorted.end(),
              [](const KeyframeRecord&a,const KeyframeRecord&b){return a.idx<b.idx;});
    for (auto& k : sorted) v.insert(X(k.idx), Pose3(Rot3(k.pose.q_wb), Point3(k.pose.p_wb)));
    if (sorted.empty()) return;
    g.addPrior(X(sorted.front().idx),
               Pose3(Rot3(sorted.front().pose.q_wb), Point3(sorted.front().pose.p_wb)), prior_n);
    for (size_t i = 1; i < sorted.size(); ++i) {
        Pose3 a(Rot3(sorted[i-1].pose.q_wb), Point3(sorted[i-1].pose.p_wb));
        Pose3 b(Rot3(sorted[i].pose.q_wb), Point3(sorted[i].pose.p_wb));
        g.add(gtsam::BetweenFactor<Pose3>(X(sorted[i-1].idx), X(sorted[i].idx),
                                          a.between(b), odom_n));
    }
    // loop constraint
    g.add(gtsam::BetweenFactor<Pose3>(X(mi), X(qi), T_m_q, loop_n));

    try {
        gtsam::LevenbergMarquardtOptimizer opt(g, v);
        gtsam::Values r = opt.optimize();
        for (auto& k : sorted)
            if (r.exists(X(k.idx))) out.corrected[k.idx] = r.at<Pose3>(X(k.idx));
    } catch (const std::exception& e) {
        ODYXE("loop", "pose-graph optimize failed: %s", e.what());
        out.found = false;
    }
}

}  // namespace odyx
