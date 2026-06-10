#include "reloc/relocalizer.h"
#include "common/log.h"
#include <opencv2/calib3d.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/core/eigen.hpp>

namespace odyx {

RelocResult Relocalizer::relocalize(const cv::Mat& desc,
                                    const std::vector<cv::KeyPoint>& kps,
                                    const std::map<uint64_t, Vec3>& world_landmarks) {
    RelocResult res;
    auto voc = map_.vocab();
    if (!voc || desc.empty()) return res;

    DBoW3::BowVector qbow;
    voc->transform(desc, qbow);

    auto kfs = map_.keyframes();
    double best = 0; const KeyframeRecord* cand = nullptr;
    for (auto& k : kfs) {
        if (k.bow.empty()) continue;
        double s = voc->score(qbow, k.bow);
        if (s > best) { best = s; cand = &k; }
    }
    if (!cand || best < cfg_.dbow_score_thresh) return res;

    // match current descriptors to the candidate keyframe.
    cv::BFMatcher bf(cv::NORM_HAMMING);
    std::vector<std::vector<cv::DMatch>> knn;
    bf.knnMatch(desc, cand->descriptors, knn, 2);

    std::vector<cv::Point3f> obj; std::vector<cv::Point2f> img;
    for (auto& mm : knn) {
        if (mm.size() < 2) continue;
        if (mm[0].distance > 0.75f * mm[1].distance) continue;
        int qi = mm[0].queryIdx, mi = mm[0].trainIdx;
        if (mi >= (int)cand->obs_ids.size()) continue;
        uint64_t lid = cand->obs_ids[mi];
        auto it = world_landmarks.find(lid);
        if (it == world_landmarks.end()) continue;
        obj.emplace_back((float)it->second.x(), (float)it->second.y(), (float)it->second.z());
        img.push_back(kps[qi].pt);
    }
    if (obj.size() < (size_t)cfg_.loop_min_inliers) return res;

    const auto& C = cfg_.cam;
    cv::Mat K = (cv::Mat_<double>(3,3) << C.fx,0,C.cx, 0,C.fy,C.cy, 0,0,1);
    cv::Mat D = (cv::Mat_<double>(1,5) << C.dist[0],C.dist[1],C.dist[2],C.dist[3],C.dist[4]);
    cv::Mat rvec, tvec, inl;
    bool ok = cv::solvePnPRansac(obj, img, K, D, rvec, tvec, false,
                                 200, 3.0f, 0.99, inl, cv::SOLVEPNP_EPNP);
    if (!ok || inl.rows < cfg_.loop_min_inliers) return res;

    cv::Mat Rcv; cv::Rodrigues(rvec, Rcv);
    Mat3 Rcw; Vec3 tcw; cv::cv2eigen(Rcv, Rcw); cv::cv2eigen(tvec, tcw);
    // PnP gives camera-from-world (x_c = Rcw*x_w + tcw); invert to world pose.
    Mat3 Rwc = Rcw.transpose();
    Vec3 twc = -Rwc * tcw;

    // camera->body via extrinsics
    Mat3 Rbc = cfg_.extr.q_bc.toRotationMatrix();
    Mat3 Rwb = Rwc * Rbc.transpose();
    Vec3 twb = twc - Rwb * cfg_.extr.t_bc;

    res.ok = true; res.matched_kf = cand->idx; res.inliers = inl.rows;
    res.pose.q_wb = Quat(Rwb);
    res.pose.p_wb = twb;
    res.pose.valid = true;
    ODYXI("reloc", "relocalized to kf %llu, inliers=%d, score=%.3f",
          (unsigned long long)cand->idx, res.inliers, best);
    return res;
}

}  // namespace odyx
