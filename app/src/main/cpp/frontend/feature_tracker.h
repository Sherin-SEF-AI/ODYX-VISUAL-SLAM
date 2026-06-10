// ===========================================================================
// ODYX :: visual front-end.
//
// KLT optical-flow tracking of GFTT/FAST corners across frames, grid-spread
// redetection to maintain a target count, RANSAC (fundamental) outlier
// rejection, and keyframe selection from parallax / tracked-ratio / time.
// ORB descriptors are computed on KEYFRAMES ONLY (for loop closure / reloc).
//
// Reimplemented from first principles on OpenCV primitives — no GPL SLAM code.
// ===========================================================================
#pragma once
#include "common/odyx_types.h"
#include "common/config.h"
#include <opencv2/core.hpp>
#include <unordered_map>
#include <memory>

namespace odyx {

// One tracked feature observation in the current frame.
struct FeatureObs {
    uint64_t id = 0;          // persistent track id
    Vec2 uv = Vec2::Zero();   // pixel (distorted) coords
    Vec2 undist = Vec2::Zero(); // normalized undistorted coords (x/z, y/z)
    int age = 0;              // # frames tracked
    bool is_new = false;
};

struct TrackResult {
    TimeNs t_ns = 0;
    std::vector<FeatureObs> obs;
    int n_tracked = 0;
    int n_new = 0;
    int n_ransac_rejected = 0;
    double median_parallax = 0;   // px, vs last keyframe
    bool is_keyframe = false;
};

class FeatureTracker {
public:
    explicit FeatureTracker(const Config& cfg);
    void setConfig(const Config& cfg) { cfg_ = cfg; }

    // Track features from previous frame into `frame`. Decides keyframe-ness.
    TrackResult track(const Frame& frame);

    // For keyframes: ORB descriptors at the current observations.
    // Returns keypoints aligned with `ids`; descriptors rows match.
    void computeKeyframeOrb(const Frame& frame,
                            const std::vector<FeatureObs>& obs,
                            std::vector<cv::KeyPoint>& kps_out,
                            cv::Mat& desc_out,
                            std::vector<uint64_t>& ids_out) const;

    void markKeyframeTaken(const TrackResult& tr);  // resets parallax baseline
    void reset();

private:
    cv::Mat toMat(const Frame& f) const;
    void undistort(std::vector<FeatureObs>& obs) const;
    void redetect(const cv::Mat& img);
    int ransacReject(std::vector<cv::Point2f>& prev,
                     std::vector<cv::Point2f>& cur,
                     std::vector<uint64_t>& ids,
                     std::vector<int>& ages);

    Config cfg_;
    cv::Mat prev_img_;
    std::vector<cv::Point2f> prev_pts_;
    std::vector<uint64_t> ids_;
    std::vector<int> ages_;
    std::unordered_map<uint64_t, cv::Point2f> kf_baseline_;  // id->uv at last KF
    uint64_t next_id_ = 1;
    int frames_since_kf_ = 0;
    TimeNs last_kf_t_ = 0;
};

}  // namespace odyx
