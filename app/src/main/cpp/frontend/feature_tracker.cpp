#include "frontend/feature_tracker.h"
#include "common/log.h"
#include <opencv2/imgproc.hpp>
#include <opencv2/video/tracking.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/calib3d.hpp>
#include <algorithm>
#include <numeric>

namespace odyx {

FeatureTracker::FeatureTracker(const Config& cfg) : cfg_(cfg) {}

cv::Mat FeatureTracker::toMat(const Frame& f) const {
    return cv::Mat(f.height, f.width, CV_8UC1,
                   const_cast<uint8_t*>(f.gray.data())).clone();
}

void FeatureTracker::undistort(std::vector<FeatureObs>& obs) const {
    // Always normalize with the available intrinsics. `cam.valid` only gates the
    // UI accuracy warning — NOT whether to undistort. Skipping this left every
    // `undist` at (0,0), which zeroed all parallax and broke SfM/init when
    // running on default (uncalibrated) intrinsics.
    if (obs.empty() || cfg_.cam.fx <= 1.0) return;
    const auto& c = cfg_.cam;
    cv::Mat K = (cv::Mat_<double>(3,3) << c.fx,0,c.cx, 0,c.fy,c.cy, 0,0,1);
    cv::Mat D = (cv::Mat_<double>(1,5) << c.dist[0],c.dist[1],c.dist[2],c.dist[3],c.dist[4]);
    std::vector<cv::Point2f> in(obs.size()), out;
    for (size_t i = 0; i < obs.size(); ++i)
        in[i] = cv::Point2f((float)obs[i].uv.x(), (float)obs[i].uv.y());
    cv::undistortPoints(in, out, K, D);   // -> normalized coords
    for (size_t i = 0; i < obs.size(); ++i)
        obs[i].undist = Vec2(out[i].x, out[i].y);
}

int FeatureTracker::ransacReject(std::vector<cv::Point2f>& prev,
                                 std::vector<cv::Point2f>& cur,
                                 std::vector<uint64_t>& ids,
                                 std::vector<int>& ages) {
    if (prev.size() < 8) return 0;
    std::vector<uchar> inl;
    cv::findFundamentalMat(prev, cur, cv::FM_RANSAC,
                           cfg_.ransac_thresh_px, 0.99, inl);
    int rejected = 0;
    std::vector<cv::Point2f> p2, c2;
    std::vector<uint64_t> id2; std::vector<int> a2;
    p2.reserve(prev.size()); c2.reserve(cur.size());
    for (size_t i = 0; i < inl.size(); ++i) {
        if (inl[i]) { p2.push_back(prev[i]); c2.push_back(cur[i]); id2.push_back(ids[i]); a2.push_back(ages[i]); }
        else ++rejected;
    }
    prev.swap(p2); cur.swap(c2); ids.swap(id2); ages.swap(a2);
    return rejected;
}

void FeatureTracker::redetect(const cv::Mat& img) {
    const int want = std::max(0, cfg_.target_features - (int)prev_pts_.size());
    if (want <= 0) return;

    // Build an occupancy mask so new corners spread away from existing tracks.
    cv::Mat mask(img.size(), CV_8UC1, cv::Scalar(255));
    const int r = std::max(8, img.cols / (cfg_.grid_cols * 2));
    for (const auto& p : prev_pts_)
        cv::circle(mask, p, r, cv::Scalar(0), -1);

    // GFTT per grid cell for spatial spread.
    const int cw = img.cols / cfg_.grid_cols;
    const int ch = img.rows / cfg_.grid_rows;
    const int per_cell = std::max(1, want / (cfg_.grid_cols * cfg_.grid_rows) + 1);
    for (int gy = 0; gy < cfg_.grid_rows; ++gy) {
        for (int gx = 0; gx < cfg_.grid_cols; ++gx) {
            cv::Rect roi(gx*cw, gy*ch,
                         (gx==cfg_.grid_cols-1)? img.cols-gx*cw : cw,
                         (gy==cfg_.grid_rows-1)? img.rows-gy*ch : ch);
            std::vector<cv::Point2f> corners;
            cv::goodFeaturesToTrack(img(roi), corners, per_cell, 0.01, 10,
                                    mask(roi), 3, false, 0.04);
            for (auto& c : corners) {
                cv::Point2f g(c.x + roi.x, c.y + roi.y);
                prev_pts_.push_back(g);
                ids_.push_back(next_id_++);
                ages_.push_back(0);
            }
        }
    }
}

TrackResult FeatureTracker::track(const Frame& frame) {
    TrackResult tr;
    tr.t_ns = frame.t_ns;
    cv::Mat img = toMat(frame);

    if (prev_img_.empty() || prev_pts_.empty()) {
        // bootstrap detection
        prev_img_ = img;
        redetect(img);
        for (size_t i = 0; i < prev_pts_.size(); ++i) {
            FeatureObs o; o.id = ids_[i]; o.age = 0; o.is_new = true;
            o.uv = Vec2(prev_pts_[i].x, prev_pts_[i].y);
            tr.obs.push_back(o);
        }
        undistort(tr.obs);
        tr.n_new = (int)tr.obs.size();
        tr.is_keyframe = true;          // first frame is always a keyframe
        last_kf_t_ = frame.t_ns;
        frames_since_kf_ = 0;
        return tr;
    }

    // --- forward KLT ---
    std::vector<cv::Point2f> cur;
    std::vector<uchar> status; std::vector<float> err;
    // 5 pyramid levels + 31px window track much larger inter-frame motion
    // (fast handheld / low-fps), reducing tracking loss under aggressive motion.
    cv::calcOpticalFlowPyrLK(prev_img_, img, prev_pts_, cur, status, err,
                             cv::Size(31,31), 5,
                             cv::TermCriteria(cv::TermCriteria::COUNT+cv::TermCriteria::EPS, 30, 0.01));

    // --- backward KLT consistency check ---
    std::vector<cv::Point2f> back;
    std::vector<uchar> bstatus; std::vector<float> berr;
    cv::calcOpticalFlowPyrLK(img, prev_img_, cur, back, bstatus, berr,
                             cv::Size(31,31), 5);

    std::vector<cv::Point2f> kept_prev, kept_cur;
    std::vector<uint64_t> kept_ids; std::vector<int> kept_ages;
    for (size_t i = 0; i < cur.size(); ++i) {
        if (!status[i] || !bstatus[i]) continue;
        if (cv::norm(prev_pts_[i] - back[i]) > 3.0) continue;       // FB error (relaxed for motion survival)
        if (cur[i].x < 0 || cur[i].y < 0 || cur[i].x >= img.cols || cur[i].y >= img.rows) continue;
        kept_prev.push_back(prev_pts_[i]);
        kept_cur.push_back(cur[i]);
        kept_ids.push_back(ids_[i]);
        kept_ages.push_back(ages_[i] + 1);
    }

    tr.n_ransac_rejected = ransacReject(kept_prev, kept_cur, kept_ids, kept_ages);
    tr.n_tracked = (int)kept_cur.size();

    // --- parallax vs last keyframe baseline ---
    std::vector<double> par;
    for (size_t i = 0; i < kept_ids.size(); ++i) {
        auto it = kf_baseline_.find(kept_ids[i]);
        if (it != kf_baseline_.end())
            par.push_back(cv::norm(kept_cur[i] - it->second));
    }
    if (!par.empty()) {
        std::nth_element(par.begin(), par.begin()+par.size()/2, par.end());
        tr.median_parallax = par[par.size()/2];
    }

    // commit current tracks
    prev_img_ = img;
    prev_pts_.swap(kept_cur);
    ids_.swap(kept_ids);
    ages_.swap(kept_ages);
    ++frames_since_kf_;

    // --- keyframe decision ---
    const double dt = ns_to_s(frame.t_ns - last_kf_t_);
    const int n0 = (int)kf_baseline_.size();
    const double tracked_ratio = n0 > 0 ? (double)par.size() / n0 : 1.0;
    bool kf = false;
    if (dt >= cfg_.kf_min_dt_s) {
        if (tr.median_parallax >= cfg_.kf_parallax_px) kf = true;
        if (tracked_ratio < cfg_.kf_tracked_ratio) kf = true;
        if (dt >= cfg_.kf_max_dt_s) kf = true;
    }
    if (tr.n_tracked < cfg_.target_features / 2 && dt >= cfg_.kf_min_dt_s) kf = true;

    if (kf) redetect(prev_img_);  // top up before exposing the keyframe set

    // assemble observations
    for (size_t i = 0; i < prev_pts_.size(); ++i) {
        FeatureObs o; o.id = ids_[i]; o.age = ages_[i];
        o.is_new = (ages_[i] == 0);
        o.uv = Vec2(prev_pts_[i].x, prev_pts_[i].y);
        if (o.is_new) ++tr.n_new;
        tr.obs.push_back(o);
    }
    undistort(tr.obs);
    tr.is_keyframe = kf;
    return tr;
}

void FeatureTracker::markKeyframeTaken(const TrackResult& tr) {
    kf_baseline_.clear();
    for (const auto& o : tr.obs)
        kf_baseline_[o.id] = cv::Point2f((float)o.uv.x(), (float)o.uv.y());
    last_kf_t_ = tr.t_ns;
    frames_since_kf_ = 0;
}

void FeatureTracker::computeKeyframeOrb(const Frame& frame,
                                        const std::vector<FeatureObs>& obs,
                                        std::vector<cv::KeyPoint>& kps_out,
                                        cv::Mat& desc_out,
                                        std::vector<uint64_t>& ids_out) const {
    cv::Mat img = toMat(frame);
    std::vector<cv::KeyPoint> kps;
    std::vector<uint64_t> ids;
    kps.reserve(obs.size());
    for (const auto& o : obs) {
        kps.emplace_back(cv::Point2f((float)o.uv.x(), (float)o.uv.y()), 31.f);
        ids.push_back(o.id);
    }
    static cv::Ptr<cv::ORB> orb = cv::ORB::create(1000, 1.2f, 8);
    cv::Mat desc;
    // compute() may drop keypoints near the border; track which survive.
    std::vector<cv::KeyPoint> kps_copy = kps;
    orb->compute(img, kps_copy, desc);
    // Map surviving keypoints back to ids by nearest original location.
    ids_out.clear(); kps_out.clear();
    desc_out = cv::Mat();
    if (kps_copy.empty()) return;
    desc_out = desc.clone();
    for (auto& kp : kps_copy) {
        // find closest original
        double best = 1e9; size_t bi = 0;
        for (size_t i = 0; i < kps.size(); ++i) {
            double d = cv::norm(kp.pt - kps[i].pt);
            if (d < best) { best = d; bi = i; }
        }
        kps_out.push_back(kp);
        ids_out.push_back(ids[bi]);
    }
}

void FeatureTracker::reset() {
    prev_img_.release();
    prev_pts_.clear(); ids_.clear(); ages_.clear();
    kf_baseline_.clear();
    next_id_ = 1; frames_since_kf_ = 0; last_kf_t_ = 0;
}

}  // namespace odyx
