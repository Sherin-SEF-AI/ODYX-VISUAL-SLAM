#include "frontend/calibrator.h"
#include "common/log.h"
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>

namespace odyx {

bool CameraCalibrator::process(const Frame& f) {
    cv::Mat img(f.height, f.width, CV_8UC1, const_cast<uint8_t*>(f.gray.data()));
    std::vector<cv::Point2f> corners;
    bool found = cv::findChessboardCorners(
        img, cv::Size(cols_, rows_), corners,
        cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_NORMALIZE_IMAGE | cv::CALIB_CB_FAST_CHECK);
    if (found) {
        cv::cornerSubPix(img, corners, cv::Size(11, 11), cv::Size(-1, -1),
            cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::COUNT, 30, 0.01));
    }
    std::lock_guard<std::mutex> lk(mtx_);
    img_w_ = f.width; img_h_ = f.height;
    last_corners_ = corners; last_found_ = found;
    return found;
}

int CameraCalibrator::capture() {
    std::lock_guard<std::mutex> lk(mtx_);
    if (last_found_ && (int)last_corners_.size() == cols_ * rows_) {
        image_pts_.push_back(last_corners_);
    }
    return (int)image_pts_.size();
}

CalibResult CameraCalibrator::run() {
    CalibResult r;
    std::lock_guard<std::mutex> lk(mtx_);
    if (image_pts_.size() < 5) { ODYXW("calib", "need >=5 views, have %zu", image_pts_.size()); return r; }

    // object points: planar board, Z=0, in metres.
    std::vector<cv::Point3f> objp;
    for (int y = 0; y < rows_; ++y)
        for (int x = 0; x < cols_; ++x)
            objp.emplace_back(x * (float)square_, y * (float)square_, 0.f);
    std::vector<std::vector<cv::Point3f>> obj_pts(image_pts_.size(), objp);

    cv::Mat K, D;
    std::vector<cv::Mat> rvecs, tvecs;
    double rms = cv::calibrateCamera(obj_pts, image_pts_, cv::Size(img_w_, img_h_),
                                     K, D, rvecs, tvecs);
    r.ok = std::isfinite(rms) && rms < 5.0;
    r.rms = rms; r.n_views = (int)image_pts_.size();
    r.width = img_w_; r.height = img_h_;
    r.fx = K.at<double>(0, 0); r.fy = K.at<double>(1, 1);
    r.cx = K.at<double>(0, 2); r.cy = K.at<double>(1, 2);
    for (int i = 0; i < 5 && i < D.cols * D.rows; ++i) r.dist[i] = D.at<double>(i);
    ODYXI("calib", "calibrated rms=%.3f fx=%.1f cx=%.1f views=%d", rms, r.fx, r.cx, r.n_views);
    return r;
}

void CameraCalibrator::reset() {
    std::lock_guard<std::mutex> lk(mtx_);
    image_pts_.clear(); last_corners_.clear(); last_found_ = false;
}

std::vector<float> CameraCalibrator::cornersFlat() const {
    std::lock_guard<std::mutex> lk(mtx_);
    std::vector<float> out;
    out.push_back(last_found_ ? (float)last_corners_.size() : 0.f);
    if (last_found_) for (auto& c : last_corners_) { out.push_back(c.x); out.push_back(c.y); }
    return out;
}

}  // namespace odyx
