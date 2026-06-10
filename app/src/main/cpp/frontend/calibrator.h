// ===========================================================================
// ODYX :: in-app camera intrinsic calibration (OpenCV checkerboard).
//
// Fed grayscale frames while calibration mode is active. Detects a chessboard
// each frame (for the live overlay), accumulates user-captured views, and runs
// cv::calibrateCamera to estimate K + radtan distortion. Reused live-camera
// pipeline (no second Camera2 session). Pure OpenCV — no GPL code.
// ===========================================================================
#pragma once
#include "common/odyx_types.h"
#include <opencv2/core.hpp>
#include <mutex>
#include <vector>

namespace odyx {

struct CalibResult {
    bool ok = false;
    double fx = 0, fy = 0, cx = 0, cy = 0;
    std::array<double, 5> dist = {0, 0, 0, 0, 0};
    double rms = 0;          // reprojection error (px)
    int n_views = 0;
    int width = 0, height = 0;
};

class CameraCalibrator {
public:
    void setBoard(int cols, int rows, double square_m) {
        std::lock_guard<std::mutex> lk(mtx_); cols_ = cols; rows_ = rows; square_ = square_m;
    }
    // Detect the board in this frame (for overlay). Returns found.
    bool process(const Frame& f);
    // Capture the current detection as a calibration view. Returns total views.
    int capture();
    // Run calibration over the accumulated views.
    CalibResult run();
    void reset();

    // Packed last-detected corners for the UI overlay: [n, (u,v)*n].
    std::vector<float> cornersFlat() const;
    int viewCount() const { std::lock_guard<std::mutex> lk(mtx_); return (int)image_pts_.size(); }

private:
    mutable std::mutex mtx_;
    int cols_ = 9, rows_ = 6;
    double square_ = 0.025;   // 25 mm default
    int img_w_ = 0, img_h_ = 0;
    std::vector<cv::Point2f> last_corners_;
    bool last_found_ = false;
    std::vector<std::vector<cv::Point2f>> image_pts_;
};

}  // namespace odyx
