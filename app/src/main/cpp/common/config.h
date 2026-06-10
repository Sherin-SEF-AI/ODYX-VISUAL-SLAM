// ODYX :: estimator configuration (loaded from Kotlin/DataStore + calib YAML).
#pragma once
#include "common/odyx_types.h"

namespace odyx {

struct ImuNoise {
    // Continuous-time noise densities (Allan-variance derived).
    double acc_n = 8.0e-3;     // m/s^2 / sqrt(Hz)
    double gyr_n = 1.2e-3;     // rad/s / sqrt(Hz)
    double acc_w = 4.0e-4;     // m/s^3 / sqrt(Hz)  (bias random walk)
    double gyr_w = 2.0e-5;     // rad/s^2 / sqrt(Hz)
    double g = 9.81;
};

struct Extrinsics {
    // T_bc: camera pose in the body(IMU) frame. p_b = R_bc * p_c + t_bc.
    Quat q_bc = Quat::Identity();
    Vec3 t_bc = Vec3::Zero();
    double td = 0.0;           // initial camera<->imu time offset (s)
    bool valid = false;
};

struct Config {
    CameraModel cam;
    ImuNoise imu;
    Extrinsics extr;
    Vec3 gnss_lever_arm = Vec3::Zero();  // GNSS antenna position in body frame

    // Front-end.
    int target_features = 150;
    int grid_cols = 6;
    int grid_rows = 5;
    int min_track_len_kf = 3;
    double ransac_thresh_px = 1.5;
    double kf_parallax_px = 12.0;     // median parallax to trigger keyframe
    double kf_tracked_ratio = 0.5;    // tracked/total below this -> keyframe
    double kf_min_dt_s = 0.10;
    double kf_max_dt_s = 0.50;

    // Back-end.
    int window_keyframes = 12;
    int max_landmarks = 400;
    double huber_px = 1.5;
    bool online_td = true;
    bool online_extr = true;
    bool rolling_shutter = true;

    // Loop closure.
    bool loop_enabled = true;
    double dbow_score_thresh = 0.045;
    int loop_min_inliers = 25;
    int loop_min_kf_gap = 20;

    // GNSS.
    GnssMode gnss_mode = GnssMode::kLoose;
    double gnss_min_hacc_m = 12.0;    // reject fixes worse than this for alignment
    double gnss_match_window_s = 0.35; // max |fix - keyframe| time to form a constraint

    // Map caps.
    int max_keyframes_db = 300;

    // Thermal degradation steps applied to target_features / frame stride.
    int frame_stride = 1;             // process every Nth frame
};

}  // namespace odyx
