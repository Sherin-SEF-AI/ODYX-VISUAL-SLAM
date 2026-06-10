// ===========================================================================
// ODYX :: relocalization.
//
// On tracking loss, query the keyframe DB by DBoW3 for the most similar
// keyframe, match ORB descriptors, and solve PnP+RANSAC of the current frame's
// features against the matched keyframe's landmarks to recover the camera pose
// in the existing map. If it fails repeatedly, the caller starts a new map
// segment and stitches later when relocalization succeeds.
// ===========================================================================
#pragma once
#include "common/odyx_types.h"
#include "common/config.h"
#include "map/map_store.h"

namespace odyx {

struct RelocResult {
    bool ok = false;
    uint64_t matched_kf = 0;
    NavStateOut pose;     // recovered body pose in world (local)
    int inliers = 0;
};

class Relocalizer {
public:
    Relocalizer(const Config& cfg, MapStore& map) : cfg_(cfg), map_(map) {}

    // query: ORB descriptors + keypoints of the current frame, plus a map from
    // descriptor row -> landmark world position is NOT required; we PnP against
    // the matched keyframe's landmark world positions supplied via `landmarks`.
    RelocResult relocalize(const cv::Mat& desc, const std::vector<cv::KeyPoint>& kps,
                           const std::map<uint64_t, Vec3>& world_landmarks);

private:
    Config cfg_;
    MapStore& map_;
};

}  // namespace odyx
