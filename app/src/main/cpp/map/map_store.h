// ===========================================================================
// ODYX :: map store (bounded).
//
// Keyframe database (pose, ORB descriptors, DBoW BoW vector, kept landmark ids),
// sparse landmark store, full trajectory (local + ENU), loop edges. Shared by
// the loop-closure and relocalization threads. Thread-safe; snapshots copied.
// ===========================================================================
#pragma once
#include "common/odyx_types.h"
#include "common/snapshot.h"
#include <opencv2/core.hpp>
#include <DBoW3/DBoW3.h>
#include <map>
#include <mutex>
#include <vector>

namespace odyx {

struct KeyframeRecord {
    uint64_t idx = 0;
    TimeNs t_ns = 0;
    NavStateOut pose;                 // current best estimate (local world)
    cv::Mat descriptors;              // ORB, N x 32, CV_8U
    std::vector<cv::KeyPoint> kps;    // pixel coords aligned to descriptors
    std::vector<uint64_t> obs_ids;    // landmark id per descriptor row
    DBoW3::BowVector bow;
    DBoW3::FeatureVector feat;
};

class MapStore {
public:
    // Lightweight ENU view so map_store needn't depend on gnss headers.
    struct EnuTransformView { bool valid=false; double yaw=0; Vec3 t=Vec3::Zero(); };

    MapStore() = default;

    void addKeyframe(KeyframeRecord rec);          // computes BoW if vocab set
    void updateKeyframePose(uint64_t idx, const NavStateOut& p);
    void setVocabulary(std::shared_ptr<DBoW3::Vocabulary> voc);
    std::shared_ptr<DBoW3::Vocabulary> vocab() const;

    // copy out for loop/reloc queries
    std::vector<KeyframeRecord> keyframes() const;
    bool getKeyframe(uint64_t idx, KeyframeRecord& out) const;
    size_t size() const;

    void addLoopEdge(uint64_t a, uint64_t b);
    void setLandmark(uint64_t id, const Vec3& p, float cov_trace);
    void setEnuMarker(const Vec3& enu);

    // Build the UI snapshot (trajectory + landmarks + edges), copied out.
    MapSnapshot snapshot(const NavStateOut& current,
                         const EnuTransformView& enu) const;

    void cull(int max_keyframes);
    void reset();

private:
    mutable std::mutex mtx_;
    std::map<uint64_t, KeyframeRecord> kfs_;
    std::map<uint64_t, LandmarkOut> lms_;
    std::vector<LoopEdge> edges_;
    std::vector<Vec3> enu_markers_;
    std::shared_ptr<DBoW3::Vocabulary> voc_;
};

}  // namespace odyx
