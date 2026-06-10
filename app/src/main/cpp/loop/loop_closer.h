// ===========================================================================
// ODYX :: loop closure (async thread).
//
// On each new keyframe pushed by the VIO thread: DBoW3 query against older
// keyframes -> best candidate -> ORB descriptor matching -> PnP+RANSAC
// geometric verification -> relative-pose constraint -> GTSAM pose-graph
// optimization over keyframe poses -> corrections handed back to the backend
// and the ENU aligner re-anchored.
//
// Bounded revisit frequency (min keyframe gap, cooldown). Verify before commit.
// ===========================================================================
#pragma once
#include "common/odyx_types.h"
#include "common/config.h"
#include "map/map_store.h"
#include <gtsam/geometry/Pose3.h>
#include <atomic>
#include <condition_variable>
#include <deque>
#include <functional>
#include <mutex>
#include <thread>

namespace odyx {

struct LoopResult {
    bool found = false;
    uint64_t kf_query = 0, kf_match = 0;
    int inliers = 0;
    double score = 0;
    std::map<uint64_t, gtsam::Pose3> corrected;   // idx -> corrected pose
};

class LoopCloser {
public:
    using CommitFn = std::function<void(const LoopResult&)>;

    LoopCloser(const Config& cfg, MapStore& map);
    ~LoopCloser();

    void start(CommitFn on_commit);
    void stop();

    // VIO thread enqueues a keyframe idx to test (non-blocking).
    void enqueue(uint64_t kf_idx);

    int loopCount() const { return loop_count_.load(); }
    void setEnabled(bool e) { enabled_ = e; }

private:
    void run();
    bool detect(uint64_t kf_idx, LoopResult& out);
    bool verifyAndSolve(const KeyframeRecord& q, const KeyframeRecord& m,
                        gtsam::Pose3& T_m_q, int& inliers);
    void optimizePoseGraph(const std::vector<KeyframeRecord>& kfs,
                           uint64_t qi, uint64_t mi, const gtsam::Pose3& T_m_q,
                           LoopResult& out);

    Config cfg_;
    MapStore& map_;
    CommitFn commit_;

    std::thread th_;
    std::mutex mtx_;
    std::condition_variable cv_;
    std::deque<uint64_t> queue_;
    std::atomic<bool> running_{false};
    std::atomic<bool> enabled_{true};
    std::atomic<int> loop_count_{0};
    uint64_t last_loop_kf_ = 0;
};

}  // namespace odyx
