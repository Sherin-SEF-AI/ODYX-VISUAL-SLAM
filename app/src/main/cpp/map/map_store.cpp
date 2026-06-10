#include "map/map_store.h"
#include "common/math_util.h"
#include <set>

namespace odyx {

void MapStore::setVocabulary(std::shared_ptr<DBoW3::Vocabulary> voc) {
    std::lock_guard<std::mutex> lk(mtx_);
    voc_ = std::move(voc);
}
std::shared_ptr<DBoW3::Vocabulary> MapStore::vocab() const {
    std::lock_guard<std::mutex> lk(mtx_);
    return voc_;
}

void MapStore::addKeyframe(KeyframeRecord rec) {
    std::lock_guard<std::mutex> lk(mtx_);
    if (voc_ && !rec.descriptors.empty() && rec.bow.empty()) {
        // 2-arg transform only. The 4-arg (FeatureVector) form trips an OpenCV
        // 4.10 copyTo FIXED_TYPE assertion inside DBoW3, and the FeatureVector is
        // not used downstream (loop/reloc match ORB descriptors directly).
        voc_->transform(rec.descriptors, rec.bow);
    }
    kfs_[rec.idx] = std::move(rec);
}

void MapStore::updateKeyframePose(uint64_t idx, const NavStateOut& p) {
    std::lock_guard<std::mutex> lk(mtx_);
    auto it = kfs_.find(idx);
    if (it != kfs_.end()) it->second.pose = p;
}

std::vector<KeyframeRecord> MapStore::keyframes() const {
    std::lock_guard<std::mutex> lk(mtx_);
    std::vector<KeyframeRecord> out;
    out.reserve(kfs_.size());
    for (auto& kv : kfs_) out.push_back(kv.second);
    return out;
}

bool MapStore::getKeyframe(uint64_t idx, KeyframeRecord& out) const {
    std::lock_guard<std::mutex> lk(mtx_);
    auto it = kfs_.find(idx);
    if (it == kfs_.end()) return false;
    out = it->second; return true;
}

size_t MapStore::size() const {
    std::lock_guard<std::mutex> lk(mtx_);
    return kfs_.size();
}

void MapStore::addLoopEdge(uint64_t a, uint64_t b) {
    std::lock_guard<std::mutex> lk(mtx_);
    edges_.push_back({a, b});
}

void MapStore::setLandmark(uint64_t id, const Vec3& p, float cov_trace) {
    std::lock_guard<std::mutex> lk(mtx_);
    LandmarkOut o; o.id = id; o.p_w = p; o.cov_trace = cov_trace;
    lms_[id] = o;
}

void MapStore::setEnuMarker(const Vec3& enu) {
    std::lock_guard<std::mutex> lk(mtx_);
    enu_markers_.push_back(enu);
    while (enu_markers_.size() > 2000) enu_markers_.erase(enu_markers_.begin());
}

MapSnapshot MapStore::snapshot(const NavStateOut& current,
                               const EnuTransformView& enu) const {
    std::lock_guard<std::mutex> lk(mtx_);
    MapSnapshot s;
    s.current = current;
    s.enu_valid = enu.valid;
    Mat3 R; { const double c=std::cos(enu.yaw), si=std::sin(enu.yaw); R<<c,-si,0, si,c,0, 0,0,1; }
    for (auto& kv : kfs_) {
        s.trajectory_local.push_back(kv.second.pose);
        if (enu.valid)
            s.trajectory_enu.push_back(R*kv.second.pose.p_wb + enu.t);
    }
    for (auto& kv : lms_) s.landmarks.push_back(kv.second);
    s.loop_edges = edges_;
    s.gnss_markers_enu = enu_markers_;
    return s;
}

void MapStore::cull(int max_keyframes) {
    std::lock_guard<std::mutex> lk(mtx_);
    // Keep the most recent keyframes + any involved in a loop edge.
    if ((int)kfs_.size() <= max_keyframes) return;
    std::set<uint64_t> keep_loop;
    for (auto& e : edges_) { keep_loop.insert(e.kf_a); keep_loop.insert(e.kf_b); }
    int to_remove = (int)kfs_.size() - max_keyframes;
    for (auto it = kfs_.begin(); it != kfs_.end() && to_remove > 0;) {
        if (keep_loop.count(it->first)) { ++it; continue; }
        it = kfs_.erase(it); --to_remove;
    }
}

void MapStore::reset() {
    std::lock_guard<std::mutex> lk(mtx_);
    kfs_.clear(); lms_.clear(); edges_.clear(); enu_markers_.clear();
}

}  // namespace odyx
