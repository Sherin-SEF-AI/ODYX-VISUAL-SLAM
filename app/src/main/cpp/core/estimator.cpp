#include "core/estimator.h"
#include "common/math_util.h"
#include "common/time.h"
#include "common/log.h"
#include <chrono>

namespace odyx {

Estimator::Estimator() {
    map_ = std::make_unique<MapStore>();
    enu_ = std::make_unique<EnuAligner>(cfg_);
}
Estimator::~Estimator() { stop(); }

void Estimator::loadConfig(const Config& cfg) {
    std::lock_guard<std::mutex> lk(state_mtx_);
    cfg_ = cfg;
    gnss_mode_ = cfg.gnss_mode;
    if (front_) front_->setConfig(cfg_);
}

void Estimator::setVocabulary(std::shared_ptr<DBoW3::Vocabulary> voc) {
    map_->setVocabulary(std::move(voc));
}

void Estimator::rebuildCoupling() {
    loose_ = nullptr; tight_ = nullptr;
    if (gnss_mode_ == GnssMode::kTight) {
        auto t = std::make_unique<TightCoupling>(cfg_, *enu_);
        tight_ = t.get(); coupling_ = std::move(t);
    } else if (gnss_mode_ == GnssMode::kLoose) {
        auto l = std::make_unique<LooseCoupling>(cfg_, *enu_);
        loose_ = l.get(); coupling_ = std::move(l);
    } else {
        coupling_.reset();
    }
}

void Estimator::start() {
    if (running_) return;
    front_   = std::make_unique<FeatureTracker>(cfg_);
    init_    = std::make_unique<Initializer>(cfg_);
    backend_ = std::make_unique<VioBackend>(cfg_);
    reloc_   = std::make_unique<Relocalizer>(cfg_, *map_);
    rebuildCoupling();
    loop_    = std::make_unique<LoopCloser>(cfg_, *map_);
    loop_->setEnabled(cfg_.loop_enabled);
    loop_->start([this](const LoopResult& r){
        backend_->applyPoseGraphCorrection(r.corrected);
        for (auto& kv : r.corrected) {
            NavStateOut p; p.q_wb = Quat(kv.second.rotation().matrix());
            p.p_wb = kv.second.translation(); p.valid = true;
            map_->updateKeyframePose(kv.first, p);
        }
        std::lock_guard<std::mutex> lk(state_mtx_); loop_count_ = loop_->loopCount();
    });

    running_ = true;
    th_ = std::thread(&Estimator::run, this);
    ODYXI("est", "started");
}

void Estimator::stop() {
    running_ = false;
    if (th_.joinable()) th_.join();
    if (loop_) loop_->stop();
}

void Estimator::reset() {
    bool was = running_;
    stop();
    sync_.reset();
    if (map_) map_->reset();
    if (enu_) enu_->reset();
    {
        std::lock_guard<std::mutex> lk(state_mtx_);
        pose_ = NavStateOut(); tele_ = Telemetry();
        tstate_ = TrackingState::kUninitialized; istate_ = InitState::kCollecting;
        boot_kfs_.clear(); boot_kf_counter_ = 0; last_kf_t_ = 0; lost_streak_ = 0;
    }
    if (was) start();
}

void Estimator::pushGnssFix(const GnssFix& f) {
    sync_.pushGnssFix(f);
    std::lock_guard<std::mutex> lk(state_mtx_);
    last_gnss_sats_ = f.n_sats; last_gnss_fix_ = f.valid ? 1 : 0; last_gnss_acc_ = f.hAcc_m;
}
void Estimator::pushGnssRaw(const GnssClock& c, const std::vector<GnssRawMeas>& m) {
    sync_.pushGnssRaw(c, m);
}

// --- VIO thread ------------------------------------------------------------
void Estimator::run() {
    int stride_counter = 0;
    while (running_) {
        if (!sync_.hasFrame()) {
            std::this_thread::sleep_for(std::chrono::milliseconds(2));
            continue;
        }
        auto f = sync_.takeFrame();
        if (!f) continue;
        if (++stride_counter < cfg_.frame_stride) continue;
        stride_counter = 0;
        applyThermal();
        try {
            processFrame(f);
        } catch (const std::exception& e) {
            ODYXE("vio", "processFrame threw (recovered): %s", e.what());
        }
    }
}

void Estimator::applyThermal() {
    int level = thermal_.load();
    std::lock_guard<std::mutex> lk(state_mtx_);
    // degrade target feature count / frame stride under thermal pressure
    int base_feat = 150;
    switch (level) {
        case 0: cfg_.target_features = base_feat; cfg_.frame_stride = 1; break;
        case 1: cfg_.target_features = 120; cfg_.frame_stride = 1; break;
        case 2: cfg_.target_features = 90;  cfg_.frame_stride = 2; break;
        default: cfg_.target_features = 60; cfg_.frame_stride = 3; break;
    }
    if (front_) front_->setConfig(cfg_);
}

void Estimator::processFrame(const std::shared_ptr<Frame>& f) {
    // In checkerboard-calibration mode, route frames to the calibrator only
    // (the user is pointing at a board, not navigating) and skip VIO.
    if (calib_active_.load()) { calib_.process(*f); return; }

    using clock = std::chrono::steady_clock;
    auto t0 = clock::now();
    TrackResult tr = front_->track(*f);
    double frontend_ms = std::chrono::duration<double,std::milli>(clock::now()-t0).count();

    {
        std::lock_guard<std::mutex> lk(state_mtx_);
        last_feats_.clear();
        last_feats_.reserve(tr.obs.size());
        for (const auto& o : tr.obs)
            last_feats_.push_back({(float)o.uv.x(), (float)o.uv.y(),
                                   (float)o.age, o.is_new ? 1.f : 0.f});
        tele_.frontend_ms = frontend_ms;
        tele_.n_tracked = tr.n_tracked;
        tele_.imu_rate = sync_.imuRate();
        tele_.frame_rate = sync_.frameRate();
        tele_.gnss_rate = sync_.gnssRate();
        tele_.target_features = cfg_.target_features;
        tele_.frame_stride = cfg_.frame_stride;
        tele_.thermal_status = thermal_.load();
        tele_.calib_default = !cfg_.cam.valid || !cfg_.extr.valid;
    }

    // ~1 Hz diagnostic line so live walks / replays are debuggable from logcat.
    if (f->t_ns - last_log_ns_ > (TimeNs)1e9) {
        last_log_ns_ = f->t_ns;
        ODYXI("vio", "track=%d init=%d feat=%d kf=%d lm=%d par=%.1f td=%.4f opt=%.1fms imu=%.0f",
              (int)tstate_, (int)istate_, tr.n_tracked, (int)tele_.n_keyframes,
              (int)tele_.n_landmarks, tr.median_parallax,
              backend_ ? backend_->tdEstimate() : 0.0, tele_.optimize_ms, sync_.imuRate());
    }

    if (!tr.is_keyframe) return;

    if (tstate_ == TrackingState::kUninitialized || tstate_ == TrackingState::kInitializing) {
        tryInitialize(tr, f);
    } else {
        handleKeyframe(tr, f);
    }
    front_->markKeyframeTaken(tr);
    prev_track_ = tr;
    last_kf_t_ = tr.t_ns;
}

std::map<uint64_t, Vec2> Estimator::imageVelocities(const TrackResult& tr) {
    // normalized image-plane velocity per feature = (undist_now - undist_prev)/dt
    std::map<uint64_t, Vec2> vel;
    if (prev_track_.t_ns == 0) return vel;
    const double dt = ns_to_s(tr.t_ns - prev_track_.t_ns);
    if (dt <= 0) return vel;
    std::map<uint64_t, Vec2> prev;
    for (const auto& o : prev_track_.obs) prev[o.id] = o.undist;
    for (const auto& o : tr.obs) {
        auto it = prev.find(o.id);
        if (it != prev.end()) vel[o.id] = (o.undist - it->second) / dt;
    }
    return vel;
}

void Estimator::tryInitialize(const TrackResult& tr, const std::shared_ptr<Frame>& f) {
    tstate_ = TrackingState::kInitializing;
    // preintegration from previous boot keyframe to this one
    std::shared_ptr<Preintegrator> pim;
    if (last_kf_t_ != 0) {
        // apply current td estimate to align camera time to the IMU timeline
        const double td = cfg_.extr.td;
        TimeNs a = last_kf_t_ + s_to_ns(td);
        TimeNs b = tr.t_ns + s_to_ns(td);
        auto samples = sync_.imuBetween(a, b);
        if (samples.size() >= 2) {
            pim = std::make_shared<Preintegrator>(cfg_.imu, Vec3::Zero(), Vec3::Zero());
            pim->integrateSamples(samples);
        } else {
            return;   // wait for IMU to span this interval
        }
    }

    BootKf bk;
    bk.t_ns = tr.t_ns;
    bk.obs = tr.obs;
    bk.preint = pim;
    InitResult r = init_->addKeyframe(bk);

    KeyframeMeta km; km.idx = ++boot_kf_counter_; km.t_ns = tr.t_ns; km.obs = tr.obs;
    boot_kfs_.push_back(km);

    {
        std::lock_guard<std::mutex> lk(state_mtx_);
        istate_ = r.state;
        tele_.init_state = (int)r.state;
        tele_.gravity_align_resid = r.gravity_resid;
    }
    // log the init attempt outcome (boot-keyframe rate, low) so failures are visible
    ODYXI("init", "boot=%zu state=%d scale=%.3f gres=%.3f reason='%s'",
          init_->size(), (int)r.state, r.scale, r.gravity_resid, r.reason.c_str());

    if (r.ok) {
        // Match each InitResult state to its boot keyframe BY TIMESTAMP (the
        // initializer slides its own window, so positional offset is unreliable);
        // each state already carries t_ns. Assign fresh monotonic indices that
        // the backend uses as GTSAM keys and the map uses as keyframe ids.
        std::map<TimeNs, std::vector<FeatureObs>> obsByT;
        for (auto& bk : boot_kfs_) obsByT[bk.t_ns] = bk.obs;

        std::vector<KeyframeMeta> seeded;
        uint64_t nextIdx = 0;
        for (size_t i = 0; i < r.states.size(); ++i) {
            KeyframeMeta m;
            m.idx = ++nextIdx;
            m.t_ns = r.states[i].t_ns;
            m.state = r.states[i];
            auto it = obsByT.find(m.t_ns);
            if (it != obsByT.end()) m.obs = it->second;
            seeded.push_back(m);
        }
        // Preintegrate IMU between consecutive seeded keyframes so the backend
        // can add CombinedImuFactors (constrains bootstrap velocities/biases).
        std::vector<std::shared_ptr<Preintegrator>> imu_between;
        for (size_t i = 0; i + 1 < seeded.size(); ++i) {
            const double td = cfg_.extr.td;
            auto samples = sync_.imuBetween(seeded[i].t_ns + s_to_ns(td),
                                            seeded[i+1].t_ns + s_to_ns(td));
            std::shared_ptr<Preintegrator> p;
            if (samples.size() >= 2) {
                p = std::make_shared<Preintegrator>(cfg_.imu, Vec3::Zero(), r.bg);
                p->integrateSamples(samples);
            }
            imu_between.push_back(p);
        }
        backend_->initializeFromResult(r, seeded, imu_between);
        boot_kf_counter_ = seeded.back().idx;
        last_kf_t_ = seeded.back().t_ns;

        // Only the latest boot keyframe has its image buffered, so build the
        // loop-closure/reloc map record for it (the current frame f). Earlier
        // boot keyframes enter the map naturally as new keyframes arrive.
        buildKeyframeRecord(tr, f, seeded.back().idx);
        map_->updateKeyframePose(seeded.back().idx, backend_->current());
        {
            std::lock_guard<std::mutex> lk(state_mtx_);
            tstate_ = TrackingState::kNominal;
            istate_ = InitState::kDone;
        }
        ODYXI("est", "init OK scale=%.3f gres=%.3f kfs=%zu",
              r.scale, r.gravity_resid, seeded.size());
    }
}

void Estimator::handleKeyframe(const TrackResult& tr, const std::shared_ptr<Frame>& f) {
    // preintegration since last keyframe over the IMU timeline (td-aligned)
    // tdEstimate() is the ABSOLUTE cam<->imu offset (the kD state is seeded at
    // cfg_.extr.td and optimized), so use it directly — adding cfg_.extr.td
    // again would double-count.
    const double td = backend_->tdEstimate();
    TimeNs a = last_kf_t_ + s_to_ns(td);
    TimeNs b = tr.t_ns + s_to_ns(td);
    auto samples = sync_.imuBetween(a, b);
    if (samples.size() < 2) return;
    auto pim = std::make_shared<Preintegrator>(cfg_.imu,
                   backend_->accelBias(), backend_->gyroBias());
    pim->integrateSamples(samples);

    auto vel = imageVelocities(tr);
    double opt_ms = 0;
    bool ok = backend_->addKeyframe(tr, pim, vel, opt_ms);
    uint64_t idx = backend_->latestKeyframeIdx();

    NavStateOut cur = backend_->current();
    {
        std::lock_guard<std::mutex> lk(state_mtx_);
        pose_ = cur;
        tele_.optimize_ms = opt_ms;
        tele_.n_keyframes = backend_->windowSize();
        tele_.window_size = cfg_.window_keyframes;
        tele_.n_landmarks = backend_->landmarkCount();
        tele_.td = backend_->tdEstimate();   // absolute cam<->imu offset (s)
        tele_.bg = backend_->gyroBias();
        tele_.ba = backend_->accelBias();
        tele_.vel = backend_->velocity();
        tele_.loop_count = loop_count_;
    }

    if (!ok || backend_->diverged()) {
        ++lost_streak_;
        std::lock_guard<std::mutex> lk(state_mtx_);
        tstate_ = TrackingState::kUnstable;
        if (lost_streak_ >= 3) {
            tstate_ = TrackingState::kLost;
            // tracking loss: try relocalization via map, else re-initialize.
            std::vector<cv::KeyPoint> kps; cv::Mat desc; std::vector<uint64_t> ids;
            front_->computeKeyframeOrb(*f, tr.obs, kps, desc, ids);
            std::vector<KeyframeMeta> kf; std::vector<LandmarkOut> lm;
            backend_->snapshot(kf, lm);
            std::map<uint64_t, Vec3> wl; for (auto& l : lm) wl[l.id] = l.p_w;
            RelocResult rr = reloc_->relocalize(desc, kps, wl);
            if (rr.ok) { pose_ = rr.pose; tstate_ = TrackingState::kNominal; lost_streak_ = 0; }
            else {
                // start a new map segment: reset front-end, initializer AND the
                // diverged backend so a fresh init can cleanly re-seed it.
                front_->reset(); init_->reset(); backend_->reset(); boot_kfs_.clear();
                boot_kf_counter_ = idx;   // continue ids monotonically
                last_kf_t_ = 0;
                tstate_ = TrackingState::kUninitialized; istate_ = InitState::kCollecting;
            }
        }
        return;
    }
    lost_streak_ = 0;
    { std::lock_guard<std::mutex> lk(state_mtx_); tstate_ = TrackingState::kNominal; }

    // map keyframe record + loop closure enqueue
    buildKeyframeRecord(tr, f, idx);
    map_->updateKeyframePose(idx, cur);
    if (cfg_.loop_enabled) loop_->enqueue(idx);
    map_->cull(cfg_.max_keyframes_db);

    // GNSS fusion
    feedGnss(idx, tr.t_ns);
}

void Estimator::buildKeyframeRecord(const TrackResult& tr, const std::shared_ptr<Frame>& f,
                                    uint64_t idx) {
    KeyframeRecord rec;
    rec.idx = idx; rec.t_ns = tr.t_ns; rec.pose = backend_->current();
    std::vector<uint64_t> ids;
    front_->computeKeyframeOrb(*f, tr.obs, rec.kps, rec.descriptors, ids);
    rec.obs_ids = ids;
    map_->addKeyframe(std::move(rec));
}

void Estimator::feedGnss(uint64_t kf_idx, TimeNs kf_t) {
    if (gnss_mode_ == GnssMode::kOff) return;
    // ingest queued data into the active coupling
    auto fixes = sync_.drainFixes();
    auto raws  = sync_.drainRaw();
    auto navs  = sync_.drainNav();

    NavStateOut cur = backend_->current();

    // Ingest fixes/raw into the active coupling. tight_->addFix sets the ENU
    // origin via its loose fallback, so origin gets established in either mode.
    if (loose_) for (auto& fx : fixes) loose_->addFix(fx);
    if (tight_) {
        for (auto& n : navs) tight_->addNav(n);
        for (auto& r : raws) tight_->addRaw(r.first, r.second);
        for (auto& fx : fixes) tight_->addFix(fx);
    }

    // Feed ENU<->local alignment correspondences from the fused fixes. BOTH modes
    // need this — previously only LOOSE fed them, so TIGHT never aligned and its
    // constraints (which require a valid transform) were never applied.
    // (Lever arm ~cm is negligible here vs metre-level GNSS noise; it is applied
    // exactly at the position factor below.)
    if (enu_->hasOrigin()) {
        for (auto& fx : fixes) {
            if (!fx.valid || fx.hAcc_m > cfg_.gnss_min_hacc_m * 2.5) continue;
            Vec3 e = enu_->fixToEnu(fx);
            enu_->addCorrespondence(cur.p_wb, e, 1.0 / std::max(1.0, fx.hAcc_m));
            map_->setEnuMarker(e);
        }
    }

    if (!coupling_) return;
    auto c = coupling_->constraintFor(kf_t);
    if (c && c->valid) {
        // The GNSS solution is the ANTENNA phase-center position; the graph
        // state X(kf) is the BODY(IMU). Convert antenna -> body using the lever
        // arm (antenna position in the body frame):
        //   p_antenna = p_body + R_wb * lever_arm  =>  p_body = p_antenna - R_wb*lever
        const Vec3 p_body = c->p_world - cur.q_wb.toRotationMatrix() * cfg_.gnss_lever_arm;
        backend_->addGnssPositionFactor(kf_idx, p_body, c->std_m);
        // re-anchor correspondence for the aligner & drift proxy
        if (enu_->hasOrigin() && enu_->transform().valid) {
            Vec3 enu_from_vio = enu_->transform().localToEnu(cur.p_wb);
            Vec3 enu_from_gnss = enu_->transform().localToEnu(c->p_world);
            drift_proxy_ = (cur.p_wb - c->p_world).norm();
            (void)enu_from_vio; (void)enu_from_gnss;
        }
        std::lock_guard<std::mutex> lk(state_mtx_);
        tele_.gnss_n_sats = c->n_sats; tele_.gnss_fix_type = c->fix_type;
        tele_.gnss_acc_m = c->std_m; tele_.enu_aligned = enu_->transform().valid;
        tele_.enu_resid_m = enu_->transform().resid_m;
        tele_.drift_proxy_m = drift_proxy_;
    }
}

// --- consumers -------------------------------------------------------------
NavStateOut Estimator::pose() const {
    std::lock_guard<std::mutex> lk(state_mtx_); return pose_;
}
Telemetry Estimator::telemetry() const {
    std::lock_guard<std::mutex> lk(state_mtx_);
    Telemetry t = tele_;
    t.tracking_state = (int)tstate_; t.init_state = (int)istate_;
    t.gnss_mode = (int)gnss_mode_;
    return t;
}
std::vector<float> Estimator::featuresFlat() const {
    std::lock_guard<std::mutex> lk(state_mtx_);
    std::vector<float> out;
    out.reserve(1 + last_feats_.size() * 4);
    out.push_back((float)last_feats_.size());
    for (const auto& f : last_feats_) { out.push_back(f[0]); out.push_back(f[1]); out.push_back(f[2]); out.push_back(f[3]); }
    return out;
}

MapSnapshot Estimator::mapSnapshot() const {
    MapStore::EnuTransformView v;
    if (enu_) { v.valid = enu_->transform().valid; v.yaw = enu_->transform().yaw; v.t = enu_->transform().t; }
    return map_->snapshot(pose(), v);
}

void Estimator::setGnssMode(GnssMode m) {
    std::lock_guard<std::mutex> lk(state_mtx_);
    gnss_mode_ = m; cfg_.gnss_mode = m;
    rebuildCoupling();
}
void Estimator::setOnlineCalib(bool td, bool extr) {
    std::lock_guard<std::mutex> lk(state_mtx_); cfg_.online_td = td; cfg_.online_extr = extr;
}
void Estimator::setRollingShutter(bool on) {
    std::lock_guard<std::mutex> lk(state_mtx_); cfg_.rolling_shutter = on;
}
void Estimator::setLoopClosure(bool on) {
    std::lock_guard<std::mutex> lk(state_mtx_); cfg_.loop_enabled = on;
    if (loop_) loop_->setEnabled(on);
}
void Estimator::setThermalStatus(int level) { thermal_.store(level); }

}  // namespace odyx
