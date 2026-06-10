// ===========================================================================
// ODYX :: Estimator — the orchestrator (owns the native VIO thread).
//
// Pulls frames from SensorSync, runs the front-end, drives initialization, then
// the fixed-lag backend; feeds GNSS constraints; pushes keyframes to the loop
// thread; relocalizes on tracking loss. Exposes pose / telemetry / map snapshot
// to JNI. This is layer C's top — the JNI bridge only forwards to it.
// ===========================================================================
#pragma once
#include "common/odyx_types.h"
#include "common/config.h"
#include "common/snapshot.h"
#include "sync/sensor_sync.h"
#include "frontend/feature_tracker.h"
#include "frontend/calibrator.h"
#include "init/initializer.h"
#include "backend/vio_backend.h"
#include "gnss/gnss_fusion.h"
#include "gnss/coupling.h"
#include "loop/loop_closer.h"
#include "map/map_store.h"
#include "reloc/relocalizer.h"
#include <atomic>
#include <memory>
#include <thread>

namespace odyx {

class Estimator {
public:
    Estimator();
    ~Estimator();

    void loadConfig(const Config& cfg);
    void setVocabulary(std::shared_ptr<DBoW3::Vocabulary> voc);
    void start();
    void stop();
    void reset();

    // producers (called from JNI / Android threads)
    void pushFrame(std::shared_ptr<Frame> f) { sync_.pushFrame(std::move(f)); }
    void pushImu(const ImuSample& s)         { sync_.pushImu(s); }
    void pushGnssFix(const GnssFix& f);
    void pushGnssRaw(const GnssClock& c, const std::vector<GnssRawMeas>& m);
    void pushNavMsg(const GnssNavMsg& n)     { sync_.pushNavMsg(n); }

    // consumers
    NavStateOut pose() const;
    Telemetry telemetry() const;
    MapSnapshot mapSnapshot() const;
    // Packed current-frame features for the SLAM overlay: [u,v,age,isNew]*N.
    std::vector<float> featuresFlat() const;

    // ---- in-app checkerboard calibration (uses the live frame stream) ----
    void setCalibActive(bool on) { calib_active_ = on; }
    void setCalibBoard(int cols, int rows, double square_m) { calib_.setBoard(cols, rows, square_m); }
    int  captureCalibView() { return calib_.capture(); }
    int  calibViewCount() const { return calib_.viewCount(); }
    CalibResult runCalibration() { return calib_.run(); }
    void resetCalibration() { calib_.reset(); }
    std::vector<float> calibCornersFlat() const { return calib_.cornersFlat(); }

    // toggles
    void setGnssMode(GnssMode m);
    void setOnlineCalib(bool td, bool extr);
    void setRollingShutter(bool on);
    void setLoopClosure(bool on);
    void setThermalStatus(int level);   // from Android; degrades front-end

private:
    void run();                          // VIO thread loop
    void processFrame(const std::shared_ptr<Frame>& f);
    void handleKeyframe(const TrackResult& tr, const std::shared_ptr<Frame>& f);
    void feedGnss(uint64_t kf_idx, TimeNs kf_t);
    void tryInitialize(const TrackResult& tr, const std::shared_ptr<Frame>& f);
    void buildKeyframeRecord(const TrackResult& tr, const std::shared_ptr<Frame>& f,
                             uint64_t idx);
    std::map<uint64_t, Vec2> imageVelocities(const TrackResult& tr);
    void applyThermal();

    Config cfg_;
    SensorSync sync_;
    std::unique_ptr<FeatureTracker> front_;
    std::unique_ptr<Initializer> init_;
    std::unique_ptr<VioBackend> backend_;
    std::unique_ptr<EnuAligner> enu_;
    std::unique_ptr<GnssCoupling> coupling_;
    LooseCoupling* loose_ = nullptr;     // non-owning view when mode==LOOSE
    TightCoupling* tight_ = nullptr;     // non-owning view when mode==TIGHT
    std::unique_ptr<MapStore> map_;
    std::unique_ptr<LoopCloser> loop_;
    std::unique_ptr<Relocalizer> reloc_;

    std::thread th_;
    std::atomic<bool> running_{false};
    std::atomic<int> thermal_{0};
    std::atomic<bool> calib_active_{false};
    mutable CameraCalibrator calib_;

    TrackResult prev_track_;
    TimeNs last_kf_t_ = 0;
    TimeNs last_log_ns_ = 0;     // throttle diagnostic logging (~1 Hz)
    uint64_t boot_kf_counter_ = 0;
    std::vector<KeyframeMeta> boot_kfs_;

    mutable std::mutex state_mtx_;
    NavStateOut pose_;
    Telemetry tele_;
    std::vector<std::array<float,4>> last_feats_;   // u,v,age,isNew
    TrackingState tstate_ = TrackingState::kUninitialized;
    InitState istate_ = InitState::kCollecting;
    int lost_streak_ = 0;
    int loop_count_ = 0;
    GnssMode gnss_mode_ = GnssMode::kLoose;
    int last_gnss_sats_ = 0, last_gnss_fix_ = 0;
    double last_gnss_acc_ = 0;
    double drift_proxy_ = 0;

    void rebuildCoupling();
};

}  // namespace odyx
