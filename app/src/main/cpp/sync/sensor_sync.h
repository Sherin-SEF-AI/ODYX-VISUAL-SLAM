// ===========================================================================
// ODYX :: SensorSync & buffers.
//
// Thread-safe ingestion of the three sensor streams on the unified time base:
//   - IMU ring  : FULL RATE, never dropped (preintegration needs every sample).
//   - Frame slot: latest-wins (the tracker only needs the newest frame).
//   - GNSS queue: bounded FIFO of fixes / raw batches.
//
// Provides imuBetween(t0, t1): the contiguous IMU samples covering [t0, t1],
// with endpoint interpolation, for keyframe-to-keyframe preintegration.
// ===========================================================================
#pragma once
#include "common/odyx_types.h"
#include <deque>
#include <mutex>
#include <memory>
#include <optional>

namespace odyx {

class SensorSync {
public:
    explicit SensorSync(size_t imu_capacity = 8192, size_t gnss_capacity = 256);

    // --- producers (called from Android sensor threads via JNI) ---
    void pushImu(const ImuSample& s);
    void pushFrame(std::shared_ptr<Frame> f);   // takes the latest
    void pushGnssFix(const GnssFix& fix);
    void pushGnssRaw(const GnssClock& clk, const std::vector<GnssRawMeas>& meas);
    void pushNavMsg(const GnssNavMsg& nav);

    // --- consumer (VIO thread) ---
    // Returns the newest unconsumed frame, or nullptr. Marks it consumed.
    std::shared_ptr<Frame> takeFrame();
    bool hasFrame() const;

    // IMU samples covering [t0_ns, t1_ns], endpoints linearly interpolated.
    // Empty if the buffer does not yet span t1 (caller should wait).
    std::vector<ImuSample> imuBetween(TimeNs t0, TimeNs t1) const;

    // Newest / oldest buffered IMU time (0 if empty).
    TimeNs latestImuTime() const;
    TimeNs oldestImuTime() const;

    // Pop all queued GNSS fixes / raw batches / nav messages.
    std::vector<GnssFix> drainFixes();
    std::vector<std::pair<GnssClock, std::vector<GnssRawMeas>>> drainRaw();
    std::vector<GnssNavMsg> drainNav();

    // Measured rates (Hz) over a sliding ~1s window, for telemetry.
    double imuRate() const;
    double frameRate() const;
    double gnssRate() const;

    void reset();

private:
    static ImuSample interp(const ImuSample& a, const ImuSample& b, TimeNs t);

    mutable std::mutex imu_mtx_;
    std::deque<ImuSample> imu_;
    size_t imu_cap_;

    mutable std::mutex frame_mtx_;
    std::shared_ptr<Frame> frame_;     // latest-wins slot
    uint64_t frame_seq_in_ = 0;
    uint64_t frame_seq_consumed_ = 0;

    mutable std::mutex gnss_mtx_;
    std::deque<GnssFix> fixes_;
    std::deque<std::pair<GnssClock, std::vector<GnssRawMeas>>> raw_;
    std::deque<GnssNavMsg> nav_;
    size_t gnss_cap_;

    // rate estimation (timestamps of recent arrivals)
    mutable std::mutex rate_mtx_;
    std::deque<TimeNs> imu_arr_, frame_arr_, gnss_arr_;
    static double rateOf(const std::deque<TimeNs>& q);
    static void noteArrival(std::deque<TimeNs>& q, TimeNs t);
};

}  // namespace odyx
