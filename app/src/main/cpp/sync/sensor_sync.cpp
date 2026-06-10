#include "sync/sensor_sync.h"
#include "common/log.h"
#include <algorithm>

namespace odyx {

SensorSync::SensorSync(size_t imu_capacity, size_t gnss_capacity)
    : imu_cap_(imu_capacity), gnss_cap_(gnss_capacity) {}

ImuSample SensorSync::interp(const ImuSample& a, const ImuSample& b, TimeNs t) {
    if (b.t_ns == a.t_ns) return a;
    const double u = static_cast<double>(t - a.t_ns) / static_cast<double>(b.t_ns - a.t_ns);
    ImuSample s;
    s.t_ns = t;
    s.acc = a.acc + (b.acc - a.acc) * u;
    s.gyr = a.gyr + (b.gyr - a.gyr) * u;
    s.acc_bias_hint = a.acc_bias_hint;
    s.gyr_bias_hint = a.gyr_bias_hint;
    return s;
}

void SensorSync::pushImu(const ImuSample& s) {
    {
        std::lock_guard<std::mutex> lk(imu_mtx_);
        // Drop out-of-order duplicates (some HALs replay the last sample).
        if (!imu_.empty() && s.t_ns <= imu_.back().t_ns) return;
        imu_.push_back(s);
        while (imu_.size() > imu_cap_) imu_.pop_front();
    }
    std::lock_guard<std::mutex> lk(rate_mtx_);
    noteArrival(imu_arr_, s.t_ns);
}

void SensorSync::pushFrame(std::shared_ptr<Frame> f) {
    if (!f) return;
    {
        std::lock_guard<std::mutex> lk(frame_mtx_);
        frame_ = std::move(f);
        frame_->seq = ++frame_seq_in_;
    }
    std::lock_guard<std::mutex> lk(rate_mtx_);
    noteArrival(frame_arr_, frame_ ? frame_->t_ns : 0);
}

std::shared_ptr<Frame> SensorSync::takeFrame() {
    std::lock_guard<std::mutex> lk(frame_mtx_);
    if (!frame_ || frame_->seq == frame_seq_consumed_) return nullptr;
    frame_seq_consumed_ = frame_->seq;
    return frame_;
}

bool SensorSync::hasFrame() const {
    std::lock_guard<std::mutex> lk(frame_mtx_);
    return frame_ && frame_->seq != frame_seq_consumed_;
}

void SensorSync::pushGnssFix(const GnssFix& fix) {
    {
        std::lock_guard<std::mutex> lk(gnss_mtx_);
        fixes_.push_back(fix);
        while (fixes_.size() > gnss_cap_) fixes_.pop_front();
    }
    std::lock_guard<std::mutex> lk(rate_mtx_);
    noteArrival(gnss_arr_, fix.t_ns);
}

void SensorSync::pushGnssRaw(const GnssClock& clk, const std::vector<GnssRawMeas>& meas) {
    std::lock_guard<std::mutex> lk(gnss_mtx_);
    raw_.emplace_back(clk, meas);
    while (raw_.size() > gnss_cap_) raw_.pop_front();
}

void SensorSync::pushNavMsg(const GnssNavMsg& nav) {
    std::lock_guard<std::mutex> lk(gnss_mtx_);
    nav_.push_back(nav);
    while (nav_.size() > gnss_cap_) nav_.pop_front();
}

std::vector<ImuSample> SensorSync::imuBetween(TimeNs t0, TimeNs t1) const {
    std::vector<ImuSample> out;
    if (t1 <= t0) return out;
    std::lock_guard<std::mutex> lk(imu_mtx_);
    if (imu_.size() < 2) return out;
    if (imu_.front().t_ns > t0 || imu_.back().t_ns < t1) return out;  // not yet spanning

    // Find first sample >= t0.
    auto it = std::lower_bound(imu_.begin(), imu_.end(), t0,
        [](const ImuSample& s, TimeNs t){ return s.t_ns < t; });

    // Interpolated start endpoint.
    if (it == imu_.begin()) {
        out.push_back(*it);
    } else {
        out.push_back(interp(*(it - 1), *it, t0));
    }
    // Interior samples strictly inside (t0, t1).
    for (; it != imu_.end() && it->t_ns < t1; ++it) {
        if (it->t_ns > t0) out.push_back(*it);
    }
    // Interpolated end endpoint.
    if (it != imu_.end()) {
        out.push_back(interp(*(it - 1), *it, t1));
    }
    return out;
}

TimeNs SensorSync::latestImuTime() const {
    std::lock_guard<std::mutex> lk(imu_mtx_);
    return imu_.empty() ? 0 : imu_.back().t_ns;
}
TimeNs SensorSync::oldestImuTime() const {
    std::lock_guard<std::mutex> lk(imu_mtx_);
    return imu_.empty() ? 0 : imu_.front().t_ns;
}

std::vector<GnssFix> SensorSync::drainFixes() {
    std::lock_guard<std::mutex> lk(gnss_mtx_);
    std::vector<GnssFix> out(fixes_.begin(), fixes_.end());
    fixes_.clear();
    return out;
}
std::vector<std::pair<GnssClock, std::vector<GnssRawMeas>>> SensorSync::drainRaw() {
    std::lock_guard<std::mutex> lk(gnss_mtx_);
    std::vector<std::pair<GnssClock, std::vector<GnssRawMeas>>> out(raw_.begin(), raw_.end());
    raw_.clear();
    return out;
}
std::vector<GnssNavMsg> SensorSync::drainNav() {
    std::lock_guard<std::mutex> lk(gnss_mtx_);
    std::vector<GnssNavMsg> out(nav_.begin(), nav_.end());
    nav_.clear();
    return out;
}

void SensorSync::noteArrival(std::deque<TimeNs>& q, TimeNs t) {
    q.push_back(t);
    const TimeNs cutoff = t - s_to_ns(1.0);
    while (!q.empty() && q.front() < cutoff) q.pop_front();
}
double SensorSync::rateOf(const std::deque<TimeNs>& q) {
    if (q.size() < 2) return 0.0;
    const double span = ns_to_s(q.back() - q.front());
    return span > 1e-6 ? (q.size() - 1) / span : 0.0;
}
double SensorSync::imuRate()   const { std::lock_guard<std::mutex> lk(rate_mtx_); return rateOf(imu_arr_); }
double SensorSync::frameRate() const { std::lock_guard<std::mutex> lk(rate_mtx_); return rateOf(frame_arr_); }
double SensorSync::gnssRate()  const { std::lock_guard<std::mutex> lk(rate_mtx_); return rateOf(gnss_arr_); }

void SensorSync::reset() {
    { std::lock_guard<std::mutex> lk(imu_mtx_);  imu_.clear(); }
    { std::lock_guard<std::mutex> lk(frame_mtx_); frame_.reset(); frame_seq_in_ = frame_seq_consumed_ = 0; }
    { std::lock_guard<std::mutex> lk(gnss_mtx_); fixes_.clear(); raw_.clear(); nav_.clear(); }
    { std::lock_guard<std::mutex> lk(rate_mtx_); imu_arr_.clear(); frame_arr_.clear(); gnss_arr_.clear(); }
}

}  // namespace odyx
