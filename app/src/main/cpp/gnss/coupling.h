// ODYX :: concrete GNSS coupling implementations (loose + tight).
#pragma once
#include "gnss/gnss_fusion.h"
#include <unordered_map>

namespace odyx {

// Loosely-coupled: buffers fixes, converts to ENU via the aligner, returns the
// fix nearest a keyframe time as a position constraint in the local world frame.
class LooseCoupling : public GnssCoupling {
public:
    LooseCoupling(const Config& cfg, EnuAligner& aligner) : cfg_(cfg), aligner_(aligner) {}
    void addFix(const GnssFix& f);
    std::optional<GnssConstraint> constraintFor(TimeNs kf_t_ns) override;
    const char* name() const override { return "LOOSE"; }
private:
    Config cfg_;
    EnuAligner& aligner_;
    std::deque<GnssFix> fixes_;
};

// Tightly-coupled: decodes raw GnssMeasurement + broadcast ephemeris (via
// RTKLIB) into per-satellite positions/clocks, runs a single-point-position
// least squares (pseudorange + Doppler) with receiver clock bias/drift, and
// emits the ECEF->ENU->local position constraint. Degrades to a wrapped
// LooseCoupling when raw data / ephemeris are missing.
class TightCoupling : public GnssCoupling {
public:
    TightCoupling(const Config& cfg, EnuAligner& aligner);
    ~TightCoupling() override;
    void addRaw(const GnssClock& clk, const std::vector<GnssRawMeas>& meas);
    void addNav(const GnssNavMsg& nav);
    void addFix(const GnssFix& f);   // fallback path
    std::optional<GnssConstraint> constraintFor(TimeNs kf_t_ns) override;
    const char* name() const override { return degraded_ ? "TIGHT(->LOOSE)" : "TIGHT"; }

    bool degraded() const { return degraded_; }
    int lastNumSats() const { return last_nsats_; }

private:
    // returns ECEF receiver position + clock-bias (m); false if SPP failed.
    bool spp(const GnssClock& clk, const std::vector<GnssRawMeas>& meas,
             Vec3& ecef_out, double& clk_bias_m, int& nsat, double& gdop);

    Config cfg_;
    EnuAligner& aligner_;
    LooseCoupling fallback_;
    bool degraded_ = false;
    int last_nsats_ = 0;

    struct RawBatch { GnssClock clk; std::vector<GnssRawMeas> meas; };
    std::deque<RawBatch> raw_;
    // opaque RTKLIB nav store (eph table). Defined in tight_coupling.cpp.
    void* nav_ = nullptr;
    // Per-GPS-SVID in-progress ephemeris accumulator (eph_t[33]) + subframe mask
    // so subframes 1-3 of one data set assemble into a complete, consistent eph
    // before being committed to nav_. Allocated in the ctor.
    void* ephwork_ = nullptr;
    unsigned char eph_frames_[33] = {0};
};

}  // namespace odyx
