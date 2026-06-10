// ===========================================================================
// Tightly-coupled GNSS via RTKLIB.
//
// Pipeline:
//   GnssNavigationMessage raw subframes --RTKLIB decode_frame--> eph_t (nav_)
//   GnssClock + GnssMeasurement --standard formula--> pseudorange obsd_t
//   RTKLIB pntpos() (SPP: satposs + WLS pseudorange) --> receiver ECEF + clk
//   ECEF -> ENU (origin) -> local world  => GnssConstraint
//
// GPS-focused (CONSTELLATION_GPS) for the SPP solution; other constellations
// are decoded for ephemeris where RTKLIB supports them. Degrades to LOOSE if
// fewer than 4 usable satellites or no ephemeris.
// ===========================================================================
#include "gnss/coupling.h"
#include "common/math_util.h"
#include "common/log.h"

extern "C" {
#include "rtklib.h"
}
#include <algorithm>
#include <cstring>

namespace odyx {

// Android GnssStatus constellation ids.
enum { CONSTELLATION_GPS = 1, CONSTELLATION_SBAS = 2, CONSTELLATION_GLONASS = 3,
       CONSTELLATION_QZSS = 4, CONSTELLATION_BEIDOU = 5, CONSTELLATION_GALILEO = 6 };

// Android GnssMeasurement state bits (subset).
enum { STATE_CODE_LOCK = 1, STATE_TOW_DECODED = 8, STATE_TOW_KNOWN = (1<<14) };

static constexpr double CLIGHT_ = 299792458.0;
static constexpr int64_t NS_PER_WEEK = 604800LL * 1000000000LL;

TightCoupling::TightCoupling(const Config& cfg, EnuAligner& aligner)
    : cfg_(cfg), aligner_(aligner), fallback_(cfg, aligner) {
    nav_t* nav = new nav_t();
    std::memset(nav, 0, sizeof(nav_t));
    nav->eph  = (eph_t*)malloc(sizeof(eph_t) * MAXSAT * 2);
    nav->n = nav->nmax = MAXSAT * 2;
    for (int i = 0; i < nav->n; ++i) std::memset(&nav->eph[i], 0, sizeof(eph_t));
    nav_ = nav;
    // in-progress ephemeris accumulator, one eph_t per GPS svid (1..32)
    eph_t* work = (eph_t*)malloc(sizeof(eph_t) * 33);
    for (int i = 0; i < 33; ++i) std::memset(&work[i], 0, sizeof(eph_t));
    ephwork_ = work;
}

TightCoupling::~TightCoupling() {
    if (nav_) { nav_t* nav = (nav_t*)nav_; if (nav->eph) free(nav->eph); delete nav; nav_ = nullptr; }
    if (ephwork_) { free(ephwork_); ephwork_ = nullptr; }
}

void TightCoupling::addFix(const GnssFix& f) { fallback_.addFix(f); }

void TightCoupling::addNav(const GnssNavMsg& msg) {
    if (msg.constellation != CONSTELLATION_GPS) return;   // GPS LNAV decode
    if (msg.data.size() < 40) return;                     // need 10 words
    nav_t* nav = (nav_t*)nav_;

    // Android delivers a 40-byte subframe (10 words, 30 bits each packed in
    // the low 30 bits of 4 bytes). RTKLIB decode_frame wants the 10 words.
    unsigned int words[10];
    for (int i = 0; i < 10; ++i) {
        const uint8_t* p = &msg.data[i*4];
        words[i] = ((unsigned)p[0]<<24)|((unsigned)p[1]<<16)|((unsigned)p[2]<<8)|p[3];
    }
    // decode_frame in RTKLIB takes a 30-byte buffer of subframe bits; we pack.
    unsigned char buff[30];
    for (int i = 0; i < 10; ++i) setbitu(buff, 30*i, 30, words[i] & 0x3FFFFFFF);
    int svid = msg.svid;
    if (svid < 1 || svid > 32) return;
    int sat = satno(SYS_GPS, svid);
    if (sat <= 0) return;

    // Accumulate subframes 1,2,3 INTO THE SAME per-SV eph (decode_frame fills the
    // fields relevant to each subframe). A complete, self-consistent GPS LNAV
    // ephemeris requires all three subframes with matching IODE (iodc low byte ==
    // iode2 == iode3). Only then commit it to the nav store for SPP.
    eph_t* work = &((eph_t*)ephwork_)[svid];
    work->sat = sat;
    alm_t alm[32]; double ion[8]={0}, utc[8]={0};
    std::memset(alm, 0, sizeof(alm));
    int id = decode_frame(buff, work, alm, ion, utc);
    if (id >= 1 && id <= 3) eph_frames_[svid] |= (unsigned char)(1u << id);  // bits 2,4,8

    const bool have123 = (eph_frames_[svid] & 0x0E) == 0x0E;
    if (have123 && work->toes != 0.0 && work->iode == (work->iodc & 0xFF)) {
        int idx = sat - 1;
        if (idx >= 0 && idx < nav->n) nav->eph[idx] = *work;   // commit complete eph
        eph_frames_[svid] = 0;                                 // ready for next set
    }
}

void TightCoupling::addRaw(const GnssClock& clk, const std::vector<GnssRawMeas>& meas) {
    raw_.push_back({clk, meas});
    while (raw_.size() > 64) raw_.pop_front();
}

// Build pseudoranges and run RTKLIB pntpos for a single-point fix.
bool TightCoupling::spp(const GnssClock& clk, const std::vector<GnssRawMeas>& meas,
                        Vec3& ecef_out, double& clk_bias_m, int& nsat, double& gdop) {
    nav_t* nav = (nav_t*)nav_;
    if (!clk.full_bias_valid) return false;

    // Receiver time in GPS time (ns): tRx = TimeNanos - (FullBias + Bias)
    const int64_t fullbias = clk.full_bias_ns;
    const double bias = clk.bias_ns;
    std::vector<obsd_t> obs;
    obs.reserve(meas.size());

    for (const auto& m : meas) {
        if (m.constellation != CONSTELLATION_GPS) continue;
        if (!(m.state & STATE_CODE_LOCK)) continue;
        if (!(m.state & (STATE_TOW_DECODED | STATE_TOW_KNOWN))) continue;
        int sat = satno(SYS_GPS, m.svid);
        if (sat <= 0) continue;

        // tTx (sv time, ns within week) and tRx (receiver, ns within week)
        int64_t tRxNs = clk.time_ns - (fullbias + (int64_t)llround(bias)) + m.time_offset_ns;
        int64_t weekNumberNanos = (tRxNs / NS_PER_WEEK) * NS_PER_WEEK;
        int64_t tRxSeconds_ns = tRxNs - weekNumberNanos;
        int64_t tTxSeconds_ns = m.received_sv_time_ns;
        double pr = (double)(tRxSeconds_ns - tTxSeconds_ns) * 1e-9 * CLIGHT_;
        // wrap correction near week rollover
        if (pr > 1e8)  pr -= (double)NS_PER_WEEK * 1e-9 * CLIGHT_;
        if (pr < -1e8) pr += (double)NS_PER_WEEK * 1e-9 * CLIGHT_;
        if (pr < 1e6 || pr > 5e7) continue;   // sane range (~ LEO..GEO)

        obsd_t o; std::memset(&o, 0, sizeof(o));
        // GPS week directly from the receiver GPS time (tRxNs is ns since the GPS
        // epoch); deriving it from FullBias sign was wrong near rollovers.
        int week = (int)(tRxNs / NS_PER_WEEK);
        double sow = (double)tRxSeconds_ns * 1e-9;
        o.time = gpst2time(week, sow);
        o.sat = sat;
        o.P[0] = pr;
        o.D[0] = (float)(m.pseudorange_rate_mps / (CLIGHT_ / (m.carrier_frequency_hz > 0 ? m.carrier_frequency_hz : FREQ1)));
        o.SNR[0] = (uint16_t)(m.cn0_dbhz / SNR_UNIT + 0.5);
        o.code[0] = CODE_L1C;
        obs.push_back(o);
    }

    nsat = (int)obs.size();
    if (nsat < 4) return false;

    sol_t sol; std::memset(&sol, 0, sizeof(sol));
    ssat_t ssat[MAXSAT]; std::memset(ssat, 0, sizeof(ssat));
    char msg[128] = {0};
    prcopt_t opt = prcopt_default;
    opt.mode = PMODE_SINGLE;
    opt.navsys = SYS_GPS;
    opt.ionoopt = IONOOPT_BRDC;
    opt.tropopt = TROPOPT_SAAS;
    opt.sateph = EPHOPT_BRDC;

    int ok = pntpos(obs.data(), nsat, nav, &opt, &sol, NULL, ssat, msg);
    if (!ok || sol.stat == SOLQ_NONE) {
        ODYXW("gnss", "pntpos failed: %s", msg);
        return false;
    }
    ecef_out = Vec3(sol.rr[0], sol.rr[1], sol.rr[2]);
    clk_bias_m = sol.dtr[0] * CLIGHT_;
    gdop = sol.qr[0];   // rough
    return true;
}

std::optional<GnssConstraint> TightCoupling::constraintFor(TimeNs kf_t_ns) {
    // (The ENU origin is bootstrapped from the SPP solution below when not yet set.)
    // find raw batch nearest the keyframe time
    const RawBatch* best = nullptr; int64_t bdt = INT64_MAX;
    for (auto& rb : raw_) {
        int64_t dt = std::llabs(rb.clk.t_ns - kf_t_ns);
        if (dt < bdt) { bdt = dt; best = &rb; }
    }
    if (!best || bdt > s_to_ns(cfg_.gnss_match_window_s)) { degraded_ = true; return fallback_.constraintFor(kf_t_ns); }

    Vec3 ecef; double clkb; int nsat; double gdop;
    if (!spp(best->clk, best->meas, ecef, clkb, nsat, gdop)) {
        degraded_ = true;
        return fallback_.constraintFor(kf_t_ns);
    }
    degraded_ = false; last_nsats_ = nsat;

    // Convert ECEF -> geodetic to (re)establish the ENU origin if needed.
    double pos[3]; ecef2pos(ecef.data(), pos);
    if (!aligner_.hasOrigin()) {
        GnssFix f; f.valid = true;
        f.lat_deg = pos[0]*R2D; f.lon_deg = pos[1]*R2D; f.alt_m = pos[2]; f.has_alt = true;
        f.hAcc_m = std::max(2.0, gdop); f.n_sats = nsat;
        aligner_.setOrigin(f);
    }
    double lat0, lon0, alt0; aligner_.originLla(lat0, lon0, alt0);
    Vec3 ecef0 = lla2ecef(lat0, lon0, alt0);
    Vec3 enu = ecef2enu(ecef, lat0, lon0, ecef0);

    if (!aligner_.transform().valid) return std::nullopt;
    Vec3 p_local = aligner_.transform().enuToLocal(enu);

    GnssConstraint c;
    c.t_ns = best->clk.t_ns;
    c.p_world = p_local;
    c.std_m = std::max(1.0, gdop * 1.5);
    c.n_sats = nsat;
    c.fix_type = 2;
    c.valid = true;
    return c;
}

}  // namespace odyx
