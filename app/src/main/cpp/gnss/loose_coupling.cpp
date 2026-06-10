#include "gnss/coupling.h"
#include "common/math_util.h"
#include "common/log.h"
#include <algorithm>

namespace odyx {

void LooseCoupling::addFix(const GnssFix& f) {
    if (!f.valid) return;
    if (!aligner_.hasOrigin() && f.hAcc_m <= cfg_.gnss_min_hacc_m)
        aligner_.setOrigin(f);
    fixes_.push_back(f);
    while (fixes_.size() > 128) fixes_.pop_front();
}

std::optional<GnssConstraint> LooseCoupling::constraintFor(TimeNs kf_t_ns) {
    if (!aligner_.hasOrigin() || fixes_.empty()) return std::nullopt;
    // nearest fix in time
    const GnssFix* best = nullptr; int64_t bestdt = INT64_MAX;
    for (const auto& f : fixes_) {
        int64_t dt = std::llabs(f.t_ns - kf_t_ns);
        if (dt < bestdt) { bestdt = dt; best = &f; }
    }
    if (!best || bestdt > s_to_ns(cfg_.gnss_match_window_s)) return std::nullopt;  // no close fix
    if (best->hAcc_m > cfg_.gnss_min_hacc_m * 2.5) return std::nullopt;

    Vec3 enu = aligner_.fixToEnu(*best);
    if (!aligner_.transform().valid) {
        // not yet aligned; report ENU directly so the aligner can build it but
        // do not yet constrain the graph in local frame.
        return std::nullopt;
    }
    Vec3 p_local = aligner_.transform().enuToLocal(enu);
    GnssConstraint c;
    c.t_ns = best->t_ns;
    c.p_world = p_local;
    c.std_m = std::max(0.5, best->hAcc_m);
    c.n_sats = best->n_sats;
    c.fix_type = 1;
    c.valid = true;
    return c;
}

}  // namespace odyx
