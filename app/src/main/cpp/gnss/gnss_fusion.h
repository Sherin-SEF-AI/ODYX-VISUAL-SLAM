// ===========================================================================
// ODYX :: GNSS fusion.
//
// EnuAligner   : online coarse-to-fine estimation of the 4-DoF transform
//                (yaw + 3D translation) between the gravity-aligned local VIO
//                world and the global ENU frame (origin = first good fix).
//                Reimplemented GVINS-style coarse-to-fine logic (not copied).
//
// GnssCoupling : interface with two implementations selected at runtime:
//                LooseCoupling  - per-fix GPS position factor (default)
//                TightCoupling  - raw pseudorange/Doppler via RTKLIB (advanced)
//
// Both produce GPS-style position constraints the backend adds to the graph;
// Tight additionally maintains receiver clock bias/drift and degrades to Loose
// when raw measurements / ephemeris are unavailable.
// ===========================================================================
#pragma once
#include "common/odyx_types.h"
#include "common/config.h"
#include <deque>
#include <memory>
#include <optional>

namespace odyx {

// 4-DoF similarity (no scale): p_enu = Rz(yaw) * p_local + t.
struct EnuTransform {
    double yaw = 0;
    Vec3 t = Vec3::Zero();
    bool valid = false;
    double resid_m = 0;

    Vec3 localToEnu(const Vec3& p) const;
    Vec3 enuToLocal(const Vec3& p) const;
    Mat3 R() const;     // Rz(yaw)
};

class EnuAligner {
public:
    explicit EnuAligner(const Config& cfg) : cfg_(cfg) {}

    // origin (first good fix); subsequent fixes converted to ENU internally.
    void setOrigin(const GnssFix& fix);
    bool hasOrigin() const { return has_origin_; }

    Vec3 fixToEnu(const GnssFix& fix) const;     // geodetic -> local ENU
    bool originLla(double& lat, double& lon, double& alt) const;

    // Add a {local VIO position, ENU position} correspondence; refine transform.
    void addCorrespondence(const Vec3& p_local, const Vec3& p_enu, double w);

    const EnuTransform& transform() const { return T_; }
    void hold() { holding_ = true; }            // dropout: freeze transform
    void resume() { holding_ = false; }
    void reset();

private:
    void coarse();   // closed-form yaw + translation from accumulated pairs
    void fine();     // Gauss-Newton refine on yaw + t

    Config cfg_;
    bool has_origin_ = false;
    double lat0_ = 0, lon0_ = 0, alt0_ = 0;
    Vec3 ecef0_ = Vec3::Zero();
    bool holding_ = false;

    struct Corr { Vec3 l, e; double w; };
    std::deque<Corr> corr_;
    EnuTransform T_;
};

// A position constraint to hand to the backend.
struct GnssConstraint {
    TimeNs t_ns = 0;
    Vec3 p_world = Vec3::Zero();   // antenna position in LOCAL world frame
    double std_m = 5.0;
    bool valid = false;
    int n_sats = 0;
    int fix_type = 0;
};

class GnssCoupling {
public:
    virtual ~GnssCoupling() = default;
    // Produce a constraint (in local world frame) for the given keyframe time,
    // using the current ENU transform. Returns nullopt if nothing usable.
    virtual std::optional<GnssConstraint> constraintFor(TimeNs kf_t_ns) = 0;
    virtual const char* name() const = 0;
};

}  // namespace odyx
