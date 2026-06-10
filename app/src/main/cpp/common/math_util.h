// ODYX :: small math helpers (SO3, geodesy ECEF<->ENU).
#pragma once
#include "common/odyx_types.h"
#include <cmath>

namespace odyx {

inline Mat3 skew(const Vec3& v) {
    Mat3 m;
    m <<     0, -v.z(),  v.y(),
         v.z(),      0, -v.x(),
        -v.y(),  v.x(),      0;
    return m;
}

// Exponential map so3 -> SO3.
inline Mat3 expSO3(const Vec3& w) {
    const double th = w.norm();
    if (th < 1e-9) return Mat3::Identity() + skew(w);
    const Vec3 a = w / th;
    const double s = std::sin(th), c = std::cos(th);
    return c * Mat3::Identity() + (1 - c) * a * a.transpose() + s * skew(a);
}

inline Vec3 logSO3(const Mat3& R) {
    const double tr = (R.trace() - 1.0) * 0.5;
    const double th = std::acos(std::max(-1.0, std::min(1.0, tr)));
    if (th < 1e-9) return Vec3(R(2,1)-R(1,2), R(0,2)-R(2,0), R(1,0)-R(0,1)) * 0.5;
    Vec3 v(R(2,1)-R(1,2), R(0,2)-R(2,0), R(1,0)-R(0,1));
    return v * (th / (2.0 * std::sin(th)));
}

// ---- WGS-84 geodesy --------------------------------------------------------
namespace wgs84 {
constexpr double a = 6378137.0;
constexpr double f = 1.0 / 298.257223563;
constexpr double e2 = f * (2.0 - f);
}

// lat/lon (deg), alt(m) -> ECEF (m).
inline Vec3 lla2ecef(double lat_deg, double lon_deg, double alt_m) {
    const double lat = lat_deg * M_PI / 180.0;
    const double lon = lon_deg * M_PI / 180.0;
    const double sl = std::sin(lat), cl = std::cos(lat);
    const double N = wgs84::a / std::sqrt(1.0 - wgs84::e2 * sl * sl);
    return Vec3((N + alt_m) * cl * std::cos(lon),
                (N + alt_m) * cl * std::sin(lon),
                (N * (1.0 - wgs84::e2) + alt_m) * sl);
}

// Rotation ECEF->ENU at reference lat/lon (deg).
inline Mat3 ecef2enuRot(double lat_deg, double lon_deg) {
    const double lat = lat_deg * M_PI / 180.0;
    const double lon = lon_deg * M_PI / 180.0;
    const double sl = std::sin(lat), cl = std::cos(lat);
    const double so = std::sin(lon), co = std::cos(lon);
    Mat3 R;
    R <<       -so,        co,    0,
         -sl * co,  -sl * so,   cl,
          cl * co,   cl * so,   sl;
    return R;
}

// ECEF point -> ENU relative to geodetic origin.
inline Vec3 ecef2enu(const Vec3& ecef, double lat0, double lon0, const Vec3& ecef0) {
    return ecef2enuRot(lat0, lon0) * (ecef - ecef0);
}

}  // namespace odyx
