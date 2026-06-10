// ===========================================================================
// ODYX :: JNI bridge (thin). Forwards timestamped sensor data DOWN into the
// native Estimator and pulls pose / telemetry / map snapshots UP. All inputs
// are validated; bad data is dropped, never trusted. No estimation logic here.
// ===========================================================================
#include <jni.h>
#include <android/log.h>
#include <memory>
#include <vector>
#include <cmath>

#include "core/estimator.h"
#include "common/log.h"
#include <DBoW3/DBoW3.h>

using namespace odyx;

namespace {
Estimator* E(jlong h) { return reinterpret_cast<Estimator*>(h); }

bool finite3(double a, double b, double c) {
    return std::isfinite(a) && std::isfinite(b) && std::isfinite(c);
}
}  // namespace

extern "C" {

// ------------------------------------------------------------------ M0 smoke
// Optimizes a tiny 2-pose BetweenFactor graph in GTSAM and returns the
// recovered x-translation of pose 1 (~1.0). Proves the GTSAM cross-compile
// links and runs before any SLAM code is exercised.
JNIEXPORT jdouble JNICALL
Java_ai_deepmost_odyx_jni_NativeBridge_nativeGtsamSmoke(JNIEnv*, jobject);

// ------------------------------------------------------------------ lifecycle
JNIEXPORT jlong JNICALL
Java_ai_deepmost_odyx_jni_NativeBridge_nativeCreate(JNIEnv*, jobject) {
    return reinterpret_cast<jlong>(new Estimator());
}
JNIEXPORT void JNICALL
Java_ai_deepmost_odyx_jni_NativeBridge_nativeDestroy(JNIEnv*, jobject, jlong h) {
    delete E(h);
}
JNIEXPORT void JNICALL
Java_ai_deepmost_odyx_jni_NativeBridge_nativeStart(JNIEnv*, jobject, jlong h) {
    if (h) E(h)->start();
}
JNIEXPORT void JNICALL
Java_ai_deepmost_odyx_jni_NativeBridge_nativeStop(JNIEnv*, jobject, jlong h) {
    if (h) E(h)->stop();
}
JNIEXPORT void JNICALL
Java_ai_deepmost_odyx_jni_NativeBridge_nativeReset(JNIEnv*, jobject, jlong h) {
    if (h) E(h)->reset();
}

// loadConfig: fixed-layout double vector. See Kotlin ConfigMarshal for layout.
JNIEXPORT void JNICALL
Java_ai_deepmost_odyx_jni_NativeBridge_nativeLoadConfig(JNIEnv* env, jobject, jlong h,
                                                        jdoubleArray arr) {
    if (!h || !arr) return;
    jsize n = env->GetArrayLength(arr);
    std::vector<double> p(n);
    env->GetDoubleArrayRegion(arr, 0, n, p.data());
    if (n < 47) return;   // see OdyxConfig.toParams() for the 47-element layout
    Config c;
    int i = 0;
    c.cam.width = (int)p[i++]; c.cam.height = (int)p[i++];
    c.cam.fx = p[i++]; c.cam.fy = p[i++]; c.cam.cx = p[i++]; c.cam.cy = p[i++];
    for (int k = 0; k < 5; ++k) c.cam.dist[k] = p[i++];
    c.cam.valid = p[i++] != 0;
    c.imu.acc_n = p[i++]; c.imu.gyr_n = p[i++]; c.imu.acc_w = p[i++]; c.imu.gyr_w = p[i++]; c.imu.g = p[i++];
    c.extr.q_bc = Quat(p[i], p[i+1], p[i+2], p[i+3]).normalized(); i += 4;
    c.extr.t_bc = Vec3(p[i], p[i+1], p[i+2]); i += 3;
    c.extr.td = p[i++]; c.extr.valid = p[i++] != 0;
    c.gnss_lever_arm = Vec3(p[i], p[i+1], p[i+2]); i += 3;
    c.target_features = (int)p[i++]; c.grid_cols = (int)p[i++]; c.grid_rows = (int)p[i++];
    c.kf_parallax_px = p[i++]; c.kf_tracked_ratio = p[i++];
    c.window_keyframes = (int)p[i++]; c.max_landmarks = (int)p[i++]; c.huber_px = p[i++];
    c.online_td = p[i++] != 0; c.online_extr = p[i++] != 0; c.rolling_shutter = p[i++] != 0;
    c.loop_enabled = p[i++] != 0; c.dbow_score_thresh = p[i++]; c.loop_min_inliers = (int)p[i++];
    c.gnss_mode = (GnssMode)(int)p[i++]; c.max_keyframes_db = (int)p[i++];
    c.ransac_thresh_px = p[i++]; c.gnss_match_window_s = p[i++];
    E(h)->loadConfig(c);
}

JNIEXPORT jboolean JNICALL
Java_ai_deepmost_odyx_jni_NativeBridge_nativeLoadVocabulary(JNIEnv* env, jobject, jlong h,
                                                            jstring path) {
    if (!h || !path) return JNI_FALSE;
    const char* cp = env->GetStringUTFChars(path, nullptr);
    bool ok = false;
    try {
        auto voc = std::make_shared<DBoW3::Vocabulary>();
        voc->load(cp);
        ok = voc->size() > 0;
        if (ok) E(h)->setVocabulary(voc);
    } catch (const std::exception& e) {
        ODYXE("jni", "vocab load failed: %s", e.what());
    }
    env->ReleaseStringUTFChars(path, cp);
    return ok ? JNI_TRUE : JNI_FALSE;
}

// ------------------------------------------------------------------ producers
JNIEXPORT void JNICALL
Java_ai_deepmost_odyx_jni_NativeBridge_nativePushFrame(JNIEnv* env, jobject, jlong h,
        jbyteArray y, jint w, jint hgt, jint rowStride, jlong tNs, jlong rsSkewNs,
        jdouble exposureS, jdouble fx, jdouble fy, jdouble cx, jdouble cy) {
    if (!h || !y || w <= 0 || hgt <= 0 || tNs <= 0) return;
    jsize len = env->GetArrayLength(y);
    if (len < (jsize)rowStride * hgt) return;   // validate buffer size
    auto f = std::make_shared<Frame>();
    f->t_ns = tNs; f->width = w; f->height = hgt;
    f->rolling_shutter_skew_ns = rsSkewNs; f->exposure_s = exposureS;
    f->gray.resize((size_t)w * hgt);
    jbyte* src = env->GetByteArrayElements(y, nullptr);
    for (int r = 0; r < hgt; ++r)
        memcpy(&f->gray[(size_t)r*w], (uint8_t*)src + (size_t)r*rowStride, w);
    env->ReleaseByteArrayElements(y, src, JNI_ABORT);
    E(h)->pushFrame(std::move(f));
}

JNIEXPORT void JNICALL
Java_ai_deepmost_odyx_jni_NativeBridge_nativePushImu(JNIEnv*, jobject, jlong h,
        jlong tNs, jfloat ax, jfloat ay, jfloat az, jfloat gx, jfloat gy, jfloat gz) {
    if (!h || tNs <= 0) return;
    if (!finite3(ax,ay,az) || !finite3(gx,gy,gz)) return;
    ImuSample s; s.t_ns = tNs;
    s.acc = Vec3(ax,ay,az); s.gyr = Vec3(gx,gy,gz);
    E(h)->pushImu(s);
}

JNIEXPORT void JNICALL
Java_ai_deepmost_odyx_jni_NativeBridge_nativePushGnssFix(JNIEnv*, jobject, jlong h,
        jlong tNs, jdouble lat, jdouble lon, jdouble alt, jdouble hAcc, jdouble vAcc,
        jint nSats, jboolean hasAlt) {
    if (!h || tNs <= 0) return;
    if (std::abs(lat) > 90 || std::abs(lon) > 180) return;
    GnssFix f; f.t_ns = tNs; f.lat_deg = lat; f.lon_deg = lon; f.alt_m = alt;
    f.hAcc_m = hAcc; f.vAcc_m = vAcc; f.n_sats = nSats; f.has_alt = hasAlt; f.valid = true;
    E(h)->pushGnssFix(f);
}

JNIEXPORT void JNICALL
Java_ai_deepmost_odyx_jni_NativeBridge_nativePushGnssRaw(JNIEnv* env, jobject, jlong h,
        jlong clkTNs, jlong clkTimeNs, jlong fullBiasNs, jdouble biasNs, jdouble driftNsps,
        jint leapSec, jboolean fullBiasValid,
        jintArray svid, jintArray constellation, jdoubleArray cn0, jintArray state,
        jlongArray rxSvTimeNs, jlongArray rxSvTimeUncNs, jdoubleArray prRate,
        jdoubleArray prRateUnc, jdoubleArray carrierFreq, jlongArray timeOffsetNs) {
    if (!h) return;
    GnssClock clk; clk.t_ns = clkTNs; clk.time_ns = clkTimeNs; clk.full_bias_ns = fullBiasNs;
    clk.bias_ns = biasNs; clk.drift_nsps = driftNsps; clk.leap_second = leapSec;
    clk.full_bias_valid = fullBiasValid;
    jsize n = env->GetArrayLength(svid);
    std::vector<jint> sv(n), con(n), st(n);
    std::vector<jdouble> c0(n), pr(n), pru(n), cf(n);
    std::vector<jlong> rx(n), rxu(n), to(n);
    env->GetIntArrayRegion(svid,0,n,sv.data());
    env->GetIntArrayRegion(constellation,0,n,con.data());
    env->GetIntArrayRegion(state,0,n,st.data());
    env->GetDoubleArrayRegion(cn0,0,n,c0.data());
    env->GetDoubleArrayRegion(prRate,0,n,pr.data());
    env->GetDoubleArrayRegion(prRateUnc,0,n,pru.data());
    env->GetDoubleArrayRegion(carrierFreq,0,n,cf.data());
    env->GetLongArrayRegion(rxSvTimeNs,0,n,rx.data());
    env->GetLongArrayRegion(rxSvTimeUncNs,0,n,rxu.data());
    env->GetLongArrayRegion(timeOffsetNs,0,n,to.data());
    std::vector<GnssRawMeas> meas(n);
    for (int i = 0; i < n; ++i) {
        meas[i].t_ns = clkTNs; meas[i].svid = sv[i]; meas[i].constellation = con[i];
        meas[i].cn0_dbhz = c0[i]; meas[i].state = st[i];
        meas[i].received_sv_time_ns = rx[i]; meas[i].received_sv_time_uncertainty_ns = rxu[i];
        meas[i].pseudorange_rate_mps = pr[i]; meas[i].pseudorange_rate_uncertainty_mps = pru[i];
        meas[i].carrier_frequency_hz = cf[i]; meas[i].time_offset_ns = to[i];
        meas[i].full_biases_valid = fullBiasValid;
    }
    E(h)->pushGnssRaw(clk, meas);
}

JNIEXPORT void JNICALL
Java_ai_deepmost_odyx_jni_NativeBridge_nativePushNav(JNIEnv* env, jobject, jlong h,
        jint constellation, jint svid, jint type, jint subId, jint msgId, jbyteArray data) {
    if (!h || !data) return;
    GnssNavMsg m; m.constellation = constellation; m.svid = svid; m.type = type;
    m.submessage_id = subId; m.message_id = msgId;
    jsize n = env->GetArrayLength(data);
    m.data.resize(n);
    env->GetByteArrayRegion(data, 0, n, (jbyte*)m.data.data());
    E(h)->pushNavMsg(m);
}

// ------------------------------------------------------------------ consumers
// out layout (size 26): valid, t_ns(as double),
//   qw,qx,qy,qz, px,py,pz, vx,vy,vz, bax,bay,baz, bgx,bgy,bgz,
//   cov_pos_trace, cov_rot_trace, td, enu_px,enu_py,enu_pz, enu_valid
JNIEXPORT void JNICALL
Java_ai_deepmost_odyx_jni_NativeBridge_nativeGetPose(JNIEnv* env, jobject, jlong h,
                                                     jdoubleArray out) {
    if (!h || !out) return;
    NavStateOut p = E(h)->pose();
    MapSnapshot snap = E(h)->mapSnapshot();
    double o[26] = {0};
    o[0] = p.valid ? 1 : 0; o[1] = (double)p.t_ns;
    o[2]=p.q_wb.w(); o[3]=p.q_wb.x(); o[4]=p.q_wb.y(); o[5]=p.q_wb.z();
    o[6]=p.p_wb.x(); o[7]=p.p_wb.y(); o[8]=p.p_wb.z();
    o[9]=p.v_w.x(); o[10]=p.v_w.y(); o[11]=p.v_w.z();
    o[12]=p.ba.x(); o[13]=p.ba.y(); o[14]=p.ba.z();
    o[15]=p.bg.x(); o[16]=p.bg.y(); o[17]=p.bg.z();
    o[18]=p.pose_cov.block<3,3>(3,3).trace(); o[19]=p.pose_cov.block<3,3>(0,0).trace();
    o[20]=E(h)->telemetry().td;
    if (snap.enu_valid && !snap.trajectory_enu.empty()) {
        Vec3 e = snap.trajectory_enu.back();
        o[21]=e.x(); o[22]=e.y(); o[23]=e.z(); o[24]=1;
    }
    jsize n = env->GetArrayLength(out);
    env->SetDoubleArrayRegion(out, 0, n < 26 ? n : 26, o);
}

// telemetry out layout (size 40), mirrors Telemetry struct field order.
JNIEXPORT void JNICALL
Java_ai_deepmost_odyx_jni_NativeBridge_nativeGetTelemetry(JNIEnv* env, jobject, jlong h,
                                                          jdoubleArray out) {
    if (!h || !out) return;
    Telemetry t = E(h)->telemetry();
    double o[40] = {0}; int i = 0;
    o[i++]=t.tracking_state; o[i++]=t.init_state; o[i++]=t.gnss_mode;
    o[i++]=t.n_tracked; o[i++]=t.n_keyframes; o[i++]=t.window_size; o[i++]=t.n_landmarks;
    o[i++]=t.frontend_ms; o[i++]=t.optimize_ms;
    o[i++]=t.imu_rate; o[i++]=t.frame_rate; o[i++]=t.gnss_rate;
    o[i++]=t.td;
    o[i++]=t.bg.x(); o[i++]=t.bg.y(); o[i++]=t.bg.z();
    o[i++]=t.ba.x(); o[i++]=t.ba.y(); o[i++]=t.ba.z();
    o[i++]=t.vel.x(); o[i++]=t.vel.y(); o[i++]=t.vel.z();
    o[i++]=t.gravity_align_resid;
    o[i++]=t.gnss_n_sats; o[i++]=t.gnss_fix_type; o[i++]=t.gnss_acc_m;
    o[i++]=t.enu_aligned?1:0; o[i++]=t.enu_resid_m;
    o[i++]=t.loop_count; o[i++]=t.drift_proxy_m; o[i++]=t.thermal_status;
    o[i++]=t.frame_stride; o[i++]=t.target_features; o[i++]=t.calib_default?1:0;
    jsize n = env->GetArrayLength(out);
    env->SetDoubleArrayRegion(out, 0, n < 40 ? n : 40, o);
}

// Map snapshot packed as floats:
// [0]=nTrajLocal [1]=nTrajEnu [2]=nLm [3]=nLoop [4]=nGnss [5]=enuValid
// then trajLocal(3*nTrajLocal), trajEnu(3*nTrajEnu), lm(4*nLm: x,y,z,covtr),
// loop(2*nLoop: a,b as float), gnss(3*nGnss)
JNIEXPORT jfloatArray JNICALL
Java_ai_deepmost_odyx_jni_NativeBridge_nativeGetMapSnapshot(JNIEnv* env, jobject, jlong h) {
    if (!h) return nullptr;
    MapSnapshot s = E(h)->mapSnapshot();
    std::vector<float> buf;
    buf.resize(6);
    buf[0]=(float)s.trajectory_local.size(); buf[1]=(float)s.trajectory_enu.size();
    buf[2]=(float)s.landmarks.size(); buf[3]=(float)s.loop_edges.size();
    buf[4]=(float)s.gnss_markers_enu.size(); buf[5]=s.enu_valid?1.f:0.f;
    for (auto& st : s.trajectory_local) { buf.push_back(st.p_wb.x()); buf.push_back(st.p_wb.y()); buf.push_back(st.p_wb.z()); }
    for (auto& e : s.trajectory_enu) { buf.push_back(e.x()); buf.push_back(e.y()); buf.push_back(e.z()); }
    for (auto& l : s.landmarks) { buf.push_back(l.p_w.x()); buf.push_back(l.p_w.y()); buf.push_back(l.p_w.z()); buf.push_back(l.cov_trace); }
    for (auto& e : s.loop_edges) { buf.push_back((float)e.kf_a); buf.push_back((float)e.kf_b); }
    for (auto& g : s.gnss_markers_enu) { buf.push_back(g.x()); buf.push_back(g.y()); buf.push_back(g.z()); }
    jfloatArray out = env->NewFloatArray((jsize)buf.size());
    env->SetFloatArrayRegion(out, 0, (jsize)buf.size(), buf.data());
    return out;
}

// Current-frame features for the SLAM overlay: [count, (u,v,age,isNew)*count].
JNIEXPORT jfloatArray JNICALL
Java_ai_deepmost_odyx_jni_NativeBridge_nativeGetFeatures(JNIEnv* env, jobject, jlong h) {
    if (!h) return nullptr;
    std::vector<float> f = E(h)->featuresFlat();
    jfloatArray out = env->NewFloatArray((jsize)f.size());
    env->SetFloatArrayRegion(out, 0, (jsize)f.size(), f.data());
    return out;
}

// ------------------------------------------------------------- calibration
JNIEXPORT void JNICALL
Java_ai_deepmost_odyx_jni_NativeBridge_nativeSetCalibActive(JNIEnv*, jobject, jlong h,
        jboolean on, jint cols, jint rows, jdouble square) {
    if (!h) return;
    E(h)->setCalibBoard(cols, rows, square);
    E(h)->setCalibActive(on);
}
JNIEXPORT jint JNICALL
Java_ai_deepmost_odyx_jni_NativeBridge_nativeCaptureCalibView(JNIEnv*, jobject, jlong h) {
    return h ? E(h)->captureCalibView() : 0;
}
JNIEXPORT jint JNICALL
Java_ai_deepmost_odyx_jni_NativeBridge_nativeCalibViewCount(JNIEnv*, jobject, jlong h) {
    return h ? E(h)->calibViewCount() : 0;
}
JNIEXPORT void JNICALL
Java_ai_deepmost_odyx_jni_NativeBridge_nativeResetCalibration(JNIEnv*, jobject, jlong h) {
    if (h) E(h)->resetCalibration();
}
JNIEXPORT jfloatArray JNICALL
Java_ai_deepmost_odyx_jni_NativeBridge_nativeCalibCorners(JNIEnv* env, jobject, jlong h) {
    if (!h) return nullptr;
    std::vector<float> c = E(h)->calibCornersFlat();
    jfloatArray out = env->NewFloatArray((jsize)c.size());
    env->SetFloatArrayRegion(out, 0, (jsize)c.size(), c.data());
    return out;
}
// out: [ok, fx,fy,cx,cy, k1,k2,p1,p2,k3, rms, nviews, w, h] (14)
JNIEXPORT void JNICALL
Java_ai_deepmost_odyx_jni_NativeBridge_nativeRunCalibration(JNIEnv* env, jobject, jlong h, jdoubleArray out) {
    if (!h || !out) return;
    CalibResult r = E(h)->runCalibration();
    double o[14] = {0};
    o[0]=r.ok?1:0; o[1]=r.fx; o[2]=r.fy; o[3]=r.cx; o[4]=r.cy;
    for (int i=0;i<5;++i) o[5+i]=r.dist[i];
    o[10]=r.rms; o[11]=r.n_views; o[12]=r.width; o[13]=r.height;
    jsize n = env->GetArrayLength(out);
    env->SetDoubleArrayRegion(out, 0, n<14?n:14, o);
}

// ------------------------------------------------------------------ toggles
JNIEXPORT void JNICALL
Java_ai_deepmost_odyx_jni_NativeBridge_nativeSetGnssMode(JNIEnv*, jobject, jlong h, jint m) {
    if (h) E(h)->setGnssMode((GnssMode)m);
}
JNIEXPORT void JNICALL
Java_ai_deepmost_odyx_jni_NativeBridge_nativeSetOnlineCalib(JNIEnv*, jobject, jlong h,
                                                            jboolean td, jboolean extr) {
    if (h) E(h)->setOnlineCalib(td, extr);
}
JNIEXPORT void JNICALL
Java_ai_deepmost_odyx_jni_NativeBridge_nativeSetRollingShutter(JNIEnv*, jobject, jlong h, jboolean on) {
    if (h) E(h)->setRollingShutter(on);
}
JNIEXPORT void JNICALL
Java_ai_deepmost_odyx_jni_NativeBridge_nativeSetLoopClosure(JNIEnv*, jobject, jlong h, jboolean on) {
    if (h) E(h)->setLoopClosure(on);
}
JNIEXPORT void JNICALL
Java_ai_deepmost_odyx_jni_NativeBridge_nativeSetThermal(JNIEnv*, jobject, jlong h, jint level) {
    if (h) E(h)->setThermalStatus(level);
}

}  // extern "C"
