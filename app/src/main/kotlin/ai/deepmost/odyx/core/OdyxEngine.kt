package ai.deepmost.odyx.core

import ai.deepmost.odyx.config.GnssCoupling
import ai.deepmost.odyx.config.OdyxConfig
import ai.deepmost.odyx.jni.MapSnapshotK
import ai.deepmost.odyx.jni.NativeBridge
import ai.deepmost.odyx.jni.PoseSnapshot
import ai.deepmost.odyx.jni.TelemetrySnapshot
import timber.log.Timber
import java.util.concurrent.atomic.AtomicBoolean

/**
 * High-level owner of one native estimator handle. Thread-safe to push from the
 * camera/IMU/GNSS threads and pull from the UI thread. Holds no estimation
 * logic — it only marshals across the JNI boundary.
 */
class OdyxEngine : SensorSink {
    private var handle: Long = 0
    private val started = AtomicBoolean(false)
    private val poseBuf = DoubleArray(PoseSnapshot.SIZE)
    private val teleBuf = DoubleArray(TelemetrySnapshot.SIZE)

    fun create() {
        if (handle == 0L) {
            handle = NativeBridge.nativeCreate()
            Timber.tag("OdyxEngine").i("created native handle=%d", handle)
        }
    }

    fun loadConfig(cfg: OdyxConfig) {
        if (handle == 0L) return
        NativeBridge.nativeLoadConfig(handle, cfg.toParams())
    }

    fun loadVocabulary(path: String): Boolean =
        handle != 0L && NativeBridge.nativeLoadVocabulary(handle, path)

    fun start() {
        if (handle != 0L && started.compareAndSet(false, true)) NativeBridge.nativeStart(handle)
    }
    fun stop() {
        if (handle != 0L && started.compareAndSet(true, false)) NativeBridge.nativeStop(handle)
    }
    fun reset() { if (handle != 0L) NativeBridge.nativeReset(handle) }
    fun destroy() {
        stop()
        if (handle != 0L) { NativeBridge.nativeDestroy(handle); handle = 0 }
    }

    // ---- producers -------------------------------------------------------
    override fun pushFrame(
        y: ByteArray, w: Int, h: Int, rowStride: Int, tNs: Long,
        rsSkewNs: Long, exposureS: Double, fx: Double, fy: Double, cx: Double, cy: Double
    ) {
        if (handle != 0L) NativeBridge.nativePushFrame(handle, y, w, h, rowStride, tNs, rsSkewNs, exposureS, fx, fy, cx, cy)
    }

    override fun pushImu(tNs: Long, ax: Float, ay: Float, az: Float, gx: Float, gy: Float, gz: Float) {
        if (handle != 0L) NativeBridge.nativePushImu(handle, tNs, ax, ay, az, gx, gy, gz)
    }

    override fun pushGnssFix(tNs: Long, lat: Double, lon: Double, alt: Double, hAcc: Double, vAcc: Double, nSats: Int, hasAlt: Boolean) {
        if (handle != 0L) NativeBridge.nativePushGnssFix(handle, tNs, lat, lon, alt, hAcc, vAcc, nSats, hasAlt)
    }

    override fun pushGnssRaw(
        clkTNs: Long, clkTimeNs: Long, fullBiasNs: Long, biasNs: Double, driftNsps: Double,
        leapSec: Int, fullBiasValid: Boolean,
        svid: IntArray, constellation: IntArray, cn0: DoubleArray, state: IntArray,
        rxSvTimeNs: LongArray, rxSvTimeUncNs: LongArray, prRate: DoubleArray,
        prRateUnc: DoubleArray, carrierFreq: DoubleArray, timeOffsetNs: LongArray
    ) {
        if (handle != 0L) NativeBridge.nativePushGnssRaw(
            handle, clkTNs, clkTimeNs, fullBiasNs, biasNs, driftNsps, leapSec, fullBiasValid,
            svid, constellation, cn0, state, rxSvTimeNs, rxSvTimeUncNs, prRate, prRateUnc, carrierFreq, timeOffsetNs
        )
    }

    override fun pushNav(constellation: Int, svid: Int, type: Int, subId: Int, msgId: Int, data: ByteArray) {
        if (handle != 0L) NativeBridge.nativePushNav(handle, constellation, svid, type, subId, msgId, data)
    }

    // ---- consumers -------------------------------------------------------
    @Synchronized
    fun pose(): PoseSnapshot? {
        if (handle == 0L) return null
        NativeBridge.nativeGetPose(handle, poseBuf)
        return PoseSnapshot.from(poseBuf)
    }

    @Synchronized
    fun telemetry(): TelemetrySnapshot? {
        if (handle == 0L) return null
        NativeBridge.nativeGetTelemetry(handle, teleBuf)
        return TelemetrySnapshot.from(teleBuf)
    }

    fun map(): MapSnapshotK? =
        if (handle == 0L) null else MapSnapshotK.from(NativeBridge.nativeGetMapSnapshot(handle))

    /** Current-frame tracked features as [u,v,age,isNew] rows (for the overlay). */
    fun features(): FloatArray? =
        if (handle == 0L) null else NativeBridge.nativeGetFeatures(handle)

    // ---- toggles ---------------------------------------------------------
    // ---- checkerboard calibration ----
    fun setCalibActive(on: Boolean, cols: Int = 9, rows: Int = 6, squareM: Double = 0.025) {
        if (handle != 0L) NativeBridge.nativeSetCalibActive(handle, on, cols, rows, squareM)
    }
    fun captureCalibView(): Int = if (handle == 0L) 0 else NativeBridge.nativeCaptureCalibView(handle)
    fun calibViewCount(): Int = if (handle == 0L) 0 else NativeBridge.nativeCalibViewCount(handle)
    fun resetCalibration() { if (handle != 0L) NativeBridge.nativeResetCalibration(handle) }
    fun calibCorners(): FloatArray? = if (handle == 0L) null else NativeBridge.nativeCalibCorners(handle)
    fun runCalibration(): DoubleArray {
        val out = DoubleArray(14)
        if (handle != 0L) NativeBridge.nativeRunCalibration(handle, out)
        return out
    }

    fun setGnssMode(m: GnssCoupling) { if (handle != 0L) NativeBridge.nativeSetGnssMode(handle, m.code) }
    fun setOnlineCalib(td: Boolean, extr: Boolean) { if (handle != 0L) NativeBridge.nativeSetOnlineCalib(handle, td, extr) }
    fun setRollingShutter(on: Boolean) { if (handle != 0L) NativeBridge.nativeSetRollingShutter(handle, on) }
    fun setLoopClosure(on: Boolean) { if (handle != 0L) NativeBridge.nativeSetLoopClosure(handle, on) }
    fun setThermal(level: Int) { if (handle != 0L) NativeBridge.nativeSetThermal(handle, level) }

    companion object {
        /** M0 smoke. Returns the optimized translation (~1.0) or a negative error code. */
        fun gtsamSmoke(): Double = NativeBridge.nativeGtsamSmoke()
    }
}
