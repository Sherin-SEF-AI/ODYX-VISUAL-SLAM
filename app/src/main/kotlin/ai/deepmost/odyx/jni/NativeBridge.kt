package ai.deepmost.odyx.jni

/**
 * Thin JNI surface to the native ODYX estimator (libodyx.so). One [handle] owns
 * one native [Estimator]. All methods are non-blocking from the caller's point
 * of view except the pull methods, which copy out a snapshot.
 *
 * Layouts of the marshalled arrays are documented next to the native bridge
 * (odyx_jni.cpp) and mirrored by [ConfigMarshal] / [PoseSnapshot] / [TelemetrySnapshot].
 */
object NativeBridge {
    init { System.loadLibrary("odyx") }

    // M0 smoke: runs a tiny GTSAM optimization, returns ~1.0 on success.
    external fun nativeGtsamSmoke(): Double

    external fun nativeCreate(): Long
    external fun nativeDestroy(handle: Long)
    external fun nativeStart(handle: Long)
    external fun nativeStop(handle: Long)
    external fun nativeReset(handle: Long)
    external fun nativeLoadConfig(handle: Long, params: DoubleArray)
    external fun nativeLoadVocabulary(handle: Long, path: String): Boolean

    external fun nativePushFrame(
        handle: Long, y: ByteArray, w: Int, h: Int, rowStride: Int,
        tNs: Long, rsSkewNs: Long, exposureS: Double,
        fx: Double, fy: Double, cx: Double, cy: Double
    )
    external fun nativePushImu(
        handle: Long, tNs: Long,
        ax: Float, ay: Float, az: Float, gx: Float, gy: Float, gz: Float
    )
    external fun nativePushGnssFix(
        handle: Long, tNs: Long, lat: Double, lon: Double, alt: Double,
        hAcc: Double, vAcc: Double, nSats: Int, hasAlt: Boolean
    )
    external fun nativePushGnssRaw(
        handle: Long, clkTNs: Long, clkTimeNs: Long, fullBiasNs: Long,
        biasNs: Double, driftNsps: Double, leapSec: Int, fullBiasValid: Boolean,
        svid: IntArray, constellation: IntArray, cn0: DoubleArray, state: IntArray,
        rxSvTimeNs: LongArray, rxSvTimeUncNs: LongArray, prRate: DoubleArray,
        prRateUnc: DoubleArray, carrierFreq: DoubleArray, timeOffsetNs: LongArray
    )
    external fun nativePushNav(
        handle: Long, constellation: Int, svid: Int, type: Int,
        subId: Int, msgId: Int, data: ByteArray
    )

    external fun nativeGetPose(handle: Long, out: DoubleArray)        // 26 doubles
    external fun nativeGetTelemetry(handle: Long, out: DoubleArray)   // 40 doubles
    external fun nativeGetMapSnapshot(handle: Long): FloatArray?
    external fun nativeGetFeatures(handle: Long): FloatArray?   // [n, (u,v,age,isNew)*n]

    // checkerboard calibration
    external fun nativeSetCalibActive(handle: Long, on: Boolean, cols: Int, rows: Int, square: Double)
    external fun nativeCaptureCalibView(handle: Long): Int
    external fun nativeCalibViewCount(handle: Long): Int
    external fun nativeResetCalibration(handle: Long)
    external fun nativeCalibCorners(handle: Long): FloatArray?
    external fun nativeRunCalibration(handle: Long, out: DoubleArray)  // 14 doubles

    external fun nativeSetGnssMode(handle: Long, mode: Int)
    external fun nativeSetOnlineCalib(handle: Long, td: Boolean, extr: Boolean)
    external fun nativeSetRollingShutter(handle: Long, on: Boolean)
    external fun nativeSetLoopClosure(handle: Long, on: Boolean)
    external fun nativeSetThermal(handle: Long, level: Int)
}
