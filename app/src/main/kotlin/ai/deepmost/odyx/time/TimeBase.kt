package ai.deepmost.odyx.time

import android.hardware.camera2.CameraCharacteristics
import android.os.SystemClock
import timber.log.Timber

/**
 * PRIORITY ZERO: reduce every sensor stream to ONE monotonic nanosecond clock —
 * Android `elapsedRealtimeNanos` — before anything is pushed across JNI.
 *
 * Clock facts on Android:
 *  - IMU `SensorEvent.timestamp` is `elapsedRealtimeNanos` on virtually all
 *    modern devices (the HAL is required to use the boot clock). We treat it as
 *    already on the target base and verify with a bounded skew check.
 *  - Camera `SENSOR_TIMESTAMP`'s base is reported by
 *    `SENSOR_INFO_TIMESTAMP_SOURCE`:
 *      REALTIME → same `elapsedRealtimeNanos` base (no offset).
 *      UNKNOWN  → a `SystemClock.uptimeNanos`-like monotonic base that PAUSES in
 *                 deep sleep; we measure the offset (elapsedRealtime - uptime)
 *                 once per frame and add it, which is robust while the screen is
 *                 on (the only time frames arrive).
 *
 * The residual camera↔IMU offset that survives this reduction is the online
 * state `td`, estimated inside the smoother. This class only removes the
 * *clock-base* difference, never the latency — that is td's job.
 */
class TimeBase(timestampSource: Int) {

    enum class CameraSource { REALTIME, UNKNOWN }

    val cameraSource: CameraSource =
        if (timestampSource == CameraCharacteristics.SENSOR_INFO_TIMESTAMP_SOURCE_REALTIME)
            CameraSource.REALTIME else CameraSource.UNKNOWN

    init {
        Timber.tag("TimeBase").i("camera timestamp source = %s", cameraSource)
    }

    /** Map a camera SENSOR_TIMESTAMP to the elapsedRealtimeNanos base. */
    fun cameraToUnified(sensorTimestampNs: Long): Long = when (cameraSource) {
        CameraSource.REALTIME -> sensorTimestampNs
        CameraSource.UNKNOWN -> sensorTimestampNs + uptimeToRealtimeOffsetNs()
    }

    /** IMU SensorEvent.timestamp is already elapsedRealtimeNanos on supported devices. */
    fun imuToUnified(eventTimestampNs: Long): Long = eventTimestampNs

    /**
     * GNSS Location.getElapsedRealtimeNanos() (API 29+) is already on-base. For
     * older fixes we approximate with the receive instant; callers should pass
     * the elapsedRealtimeNanos when available.
     */
    fun gnssToUnified(elapsedRealtimeNanosIfAvailable: Long?): Long =
        elapsedRealtimeNanosIfAvailable ?: SystemClock.elapsedRealtimeNanos()

    companion object {
        /** offset to add to an uptime-based stamp to reach the realtime base. */
        fun uptimeToRealtimeOffsetNs(): Long =
            SystemClock.elapsedRealtimeNanos() - SystemClock.uptimeMillis() * 1_000_000L
    }
}
