package ai.deepmost.odyx.core

/**
 * Destination for unified-timestamp sensor data. [OdyxEngine] implements it
 * (pushing into native); [session.SessionRecorder] implements it (writing an
 * EuRoC/ASL session); [CompositeSink] fans out to both so recording is a clean
 * tap that never perturbs the live estimate.
 */
interface SensorSink {
    fun pushFrame(
        y: ByteArray, w: Int, h: Int, rowStride: Int, tNs: Long,
        rsSkewNs: Long, exposureS: Double, fx: Double, fy: Double, cx: Double, cy: Double
    )
    fun pushImu(tNs: Long, ax: Float, ay: Float, az: Float, gx: Float, gy: Float, gz: Float)
    fun pushGnssFix(tNs: Long, lat: Double, lon: Double, alt: Double, hAcc: Double, vAcc: Double, nSats: Int, hasAlt: Boolean)
    fun pushGnssRaw(
        clkTNs: Long, clkTimeNs: Long, fullBiasNs: Long, biasNs: Double, driftNsps: Double,
        leapSec: Int, fullBiasValid: Boolean,
        svid: IntArray, constellation: IntArray, cn0: DoubleArray, state: IntArray,
        rxSvTimeNs: LongArray, rxSvTimeUncNs: LongArray, prRate: DoubleArray,
        prRateUnc: DoubleArray, carrierFreq: DoubleArray, timeOffsetNs: LongArray
    )
    fun pushNav(constellation: Int, svid: Int, type: Int, subId: Int, msgId: Int, data: ByteArray)
}

/** Sink that drops everything — used to isolate the live stream during replay. */
object NoOpSink : SensorSink {
    override fun pushFrame(y: ByteArray, w: Int, h: Int, rowStride: Int, tNs: Long, rsSkewNs: Long, exposureS: Double, fx: Double, fy: Double, cx: Double, cy: Double) {}
    override fun pushImu(tNs: Long, ax: Float, ay: Float, az: Float, gx: Float, gy: Float, gz: Float) {}
    override fun pushGnssFix(tNs: Long, lat: Double, lon: Double, alt: Double, hAcc: Double, vAcc: Double, nSats: Int, hasAlt: Boolean) {}
    override fun pushGnssRaw(clkTNs: Long, clkTimeNs: Long, fullBiasNs: Long, biasNs: Double, driftNsps: Double, leapSec: Int, fullBiasValid: Boolean, svid: IntArray, constellation: IntArray, cn0: DoubleArray, state: IntArray, rxSvTimeNs: LongArray, rxSvTimeUncNs: LongArray, prRate: DoubleArray, prRateUnc: DoubleArray, carrierFreq: DoubleArray, timeOffsetNs: LongArray) {}
    override fun pushNav(constellation: Int, svid: Int, type: Int, subId: Int, msgId: Int, data: ByteArray) {}
}

/** Fan-out sink: every push goes to all targets in order. */
class CompositeSink(private val targets: List<SensorSink>) : SensorSink {
    override fun pushFrame(y: ByteArray, w: Int, h: Int, rowStride: Int, tNs: Long, rsSkewNs: Long, exposureS: Double, fx: Double, fy: Double, cx: Double, cy: Double) {
        targets.forEach { it.pushFrame(y, w, h, rowStride, tNs, rsSkewNs, exposureS, fx, fy, cx, cy) }
    }
    override fun pushImu(tNs: Long, ax: Float, ay: Float, az: Float, gx: Float, gy: Float, gz: Float) {
        targets.forEach { it.pushImu(tNs, ax, ay, az, gx, gy, gz) }
    }
    override fun pushGnssFix(tNs: Long, lat: Double, lon: Double, alt: Double, hAcc: Double, vAcc: Double, nSats: Int, hasAlt: Boolean) {
        targets.forEach { it.pushGnssFix(tNs, lat, lon, alt, hAcc, vAcc, nSats, hasAlt) }
    }
    override fun pushGnssRaw(clkTNs: Long, clkTimeNs: Long, fullBiasNs: Long, biasNs: Double, driftNsps: Double, leapSec: Int, fullBiasValid: Boolean, svid: IntArray, constellation: IntArray, cn0: DoubleArray, state: IntArray, rxSvTimeNs: LongArray, rxSvTimeUncNs: LongArray, prRate: DoubleArray, prRateUnc: DoubleArray, carrierFreq: DoubleArray, timeOffsetNs: LongArray) {
        targets.forEach { it.pushGnssRaw(clkTNs, clkTimeNs, fullBiasNs, biasNs, driftNsps, leapSec, fullBiasValid, svid, constellation, cn0, state, rxSvTimeNs, rxSvTimeUncNs, prRate, prRateUnc, carrierFreq, timeOffsetNs) }
    }
    override fun pushNav(constellation: Int, svid: Int, type: Int, subId: Int, msgId: Int, data: ByteArray) {
        targets.forEach { it.pushNav(constellation, svid, type, subId, msgId, data) }
    }
}
