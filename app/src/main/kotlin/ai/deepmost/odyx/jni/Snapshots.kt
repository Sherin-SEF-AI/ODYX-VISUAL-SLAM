package ai.deepmost.odyx.jni

/** Decoded pose snapshot (mirrors nativeGetPose layout, 26 doubles). */
data class PoseSnapshot(
    val valid: Boolean,
    val tNs: Long,
    val q: FloatArray,          // w,x,y,z
    val p: FloatArray,          // x,y,z (local world)
    val v: FloatArray,
    val accelBias: FloatArray,
    val gyroBias: FloatArray,
    val covPosTrace: Double,
    val covRotTrace: Double,
    val td: Double,
    val enu: FloatArray?,       // ENU position if anchored
) {
    companion object {
        const val SIZE = 26
        fun from(o: DoubleArray) = PoseSnapshot(
            valid = o[0] != 0.0,
            tNs = o[1].toLong(),
            q = floatArrayOf(o[2].toFloat(), o[3].toFloat(), o[4].toFloat(), o[5].toFloat()),
            p = floatArrayOf(o[6].toFloat(), o[7].toFloat(), o[8].toFloat()),
            v = floatArrayOf(o[9].toFloat(), o[10].toFloat(), o[11].toFloat()),
            accelBias = floatArrayOf(o[12].toFloat(), o[13].toFloat(), o[14].toFloat()),
            gyroBias = floatArrayOf(o[15].toFloat(), o[16].toFloat(), o[17].toFloat()),
            covPosTrace = o[18], covRotTrace = o[19], td = o[20],
            enu = if (o[24] != 0.0) floatArrayOf(o[21].toFloat(), o[22].toFloat(), o[23].toFloat()) else null
        )
    }
}

enum class TrackingState { UNINIT, INITIALIZING, NOMINAL, UNSTABLE, LOST;
    companion object { fun of(i: Int) = entries.getOrElse(i) { UNINIT } } }

enum class InitStateK { COLLECTING, SFM, VIALIGN, DONE, FAILED;
    companion object { fun of(i: Int) = entries.getOrElse(i) { COLLECTING } } }

/** Decoded telemetry (mirrors nativeGetTelemetry layout, 40 doubles). */
data class TelemetrySnapshot(
    val tracking: TrackingState,
    val init: InitStateK,
    val gnssMode: Int,
    val nTracked: Int, val nKeyframes: Int, val windowSize: Int, val nLandmarks: Int,
    val frontendMs: Double, val optimizeMs: Double,
    val imuRate: Double, val frameRate: Double, val gnssRate: Double,
    val td: Double,
    val gyroBias: FloatArray, val accelBias: FloatArray, val vel: FloatArray,
    val gravityResid: Double,
    val gnssSats: Int, val gnssFixType: Int, val gnssAcc: Double,
    val enuAligned: Boolean, val enuResid: Double,
    val loopCount: Int, val driftProxy: Double, val thermal: Int,
    val frameStride: Int, val targetFeatures: Int, val calibDefault: Boolean,
) {
    companion object {
        const val SIZE = 40
        fun from(o: DoubleArray) = TelemetrySnapshot(
            tracking = TrackingState.of(o[0].toInt()),
            init = InitStateK.of(o[1].toInt()),
            gnssMode = o[2].toInt(),
            nTracked = o[3].toInt(), nKeyframes = o[4].toInt(),
            windowSize = o[5].toInt(), nLandmarks = o[6].toInt(),
            frontendMs = o[7], optimizeMs = o[8],
            imuRate = o[9], frameRate = o[10], gnssRate = o[11],
            td = o[12],
            gyroBias = floatArrayOf(o[13].toFloat(), o[14].toFloat(), o[15].toFloat()),
            accelBias = floatArrayOf(o[16].toFloat(), o[17].toFloat(), o[18].toFloat()),
            vel = floatArrayOf(o[19].toFloat(), o[20].toFloat(), o[21].toFloat()),
            gravityResid = o[22],
            gnssSats = o[23].toInt(), gnssFixType = o[24].toInt(), gnssAcc = o[25],
            enuAligned = o[26] != 0.0, enuResid = o[27],
            loopCount = o[28].toInt(), driftProxy = o[29], thermal = o[30].toInt(),
            frameStride = o[31].toInt(), targetFeatures = o[32].toInt(),
            calibDefault = o[33] != 0.0
        )
    }
}

/** Decoded map snapshot (mirrors nativeGetMapSnapshot packed FloatArray). */
class MapSnapshotK(
    val trajLocal: FloatArray,   // 3*n
    val trajEnu: FloatArray,     // 3*n
    val landmarks: FloatArray,   // 4*n (x,y,z,covtr)
    val loopEdges: FloatArray,   // 2*n (a,b)
    val gnss: FloatArray,        // 3*n
    val enuValid: Boolean,
) {
    companion object {
        fun from(buf: FloatArray?): MapSnapshotK? {
            if (buf == null || buf.size < 6) return null
            val nTL = buf[0].toInt(); val nTE = buf[1].toInt()
            val nLm = buf[2].toInt(); val nLp = buf[3].toInt(); val nG = buf[4].toInt()
            var off = 6
            fun take(count: Int): FloatArray {
                val a = buf.copyOfRange(off, off + count); off += count; return a
            }
            return MapSnapshotK(
                trajLocal = take(3 * nTL),
                trajEnu = take(3 * nTE),
                landmarks = take(4 * nLm),
                loopEdges = take(2 * nLp),
                gnss = take(3 * nG),
                enuValid = buf[5] != 0f
            )
        }
    }
}
