package ai.deepmost.odyx.session

import android.graphics.Bitmap
import ai.deepmost.odyx.core.SensorSink
import timber.log.Timber
import java.io.File
import java.io.FileOutputStream
import java.util.concurrent.Executors
import java.util.concurrent.atomic.AtomicInteger

/**
 * Records a synchronized raw session in EuRoC/ASL layout. Implements [SensorSink]
 * so it taps the SAME unified-timestamp stream the estimator consumes. All disk
 * work runs on a dedicated single-thread executor so capture is never blocked.
 *
 * Frames are stored as 8-bit grayscale PNGs (the camera Y plane), which is what
 * the front-end ingests on replay — bit-exact to the live path.
 */
class SessionRecorder(private val store: SessionStore, val name: String) : SensorSink {
    private val mav0 = store.mav0(name)
    private val camDir = File(mav0, "cam0/data").apply { mkdirs() }
    private val camCsv = File(mav0, "cam0/data.csv")
    private val imuCsv = File(mav0, "imu0/data.csv").apply { parentFile?.mkdirs() }
    private val gnssCsv = File(mav0, "gnss0/data.csv").apply { parentFile?.mkdirs() }
    private val rawCsv = File(mav0, "gnss0/raw.csv")
    private val calib = File(mav0, "calib.txt")

    private val io = Executors.newSingleThreadExecutor()
    private val frameCount = AtomicInteger(0)
    @Volatile private var calibWritten = false

    init {
        camCsv.writeText("#timestamp[ns],filename\n")
        imuCsv.writeText("#timestamp[ns],wx,wy,wz,ax,ay,az\n")
        gnssCsv.writeText("#timestamp[ns],lat,lon,alt,hAcc,vAcc,nSats\n")
        rawCsv.writeText("#clkTNs,svid,constellation,cn0,state,rxSvTimeNs,prRate,carrierHz\n")
    }

    fun frames(): Int = frameCount.get()

    override fun pushFrame(y: ByteArray, w: Int, h: Int, rowStride: Int, tNs: Long, rsSkewNs: Long, exposureS: Double, fx: Double, fy: Double, cx: Double, cy: Double) {
        if (!calibWritten) {
            calibWritten = true
            io.execute {
                calib.writeText(
                    "width $w\nheight $h\nfx $fx\nfy $fy\ncx $cx\ncy $cy\n" +
                        "rolling_shutter_skew_ns $rsSkewNs\n"
                )
            }
        }
        // copy luma (deep) so the byte array can be reused by the camera thread
        val packed = ByteArray(w * h)
        for (r in 0 until h) System.arraycopy(y, r * rowStride, packed, r * w, w)
        io.execute {
            try {
                val bmp = Bitmap.createBitmap(w, h, Bitmap.Config.ARGB_8888)
                val px = IntArray(w * h)
                for (i in 0 until w * h) { val g = packed[i].toInt() and 0xFF; px[i] = (0xFF shl 24) or (g shl 16) or (g shl 8) or g }
                bmp.setPixels(px, 0, w, 0, 0, w, h)
                val f = File(camDir, "$tNs.png")
                FileOutputStream(f).use { bmp.compress(Bitmap.CompressFormat.PNG, 100, it) }
                bmp.recycle()
                camCsv.appendText("$tNs,$tNs.png\n")
                frameCount.incrementAndGet()
            } catch (e: Exception) { Timber.tag("Recorder").e(e, "frame write") }
        }
    }

    override fun pushImu(tNs: Long, ax: Float, ay: Float, az: Float, gx: Float, gy: Float, gz: Float) {
        io.execute { imuCsv.appendText("$tNs,$gx,$gy,$gz,$ax,$ay,$az\n") }
    }

    override fun pushGnssFix(tNs: Long, lat: Double, lon: Double, alt: Double, hAcc: Double, vAcc: Double, nSats: Int, hasAlt: Boolean) {
        io.execute { gnssCsv.appendText("$tNs,$lat,$lon,$alt,$hAcc,$vAcc,$nSats\n") }
    }

    override fun pushGnssRaw(clkTNs: Long, clkTimeNs: Long, fullBiasNs: Long, biasNs: Double, driftNsps: Double, leapSec: Int, fullBiasValid: Boolean, svid: IntArray, constellation: IntArray, cn0: DoubleArray, state: IntArray, rxSvTimeNs: LongArray, rxSvTimeUncNs: LongArray, prRate: DoubleArray, prRateUnc: DoubleArray, carrierFreq: DoubleArray, timeOffsetNs: LongArray) {
        io.execute {
            val sb = StringBuilder()
            for (i in svid.indices)
                sb.append("$clkTNs,${svid[i]},${constellation[i]},${cn0[i]},${state[i]},${rxSvTimeNs[i]},${prRate[i]},${carrierFreq[i]}\n")
            rawCsv.appendText(sb.toString())
        }
    }

    override fun pushNav(constellation: Int, svid: Int, type: Int, subId: Int, msgId: Int, data: ByteArray) {
        // navigation messages are not needed for deterministic VIO replay; the
        // raw.csv above carries the per-sat measurements used by tight mode.
    }

    fun close() { io.shutdown() }
}
