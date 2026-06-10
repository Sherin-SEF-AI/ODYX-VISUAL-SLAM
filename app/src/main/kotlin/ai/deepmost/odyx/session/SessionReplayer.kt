package ai.deepmost.odyx.session

import android.graphics.BitmapFactory
import ai.deepmost.odyx.core.OdyxEngine
import timber.log.Timber
import java.io.File

/**
 * Replays a recorded EuRoC/ASL session deterministically through the SAME native
 * estimator (for tuning + regression). Streams are merged by timestamp and fed
 * in chronological order at a selectable speed (1.0 = real time, 0 = as fast as
 * possible). Camera intrinsics come from the recorded calib.txt.
 */
class SessionReplayer(private val store: SessionStore, private val engine: OdyxEngine) {

    @Volatile var running = false; private set

    data class Calib(val w: Int, val h: Int, val fx: Double, val fy: Double, val cx: Double, val cy: Double, val skewNs: Long)

    private fun readCalib(mav0: File): Calib {
        val m = HashMap<String, String>()
        File(mav0, "calib.txt").takeIf { it.exists() }?.readLines()?.forEach {
            val parts = it.trim().split(' ')
            if (parts.size == 2) m[parts[0]] = parts[1]
        }
        return Calib(
            m["width"]?.toIntOrNull() ?: 640, m["height"]?.toIntOrNull() ?: 480,
            m["fx"]?.toDoubleOrNull() ?: 480.0, m["fy"]?.toDoubleOrNull() ?: 480.0,
            m["cx"]?.toDoubleOrNull() ?: 320.0, m["cy"]?.toDoubleOrNull() ?: 240.0,
            m["rolling_shutter_skew_ns"]?.toLongOrNull() ?: 0L
        )
    }

    private sealed class Ev(val t: Long) {
        class Imu(t: Long, val v: FloatArray) : Ev(t)
        class Cam(t: Long, val file: File) : Ev(t)
        class Gnss(t: Long, val v: DoubleArray) : Ev(t)
    }

    /** Blocking replay. Run on a background thread. [speed]=1.0 real-time. */
    fun replay(name: String, speed: Double = 0.0, onProgress: (Double) -> Unit = {}) {
        val mav0 = store.mav0(name)
        val calib = readCalib(mav0)
        val events = ArrayList<Ev>()

        File(mav0, "imu0/data.csv").takeIf { it.exists() }?.forEachDataLine { c ->
            if (c.size >= 7) events.add(Ev.Imu(c[0].toLong(),
                floatArrayOf(c[4].toFloat(), c[5].toFloat(), c[6].toFloat(),  // ax,ay,az
                             c[1].toFloat(), c[2].toFloat(), c[3].toFloat()))) // wx,wy,wz
        }
        File(mav0, "gnss0/data.csv").takeIf { it.exists() }?.forEachDataLine { c ->
            if (c.size >= 7) events.add(Ev.Gnss(c[0].toLong(),
                doubleArrayOf(c[1].toDouble(), c[2].toDouble(), c[3].toDouble(), c[4].toDouble(), c[5].toDouble(), c[6].toDouble())))
        }
        File(mav0, "cam0/data.csv").takeIf { it.exists() }?.forEachDataLine { c ->
            if (c.size >= 2) events.add(Ev.Cam(c[0].toLong(), File(mav0, "cam0/data/${c[1]}")))
        }
        events.sortBy { it.t }
        if (events.isEmpty()) { Timber.tag("Replay").w("no events for %s", name); return }

        running = true
        engine.reset()
        val t0 = events.first().t
        val wall0 = System.nanoTime()
        val total = events.size
        for ((i, e) in events.withIndex()) {
            if (!running) break
            if (speed > 0) {
                val targetWall = wall0 + ((e.t - t0) / speed).toLong()
                val sleep = (targetWall - System.nanoTime()) / 1_000_000L
                if (sleep > 0) Thread.sleep(sleep)
            }
            when (e) {
                is Ev.Imu -> engine.pushImu(e.t, e.v[0], e.v[1], e.v[2], e.v[3], e.v[4], e.v[5])
                is Ev.Gnss -> engine.pushGnssFix(e.t, e.v[0], e.v[1], e.v[2], e.v[3], e.v[4], e.v[5].toInt(), true)
                is Ev.Cam -> {
                    val bmp = BitmapFactory.decodeFile(e.file.absolutePath) ?: continue
                    val w = bmp.width; val h = bmp.height
                    val px = IntArray(w * h); bmp.getPixels(px, 0, w, 0, 0, w, h)
                    val gray = ByteArray(w * h)
                    for (j in 0 until w * h) gray[j] = (px[j] and 0xFF).toByte()  // R==G==B
                    bmp.recycle()
                    engine.pushFrame(gray, w, h, w, e.t, calib.skewNs, 0.0, calib.fx, calib.fy, calib.cx, calib.cy)
                }
            }
            if (i % 50 == 0) onProgress(i.toDouble() / total)
        }
        onProgress(1.0)
        running = false
        Timber.tag("Replay").i("replayed %d events from %s", events.size, name)
    }

    fun stop() { running = false }

    private inline fun File.forEachDataLine(block: (List<String>) -> Unit) {
        bufferedReader().useLines { seq ->
            seq.forEach { line ->
                if (line.isNotBlank() && !line.startsWith("#")) block(line.split(','))
            }
        }
    }
}
