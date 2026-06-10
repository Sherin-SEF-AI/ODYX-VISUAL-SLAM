package ai.deepmost.odyx.session

import android.content.Context
import java.io.File

/** Metadata for a recorded raw session. */
data class SessionInfo(
    val name: String,
    val dir: File,
    val frames: Int,
    val durationS: Double,
    val sizeBytes: Long
)

/**
 * Sessions live under filesDir/sessions/<name>/mav0 in EuRoC/ASL layout:
 *   mav0/cam0/data/<tns>.png + mav0/cam0/data.csv
 *   mav0/imu0/data.csv
 *   mav0/gnss0/data.csv
 *   mav0/calib.txt
 */
class SessionStore(private val context: Context) {
    val root: File = File(context.filesDir, "sessions").apply { mkdirs() }

    fun sessionDir(name: String) = File(root, name)
    fun mav0(name: String) = File(sessionDir(name), "mav0")

    fun list(): List<SessionInfo> =
        (root.listFiles { f -> f.isDirectory }?.toList() ?: emptyList()).map { d ->
            val cam = File(d, "mav0/cam0/data.csv")
            val lines = if (cam.exists()) cam.readLines().filter { it.isNotBlank() && !it.startsWith("#") } else emptyList()
            val frames = lines.size
            val dur = if (frames >= 2) {
                val t0 = lines.first().substringBefore(',').toLongOrNull() ?: 0L
                val t1 = lines.last().substringBefore(',').toLongOrNull() ?: 0L
                (t1 - t0) * 1e-9
            } else 0.0
            SessionInfo(d.name, d, frames, dur, dirSize(d))
        }.sortedByDescending { it.name }

    fun delete(name: String) { sessionDir(name).deleteRecursively() }

    private fun dirSize(d: File): Long =
        d.walkTopDown().filter { it.isFile }.map { it.length() }.sum()
}
