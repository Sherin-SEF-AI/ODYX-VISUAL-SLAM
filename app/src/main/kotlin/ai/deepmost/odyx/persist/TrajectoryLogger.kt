package ai.deepmost.odyx.persist

import ai.deepmost.odyx.jni.PoseSnapshot
import timber.log.Timber
import java.io.BufferedWriter
import java.io.File

/**
 * Logs the emitted pose stream to a **TUM-format** trajectory file:
 *
 *   timestamp[s] tx ty tz qx qy qz qw
 *
 * This is the format consumed by `evo` and by the ORB-SLAM3 / VINS-Fusion
 * offline-validation workflow (README §6). Local and ENU trajectories are logged
 * to separate files; only valid poses are written.
 */
class TrajectoryLogger(dir: File, tag: String) {
    private val localFile = File(dir, "trajectory_local_$tag.tum")
    private val enuFile = File(dir, "trajectory_enu_$tag.tum")
    private var local: BufferedWriter? = null
    private var enu: BufferedWriter? = null
    private var lastTns = 0L

    init {
        dir.mkdirs()
        local = localFile.bufferedWriter().also { it.appendLine("# timestamp tx ty tz qx qy qz qw (local gravity-aligned world)") }
        enu = enuFile.bufferedWriter().also { it.appendLine("# timestamp tx ty tz qx qy qz qw (global ENU)") }
    }

    fun log(p: PoseSnapshot) {
        if (!p.valid || p.tNs == lastTns) return
        lastTns = p.tNs
        val t = p.tNs * 1e-9
        // ENU has no orientation track here, so reuse the local orientation
        // (yaw-aligned) for the ENU file — position is what validation compares.
        local?.appendLine("%.9f %.6f %.6f %.6f %.6f %.6f %.6f %.6f".format(
            t, p.p[0], p.p[1], p.p[2], p.q[1], p.q[2], p.q[3], p.q[0]))
        p.enu?.let { e ->
            enu?.appendLine("%.9f %.6f %.6f %.6f %.6f %.6f %.6f %.6f".format(
                t, e[0], e[1], e[2], p.q[1], p.q[2], p.q[3], p.q[0]))
        }
    }

    fun close() {
        try { local?.flush(); local?.close() } catch (e: Exception) { Timber.tag("Traj").w(e, "close local") }
        try { enu?.flush(); enu?.close() } catch (e: Exception) { Timber.tag("Traj").w(e, "close enu") }
        local = null; enu = null
        Timber.tag("Traj").i("trajectory written: %s", localFile.absolutePath)
    }

    fun files(): List<File> = listOf(localFile, enuFile)
}
