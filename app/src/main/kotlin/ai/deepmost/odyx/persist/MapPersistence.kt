package ai.deepmost.odyx.persist

import ai.deepmost.odyx.jni.MapSnapshotK
import timber.log.Timber
import java.io.DataInputStream
import java.io.DataOutputStream
import java.io.File

/**
 * Persists a sparse map snapshot (trajectory + landmarks) to a compact binary
 * file so a session's reconstructed map survives an app restart and can be
 * inspected/exported offline. Header: magic, version, counts; then float blocks.
 */
object MapPersistence {
    private const val MAGIC = 0x4F445958          // "ODYX"
    private const val VERSION = 1

    fun save(file: File, m: MapSnapshotK) {
        file.parentFile?.mkdirs()
        DataOutputStream(file.outputStream().buffered()).use { o ->
            o.writeInt(MAGIC); o.writeInt(VERSION)
            o.writeInt(m.trajLocal.size); o.writeInt(m.trajEnu.size)
            o.writeInt(m.landmarks.size); o.writeInt(m.loopEdges.size); o.writeInt(m.gnss.size)
            o.writeBoolean(m.enuValid)
            fun put(a: FloatArray) { for (v in a) o.writeFloat(v) }
            put(m.trajLocal); put(m.trajEnu); put(m.landmarks); put(m.loopEdges); put(m.gnss)
        }
        Timber.tag("Map").i("saved %d kf, %d landmarks -> %s",
            m.trajLocal.size / 3, m.landmarks.size / 4, file.name)
    }

    fun load(file: File): MapSnapshotK? {
        if (!file.exists()) return null
        return try {
            DataInputStream(file.inputStream().buffered()).use { i ->
                require(i.readInt() == MAGIC && i.readInt() == VERSION) { "bad map header" }
                val nTL = i.readInt(); val nTE = i.readInt(); val nLm = i.readInt()
                val nLp = i.readInt(); val nG = i.readInt(); val enuValid = i.readBoolean()
                fun take(n: Int) = FloatArray(n) { i.readFloat() }
                MapSnapshotK(take(nTL), take(nTE), take(nLm), take(nLp), take(nG), enuValid)
            }
        } catch (e: Exception) { Timber.tag("Map").w(e, "load failed"); null }
    }
}
