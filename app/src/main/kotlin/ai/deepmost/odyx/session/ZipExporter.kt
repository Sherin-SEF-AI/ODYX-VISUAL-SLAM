package ai.deepmost.odyx.session

import java.io.File
import java.io.FileOutputStream
import java.util.zip.ZipEntry
import java.util.zip.ZipOutputStream

/** Export a recorded session directory to a single EuRoC/ASL-style zip. */
object ZipExporter {
    fun export(sessionDir: File, outZip: File) {
        ZipOutputStream(FileOutputStream(outZip).buffered()).use { zos ->
            sessionDir.walkTopDown().filter { it.isFile }.forEach { f ->
                val rel = f.relativeTo(sessionDir.parentFile!!).path
                zos.putNextEntry(ZipEntry(rel))
                f.inputStream().use { it.copyTo(zos) }
                zos.closeEntry()
            }
        }
    }
}
