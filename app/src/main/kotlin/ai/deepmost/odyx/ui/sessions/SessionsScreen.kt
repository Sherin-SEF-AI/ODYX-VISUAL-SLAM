package ai.deepmost.odyx.ui.sessions

import androidx.compose.foundation.background
import androidx.compose.foundation.layout.*
import androidx.compose.foundation.lazy.LazyColumn
import androidx.compose.foundation.lazy.items
import androidx.compose.material3.*
import androidx.compose.runtime.*
import android.content.Context
import android.content.Intent
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.platform.LocalContext
import androidx.compose.ui.unit.dp
import androidx.core.content.FileProvider
import androidx.lifecycle.compose.collectAsStateWithLifecycle
import ai.deepmost.odyx.allan.AllanLogger
import ai.deepmost.odyx.core.OdyxController
import ai.deepmost.odyx.session.SessionInfo
import ai.deepmost.odyx.session.SessionReplayer
import ai.deepmost.odyx.session.ZipExporter
import ai.deepmost.odyx.ui.common.Hairline
import ai.deepmost.odyx.ui.common.Readout
import ai.deepmost.odyx.ui.common.SectionHeader
import ai.deepmost.odyx.ui.theme.*
import kotlinx.coroutines.Dispatchers
import kotlinx.coroutines.launch
import kotlinx.coroutines.withContext
import java.io.File

@Composable
fun SessionsScreen(controller: OdyxController, modifier: Modifier = Modifier) {
    val scope = rememberCoroutineScope()
    val ctx = LocalContext.current
    val recording by controller.recording.collectAsStateWithLifecycle()
    val loggingTraj by controller.loggingTraj.collectAsStateWithLifecycle()
    var sessions by remember { mutableStateOf<List<SessionInfo>>(emptyList()) }
    var status by remember { mutableStateOf("") }
    var allanProgress by remember { mutableStateOf<Double?>(null) }

    fun refresh() { sessions = controller.sessionStore.list() }
    LaunchedEffect(recording) { refresh() }

    val replayer = remember { SessionReplayer(controller.sessionStore, controller.engine) }

    Column(modifier.fillMaxSize().background(OdyxBlack)) {
        SectionHeader("RECORD")
        Row(Modifier.fillMaxWidth().padding(12.dp), horizontalArrangement = Arrangement.spacedBy(8.dp),
            verticalAlignment = Alignment.CenterVertically) {
            if (recording == null) {
                Button(onClick = {
                    val name = "sess_" + System.currentTimeMillis()
                    controller.startRecording(name); status = "recording $name"
                }) { Text("● REC") }
            } else {
                Button(onClick = { controller.stopRecording(); status = "saved"; refresh() },
                    colors = ButtonDefaults.buttonColors(containerColor = OdyxBad)) { Text("■ STOP") }
                Text(recording!!, color = OdyxAccent)
            }
            // TUM trajectory export for offline validation vs ORB-SLAM3/VINS-Fusion.
            if (!loggingTraj) {
                OutlinedButton(onClick = { controller.startTrajectoryLog("t" + System.currentTimeMillis()) }) {
                    Text("LOG TRAJ (TUM)")
                }
            } else {
                OutlinedButton(onClick = {
                    val files = controller.stopTrajectoryLog()
                    status = "trajectory written: ${files.firstOrNull()?.name ?: "-"}"
                }) { Text("STOP TRAJ") }
            }
            OutlinedButton(onClick = {
                scope.launch {
                    val f = withContext(Dispatchers.IO) { controller.saveMap("m" + System.currentTimeMillis()) }
                    if (f != null) { status = "map saved: ${f.name}"; shareFile(ctx, f, "application/octet-stream") }
                    else status = "no map yet"
                }
            }) { Text("SAVE+SHARE MAP") }
        }
        Hairline()

        SectionHeader("ALLAN-VARIANCE IMU CALIBRATION")
        Row(Modifier.fillMaxWidth().padding(12.dp), horizontalArrangement = Arrangement.spacedBy(8.dp),
            verticalAlignment = Alignment.CenterVertically) {
            val logger = remember { AllanLogger(controller.appContext()) }
            Button(onClick = {
                allanProgress = 0.0
                logger.start(2.0, object : AllanLogger.Listener {
                    override fun onProgress(seconds: Double, total: Double) { allanProgress = seconds / total }
                    override fun onDone(result: AllanLogger.Result) {
                        allanProgress = null
                        status = "imu_noise.yaml written (accN=%.3g gyrN=%.3g)".format(result.accN, result.gyrN)
                    }
                    override fun onError(msg: String) { allanProgress = null; status = "Allan error: $msg" }
                })
            }, enabled = allanProgress == null) { Text("RECORD 2 min STATIC") }
            allanProgress?.let { Text("${(it * 100).toInt()}%", color = OdyxAccent) }
        }
        Text("Place the phone perfectly still on a rigid surface, then start.",
            color = OdyxTextDim, style = MaterialTheme.typography.labelSmall,
            modifier = Modifier.padding(horizontal = 12.dp))
        Hairline()

        SectionHeader("RECORDED SESSIONS")
        if (status.isNotEmpty()) Readout("status", status)
        LazyColumn(Modifier.weight(1f)) {
            items(sessions) { s ->
                Column(Modifier.fillMaxWidth().padding(12.dp)) {
                    Text(s.name, color = OdyxText, style = MaterialTheme.typography.titleMedium)
                    Readout("frames", "${s.frames}")
                    Readout("duration", "%.1f s".format(s.durationS))
                    Readout("size", "%.1f MB".format(s.sizeBytes / 1e6))
                    Row(horizontalArrangement = Arrangement.spacedBy(8.dp)) {
                        TextButton(onClick = {
                            scope.launch {
                                status = "replaying ${s.name}… (live capture isolated)"
                                controller.replaySession(s.name) { p -> status = "replay ${(p * 100).toInt()}%" }
                                status = "replay done — check MAP/TELEMETRY"
                            }
                        }) { Text("REPLAY") }
                        TextButton(onClick = {
                            scope.launch {
                                status = "zipping ${s.name}…"
                                val out = File(controller.sessionStore.root, "${s.name}.zip")
                                withContext(Dispatchers.IO) { ZipExporter.export(s.dir, out) }
                                status = "exported ${out.name}"
                                shareFile(ctx, out, "application/zip")   // share chooser
                            }
                        }) { Text("EXPORT") }
                        TextButton(onClick = { controller.sessionStore.delete(s.name); refresh() }) {
                            Text("DELETE", color = OdyxBad)
                        }
                    }
                    Hairline()
                }
            }
        }
    }
}

/** Share a file from the app's private dir via FileProvider + ACTION_SEND chooser. */
private fun shareFile(ctx: Context, file: File, mime: String) {
    if (!file.exists()) return
    val uri = FileProvider.getUriForFile(ctx, "${ctx.packageName}.fileprovider", file)
    val send = Intent(Intent.ACTION_SEND).apply {
        type = mime
        putExtra(Intent.EXTRA_STREAM, uri)
        addFlags(Intent.FLAG_GRANT_READ_URI_PERMISSION)
    }
    ctx.startActivity(Intent.createChooser(send, "Share ${file.name}").apply {
        addFlags(Intent.FLAG_ACTIVITY_NEW_TASK)
    })
}
