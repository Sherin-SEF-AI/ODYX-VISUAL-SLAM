package ai.deepmost.odyx.ui.slam

import androidx.compose.foundation.background
import androidx.compose.foundation.layout.Box
import androidx.compose.foundation.layout.Column
import androidx.compose.foundation.layout.fillMaxSize
import androidx.compose.foundation.layout.fillMaxWidth
import androidx.compose.foundation.layout.padding
import androidx.compose.foundation.Canvas
import androidx.compose.material3.MaterialTheme
import androidx.compose.material3.Text
import androidx.compose.runtime.Composable
import androidx.compose.runtime.getValue
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.geometry.Offset
import androidx.compose.ui.graphics.Color
import androidx.compose.ui.unit.dp
import androidx.lifecycle.compose.collectAsStateWithLifecycle
import ai.deepmost.odyx.core.OdyxController
import ai.deepmost.odyx.jni.TrackingState
import ai.deepmost.odyx.ui.theme.*

private const val IMG_W = 640f
private const val IMG_H = 480f

@Composable
fun SlamScreen(controller: OdyxController, modifier: Modifier = Modifier) {
    val tele by controller.telemetry.collectAsStateWithLifecycle()
    val feats by controller.features.collectAsStateWithLifecycle()
    val fps by controller.cameraFps.collectAsStateWithLifecycle()

    Box(modifier.fillMaxSize().background(OdyxBlack)) {
        CameraPreview(controller, Modifier.fillMaxSize())

        // feature-track overlay: color encodes track age; new points distinct.
        Canvas(Modifier.fillMaxSize()) {
            val f = feats ?: return@Canvas
            if (f.isEmpty()) return@Canvas
            val n = f[0].toInt()
            val sx = size.width / IMG_W
            val sy = size.height / IMG_H
            for (i in 0 until n) {
                val o = 1 + i * 4
                if (o + 3 >= f.size) break
                val u = f[o] * sx; val v = f[o + 1] * sy
                val age = f[o + 2]; val isNew = f[o + 3] > 0.5f
                val color = when {
                    isNew -> OdyxGood                                  // freshly detected
                    age > 30 -> OdyxAccent                            // long-lived track
                    age > 10 -> Color(0xFFE0B33A)
                    else -> Color(0xFF7FB0FF)
                }
                drawCircle(color, radius = if (isNew) 3f else 2.5f, center = Offset(u, v))
            }
        }

        // top HUD
        Column(
            Modifier.fillMaxWidth().align(Alignment.TopStart)
                .background(Color(0xCC0A0A0B)).padding(8.dp)
        ) {
            val t = tele
            val state = t?.tracking ?: TrackingState.UNINIT
            val stateColor = when (state) {
                TrackingState.NOMINAL -> OdyxGood
                TrackingState.INITIALIZING, TrackingState.UNSTABLE -> OdyxWarn
                TrackingState.LOST -> OdyxBad
                else -> OdyxTextDim
            }
            Text("ODYX // ${state.name}", color = stateColor,
                style = MaterialTheme.typography.titleMedium)
            Text(
                "init=${t?.init?.name ?: "--"}  feat=${t?.nTracked ?: 0}  " +
                    "opt=${"%.1f".format(t?.optimizeMs ?: 0.0)}ms  fps=${"%.0f".format(fps)}",
                color = OdyxText, style = MaterialTheme.typography.bodySmall
            )
            if (t != null && t.thermal >= 2) {
                val lbl = listOf("none", "light", "moderate", "severe", "critical").getOrElse(t.thermal) { "critical" }
                Text("🌡 THERMAL ${lbl.uppercase()} — degraded to ${t.targetFeatures} feat / stride ${t.frameStride}",
                    color = OdyxWarn, style = MaterialTheme.typography.labelSmall)
            }
            if (t?.calibDefault == true) {
                Text("⚠ UNCALIBRATED — using defaults (accuracy degraded)",
                    color = OdyxBad, style = MaterialTheme.typography.labelSmall)
            }
        }
    }
}
