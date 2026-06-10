package ai.deepmost.odyx.ui.calib

import androidx.compose.foundation.Canvas
import androidx.compose.foundation.background
import androidx.compose.foundation.layout.*
import androidx.compose.material3.*
import androidx.compose.runtime.*
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.geometry.Offset
import androidx.compose.ui.graphics.Color
import androidx.compose.ui.unit.dp
import ai.deepmost.odyx.config.CalibLoader
import ai.deepmost.odyx.core.OdyxController
import ai.deepmost.odyx.ui.slam.CameraPreview
import ai.deepmost.odyx.ui.theme.*
import kotlinx.coroutines.delay

private const val IMG_W = 640f
private const val IMG_H = 480f

/**
 * In-app checkerboard intrinsics calibration. Reuses the running camera; the
 * native CameraCalibrator detects the board each frame (overlay), accumulates
 * user-captured views, and runs cv::calibrateCamera. Writes camera_intrinsics.yaml.
 */
@Composable
fun CheckerboardCalibScreen(controller: OdyxController, onDone: () -> Unit, modifier: Modifier = Modifier) {
    var cols by remember { mutableStateOf(9) }
    var rows by remember { mutableStateOf(6) }
    var squareMm by remember { mutableStateOf(25.0) }
    var corners by remember { mutableStateOf<FloatArray?>(null) }
    var views by remember { mutableStateOf(0) }
    var result by remember { mutableStateOf<String?>(null) }

    // activate calib mode on enter; deactivate on leave
    DisposableEffect(cols, rows, squareMm) {
        controller.engine.setCalibActive(true, cols, rows, squareMm / 1000.0)
        onDispose { controller.engine.setCalibActive(false) }
    }
    LaunchedEffect(Unit) {
        while (true) {
            corners = controller.engine.calibCorners()
            views = controller.engine.calibViewCount()
            delay(80)
        }
    }

    Box(modifier.fillMaxSize().background(OdyxBlack)) {
        CameraPreview(controller, Modifier.fillMaxSize())
        Canvas(Modifier.fillMaxSize()) {
            val c = corners ?: return@Canvas
            if (c.isEmpty() || c[0] <= 0f) return@Canvas
            val n = c[0].toInt(); val sx = size.width / IMG_W; val sy = size.height / IMG_H
            for (i in 0 until n) {
                val o = 1 + i * 2; if (o + 1 >= c.size) break
                drawCircle(OdyxGood, 4f, Offset(c[o] * sx, c[o + 1] * sy))
            }
        }

        Column(
            Modifier.fillMaxWidth().align(Alignment.TopStart).background(Color(0xCC0A0A0B)).padding(10.dp)
        ) {
            Text("CHECKERBOARD CALIBRATION", color = OdyxAccent, style = MaterialTheme.typography.titleMedium)
            val detected = (corners?.getOrNull(0) ?: 0f) > 0f
            Text("board ${cols}x${rows} @ ${squareMm.toInt()}mm   " +
                "board ${if (detected) "DETECTED" else "not found"}   views=$views",
                color = if (detected) OdyxGood else OdyxTextDim, style = MaterialTheme.typography.bodySmall)
            Text("Show a ${cols}x${rows} INNER-CORNER checkerboard from varied angles/distances; capture 8+ views.",
                color = OdyxTextDim, style = MaterialTheme.typography.labelSmall)
            result?.let { Text(it, color = OdyxText, style = MaterialTheme.typography.bodySmall) }
        }

        Row(
            Modifier.fillMaxWidth().align(Alignment.BottomCenter).background(Color(0xCC0A0A0B)).padding(10.dp),
            horizontalArrangement = Arrangement.spacedBy(8.dp), verticalAlignment = Alignment.CenterVertically
        ) {
            Button(onClick = { views = controller.engine.captureCalibView() }) { Text("CAPTURE ($views)") }
            Button(onClick = { controller.engine.resetCalibration(); views = 0; result = null }) { Text("RESET") }
            Button(
                enabled = views >= 8,
                onClick = {
                    val r = controller.engine.runCalibration()
                    if (r[0] != 0.0) {
                        CalibLoader.writeCameraIntrinsics(
                            controller.appContext(), r[12].toInt(), r[13].toInt(),
                            r[1], r[2], r[3], r[4],
                            doubleArrayOf(r[5], r[6], r[7], r[8], r[9]), r[10]
                        )
                        controller.applySettings(controller.currentSettings())  // reload calib live
                        result = "OK  fx=${"%.1f".format(r[1])} cx=${"%.1f".format(r[3])} rms=${"%.3f".format(r[10])}px — saved"
                    } else result = "calibration failed (rms too high / too few views)"
                }
            ) { Text("CALIBRATE") }
            OutlinedButton(onClick = { controller.engine.setCalibActive(false); onDone() }) { Text("DONE") }
        }
    }
}
