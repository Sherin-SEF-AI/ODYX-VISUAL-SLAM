package ai.deepmost.odyx.ui.map

import android.opengl.GLSurfaceView
import android.view.MotionEvent
import androidx.compose.foundation.background
import androidx.compose.foundation.layout.Arrangement
import androidx.compose.foundation.layout.Box
import androidx.compose.foundation.layout.Row
import androidx.compose.foundation.layout.fillMaxSize
import androidx.compose.foundation.layout.fillMaxWidth
import androidx.compose.foundation.layout.padding
import androidx.compose.material3.FilterChip
import androidx.compose.material3.Text
import androidx.compose.runtime.Composable
import androidx.compose.runtime.LaunchedEffect
import androidx.compose.runtime.getValue
import androidx.compose.runtime.remember
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.graphics.Color
import androidx.compose.ui.unit.dp
import androidx.compose.ui.viewinterop.AndroidView
import androidx.lifecycle.compose.collectAsStateWithLifecycle
import ai.deepmost.odyx.core.OdyxController
import ai.deepmost.odyx.ui.theme.OdyxBlack
import kotlin.math.hypot

@Composable
fun MapScreen(controller: OdyxController, modifier: Modifier = Modifier) {
    val renderer = remember { MapRenderer() }
    val map by controller.map.collectAsStateWithLifecycle()
    val pose = remember { controller.engine }

    // feed latest snapshot/pose into the renderer each frame
    LaunchedEffect(map) { renderer.snapshot = map }
    LaunchedEffect(Unit) {
        while (true) {
            renderer.pose = controller.engine.pose()
            kotlinx.coroutines.delay(33)
        }
    }

    Box(modifier.fillMaxSize().background(OdyxBlack)) {
        AndroidView(
            modifier = Modifier.fillMaxSize(),
            factory = { ctx ->
                GLSurfaceView(ctx).apply {
                    setEGLContextClientVersion(2)
                    setRenderer(renderer)
                    renderMode = GLSurfaceView.RENDERMODE_CONTINUOUSLY
                    var lastX = 0f; var lastY = 0f; var lastDist = 0f; var pointers = 0
                    setOnTouchListener { _, e ->
                        when (e.actionMasked) {
                            MotionEvent.ACTION_DOWN -> { lastX = e.x; lastY = e.y; pointers = 1 }
                            MotionEvent.ACTION_POINTER_DOWN -> {
                                pointers = e.pointerCount
                                if (e.pointerCount >= 2) lastDist = hypot(e.getX(0)-e.getX(1), e.getY(0)-e.getY(1))
                            }
                            MotionEvent.ACTION_MOVE -> {
                                if (pointers >= 2 && e.pointerCount >= 2) {
                                    val d = hypot(e.getX(0)-e.getX(1), e.getY(0)-e.getY(1))
                                    if (lastDist > 0) renderer.dist = (renderer.dist * (lastDist / d)).coerceIn(1f, 80f)
                                    lastDist = d
                                } else {
                                    renderer.yaw -= (e.x - lastX) * 0.005f
                                    renderer.pitch = (renderer.pitch + (e.y - lastY) * 0.005f).coerceIn(-1.5f, 1.5f)
                                    lastX = e.x; lastY = e.y
                                }
                            }
                            MotionEvent.ACTION_POINTER_UP, MotionEvent.ACTION_UP -> pointers = 0
                        }
                        true
                    }
                }
            }
        )

        Row(
            Modifier.fillMaxWidth().align(Alignment.TopStart)
                .background(Color(0xCC0A0A0B)).padding(8.dp),
            horizontalArrangement = Arrangement.spacedBy(8.dp)
        ) {
            FilterChip(selected = renderer.showEnu, onClick = { renderer.showEnu = !renderer.showEnu },
                label = { Text(if (renderer.showEnu) "ENU" else "LOCAL") })
            FilterChip(selected = renderer.followVehicle, onClick = { renderer.followVehicle = !renderer.followVehicle },
                label = { Text("FOLLOW") })
            val m = map
            Text("kf=${(m?.trajLocal?.size ?: 0) / 3} lm=${(m?.landmarks?.size ?: 0) / 4}",
                color = Color(0xFF8A8A90), modifier = Modifier.align(Alignment.CenterVertically))
        }
    }
}
