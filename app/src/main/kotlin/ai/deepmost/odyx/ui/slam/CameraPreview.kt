package ai.deepmost.odyx.ui.slam

import android.view.SurfaceHolder
import android.view.SurfaceView
import androidx.compose.runtime.Composable
import androidx.compose.ui.Modifier
import androidx.compose.ui.viewinterop.AndroidView
import ai.deepmost.odyx.core.OdyxController

/**
 * SurfaceView preview. The Camera2 session renders directly into this surface;
 * the feature overlay is drawn on top by [SlamScreen] with a Compose Canvas.
 */
@Composable
fun CameraPreview(controller: OdyxController, modifier: Modifier = Modifier) {
    AndroidView(
        modifier = modifier,
        factory = { ctx ->
            SurfaceView(ctx).apply {
                holder.addCallback(object : SurfaceHolder.Callback {
                    override fun surfaceCreated(h: SurfaceHolder) {
                        controller.setPreviewSurface(h.surface)
                    }
                    override fun surfaceChanged(h: SurfaceHolder, f: Int, w: Int, ht: Int) {
                        controller.setPreviewSurface(h.surface)
                    }
                    override fun surfaceDestroyed(h: SurfaceHolder) {
                        controller.setPreviewSurface(null)
                    }
                })
            }
        }
    )
}
