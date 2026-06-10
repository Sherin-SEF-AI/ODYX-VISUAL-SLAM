package ai.deepmost.odyx.camera

import android.Manifest
import android.content.Context
import android.content.pm.PackageManager
import android.graphics.ImageFormat
import android.hardware.camera2.CameraCaptureSession
import android.hardware.camera2.CameraCharacteristics
import android.hardware.camera2.CameraDevice
import android.hardware.camera2.CameraManager
import android.hardware.camera2.CaptureRequest
import android.hardware.camera2.CaptureResult
import android.hardware.camera2.TotalCaptureResult
import android.media.Image
import android.media.ImageReader
import android.os.Handler
import android.os.HandlerThread
import android.util.Size
import android.view.Surface
import androidx.core.content.ContextCompat
import ai.deepmost.odyx.core.SensorSink
import ai.deepmost.odyx.time.TimeBase
import timber.log.Timber

/**
 * Opens the back camera with two outputs: a YUV_420_888 [ImageReader] for the
 * estimator and an optional GL preview [Surface]. Locks AE/AF to avoid focus
 * hunting, records exposure + SENSOR_ROLLING_SHUTTER_SKEW + intrinsics, and
 * pushes the Y (luma) plane + unified timestamp to native.
 *
 * The Y plane of YUV_420_888 IS the grayscale image the front-end needs, so the
 * JNI payload is just the luma plane (with rowStride) — no colour conversion.
 */
class Camera2Controller(
    private val context: Context,
    private val engine: SensorSink,
    private val targetSize: Size = Size(640, 480),
    private val onCalibration: (CameraCalibration) -> Unit = {},
    private val onFrameStats: (fps: Double) -> Unit = {}
) {
    data class CameraCalibration(
        val fx: Double, val fy: Double, val cx: Double, val cy: Double,
        val width: Int, val height: Int,
        val rollingShutterSkewNs: Long,
        val intrinsicsReported: Boolean,
        val timeBase: TimeBase
    )

    private val cm = context.getSystemService(Context.CAMERA_SERVICE) as CameraManager
    private var device: CameraDevice? = null
    private var session: CameraCaptureSession? = null
    @Volatile private var sessionGen = 0
    private var reader: ImageReader? = null
    private var thread: HandlerThread? = null
    private var handler: Handler? = null

    private var timeBase: TimeBase? = null
    private var skewNs: Long = 0
    private var fx = 0.0; private var fy = 0.0; private var cx = 0.0; private var cy = 0.0
    private var intrinsicsReported = false
    @Volatile private var lastExposureS = 0.0

    private var previewSurface: Surface? = null
    private var lastFrameNs = 0L
    private var emaFps = 0.0

    fun setPreviewSurface(s: Surface?) {
        // Ignore redundant calls (SurfaceHolder fires created+changed with the
        // same Surface) so we don't needlessly tear down / rebuild the session.
        if (s === previewSurface) return
        previewSurface = s
        // If the camera is already open, reconfigure the session so the preview
        // target attaches (or detaches) without dropping the estimator stream.
        if (device != null) handler?.post {
            try { session?.close() } catch (_: Exception) {}
            session = null
            createSession()
        }
    }

    fun start() {
        val cameraId = pickBackCamera() ?: run {
            Timber.tag("Camera2").e("no back camera"); return
        }
        val chars = cm.getCameraCharacteristics(cameraId)
        configureCalibration(chars)

        val t = HandlerThread("odyx-cam").also { it.start() }
        thread = t; handler = Handler(t.looper)

        reader = ImageReader.newInstance(
            targetSize.width, targetSize.height, ImageFormat.YUV_420_888, 3
        ).apply { setOnImageAvailableListener(::onImage, handler) }

        if (ContextCompat.checkSelfPermission(context, Manifest.permission.CAMERA)
            != PackageManager.PERMISSION_GRANTED) {
            Timber.tag("Camera2").e("camera permission not granted"); return
        }
        cm.openCamera(cameraId, stateCallback, handler)
    }

    fun stop() {
        try { session?.close() } catch (_: Exception) {}
        try { device?.close() } catch (_: Exception) {}
        reader?.close()
        thread?.quitSafely()
        session = null; device = null; reader = null; thread = null; handler = null
    }

    private fun configureCalibration(chars: CameraCharacteristics) {
        val tsSource = chars.get(CameraCharacteristics.SENSOR_INFO_TIMESTAMP_SOURCE)
            ?: CameraCharacteristics.SENSOR_INFO_TIMESTAMP_SOURCE_UNKNOWN
        timeBase = TimeBase(tsSource)
        // SENSOR_ROLLING_SHUTTER_SKEW is a per-frame CaptureResult key (not a
        // characteristic); it is read in the capture callback below.
        skewNs = 0L

        // LENS_INTRINSIC_CALIBRATION = [fx, fy, cx, cy, skew] for the active
        // array; scale to our processing resolution if reported.
        val intr = chars.get(CameraCharacteristics.LENS_INTRINSIC_CALIBRATION)
        val active = chars.get(CameraCharacteristics.SENSOR_INFO_ACTIVE_ARRAY_SIZE)
        if (intr != null && active != null && intr.size >= 4) {
            val sx = targetSize.width.toDouble() / active.width()
            val sy = targetSize.height.toDouble() / active.height()
            fx = intr[0] * sx; fy = intr[1] * sy; cx = intr[2] * sx; cy = intr[3] * sy
            intrinsicsReported = true
        } else {
            // sensible pinhole default (~60° HFOV) — flagged as uncalibrated.
            fx = targetSize.width * 0.9; fy = fx
            cx = targetSize.width / 2.0; cy = targetSize.height / 2.0
            intrinsicsReported = false
        }
        onCalibration(
            CameraCalibration(fx, fy, cx, cy, targetSize.width, targetSize.height,
                skewNs, intrinsicsReported, timeBase!!)
        )
        Timber.tag("Camera2").i("tsSource=%d skew=%dns intr=%s fx=%.1f cx=%.1f",
            tsSource, skewNs, intrinsicsReported, fx, cx)
    }

    private val stateCallback = object : CameraDevice.StateCallback() {
        override fun onOpened(camera: CameraDevice) { device = camera; createSession() }
        override fun onDisconnected(camera: CameraDevice) { camera.close(); device = null }
        override fun onError(camera: CameraDevice, error: Int) {
            Timber.tag("Camera2").e("camera error %d", error); camera.close(); device = null
        }
    }

    private fun createSession() {
        val dev = device ?: return
        val targets = buildList {
            reader?.surface?.let { add(it) }
            previewSurface?.let { add(it) }
        }
        if (targets.isEmpty()) return
        // Generation guard: setPreviewSurface() closes + recreates the session, so
        // a stale onConfigured for a now-closed session must NOT touch it.
        val gen = ++sessionGen
        @Suppress("DEPRECATION")
        dev.createCaptureSession(targets, object : CameraCaptureSession.StateCallback() {
            override fun onConfigured(s: CameraCaptureSession) {
                if (gen != sessionGen) { try { s.close() } catch (_: Exception) {}; return }
                session = s
                val req = dev.createCaptureRequest(CameraDevice.TEMPLATE_PREVIEW).apply {
                    reader?.surface?.let { addTarget(it) }
                    previewSurface?.let { addTarget(it) }
                    // Steady 30 fps; lock AF; keep AE on but no flicker hunting.
                    set(CaptureRequest.CONTROL_AF_MODE, CaptureRequest.CONTROL_AF_MODE_OFF)
                    set(CaptureRequest.LENS_FOCUS_DISTANCE, 0f)   // focus at infinity
                    set(CaptureRequest.CONTROL_AE_MODE, CaptureRequest.CONTROL_AE_MODE_ON)
                    set(CaptureRequest.CONTROL_AE_TARGET_FPS_RANGE, android.util.Range(30, 30))
                    set(CaptureRequest.CONTROL_VIDEO_STABILIZATION_MODE,
                        CaptureRequest.CONTROL_VIDEO_STABILIZATION_MODE_OFF)
                }
                try {
                    s.setRepeatingRequest(req.build(), captureCallback, handler)
                } catch (e: Exception) {
                    Timber.tag("Camera2").w(e, "setRepeatingRequest on stale/closed session")
                }
            }
            override fun onConfigureFailed(s: CameraCaptureSession) {
                Timber.tag("Camera2").e("session configure failed")
            }
        }, handler)
    }

    private val captureCallback = object : CameraCaptureSession.CaptureCallback() {
        override fun onCaptureCompleted(
            session: CameraCaptureSession, request: CaptureRequest, result: TotalCaptureResult
        ) {
            result.get(CaptureResult.SENSOR_EXPOSURE_TIME)?.let { lastExposureS = it * 1e-9 }
            result.get(CaptureResult.SENSOR_ROLLING_SHUTTER_SKEW)?.let { skewNs = it }
        }
    }

    private fun onImage(r: ImageReader) {
        val img: Image = r.acquireLatestImage() ?: return
        try {
            val tb = timeBase ?: return
            val tNs = tb.cameraToUnified(img.timestamp)
            val y = img.planes[0]
            val rowStride = y.rowStride
            val w = img.width; val h = img.height
            val buf = y.buffer
            val bytes = ByteArray(rowStride * h)
            buf.get(bytes, 0, minOf(bytes.size, buf.remaining()))
            engine.pushFrame(bytes, w, h, rowStride, tNs, skewNs, lastExposureS, fx, fy, cx, cy)

            // fps stat
            if (lastFrameNs != 0L) {
                val dt = (tNs - lastFrameNs) * 1e-9
                if (dt > 0) { val f = 1.0 / dt; emaFps = if (emaFps == 0.0) f else 0.9 * emaFps + 0.1 * f }
                onFrameStats(emaFps)
            }
            lastFrameNs = tNs
        } finally {
            img.close()   // ALWAYS close acquired images
        }
    }

    private fun pickBackCamera(): String? =
        cm.cameraIdList.firstOrNull {
            cm.getCameraCharacteristics(it).get(CameraCharacteristics.LENS_FACING) ==
                CameraCharacteristics.LENS_FACING_BACK
        }
}
