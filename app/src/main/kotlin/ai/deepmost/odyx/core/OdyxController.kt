package ai.deepmost.odyx.core

import android.content.Context
import android.os.Build
import android.os.PowerManager
import android.util.Size
import android.view.Surface
import ai.deepmost.odyx.camera.Camera2Controller
import ai.deepmost.odyx.config.CalibLoader
import ai.deepmost.odyx.config.CameraIntrinsics
import ai.deepmost.odyx.config.OdyxConfig
import ai.deepmost.odyx.config.OdyxSettings
import ai.deepmost.odyx.gnss.GnssManager
import ai.deepmost.odyx.imu.ImuManager
import ai.deepmost.odyx.jni.MapSnapshotK
import ai.deepmost.odyx.jni.TelemetrySnapshot
import ai.deepmost.odyx.persist.MapPersistence
import ai.deepmost.odyx.persist.TrajectoryLogger
import ai.deepmost.odyx.pose.OdyxPoseProvider
import ai.deepmost.odyx.session.SessionRecorder
import ai.deepmost.odyx.session.SessionReplayer
import ai.deepmost.odyx.session.SessionStore
import ai.deepmost.odyx.time.TimeBase
import kotlinx.coroutines.CoroutineScope
import kotlinx.coroutines.Dispatchers
import kotlinx.coroutines.Job
import kotlinx.coroutines.delay
import kotlinx.coroutines.flow.MutableStateFlow
import kotlinx.coroutines.flow.StateFlow
import kotlinx.coroutines.isActive
import kotlinx.coroutines.launch
import timber.log.Timber
import java.io.File

/**
 * Owns the full capture→estimate pipeline and the native engine lifecycle.
 * Both [ui.MainActivity] and [service.CaptureService] drive a single instance.
 */
class OdyxController(private val context: Context) {

    val engine = OdyxEngine()
    val poseProvider = OdyxPoseProvider(engine)
    val sessionStore = SessionStore(context)

    // Sensor managers push into this forwarder; its target is `engine` normally,
    // or a CompositeSink(engine, recorder) while recording — recording is a tap
    // that never perturbs the live estimate.
    @Volatile private var activeSink: SensorSink = engine
    private val forwardingSink = object : SensorSink {
        override fun pushFrame(y: ByteArray, w: Int, h: Int, rowStride: Int, tNs: Long, rsSkewNs: Long, exposureS: Double, fx: Double, fy: Double, cx: Double, cy: Double) =
            activeSink.pushFrame(y, w, h, rowStride, tNs, rsSkewNs, exposureS, fx, fy, cx, cy)
        override fun pushImu(tNs: Long, ax: Float, ay: Float, az: Float, gx: Float, gy: Float, gz: Float) =
            activeSink.pushImu(tNs, ax, ay, az, gx, gy, gz)
        override fun pushGnssFix(tNs: Long, lat: Double, lon: Double, alt: Double, hAcc: Double, vAcc: Double, nSats: Int, hasAlt: Boolean) =
            activeSink.pushGnssFix(tNs, lat, lon, alt, hAcc, vAcc, nSats, hasAlt)
        override fun pushGnssRaw(clkTNs: Long, clkTimeNs: Long, fullBiasNs: Long, biasNs: Double, driftNsps: Double, leapSec: Int, fullBiasValid: Boolean, svid: IntArray, constellation: IntArray, cn0: DoubleArray, state: IntArray, rxSvTimeNs: LongArray, rxSvTimeUncNs: LongArray, prRate: DoubleArray, prRateUnc: DoubleArray, carrierFreq: DoubleArray, timeOffsetNs: LongArray) =
            activeSink.pushGnssRaw(clkTNs, clkTimeNs, fullBiasNs, biasNs, driftNsps, leapSec, fullBiasValid, svid, constellation, cn0, state, rxSvTimeNs, rxSvTimeUncNs, prRate, prRateUnc, carrierFreq, timeOffsetNs)
        override fun pushNav(constellation: Int, svid: Int, type: Int, subId: Int, msgId: Int, data: ByteArray) =
            activeSink.pushNav(constellation, svid, type, subId, msgId, data)
    }
    private var recorder: SessionRecorder? = null
    private val _recording = MutableStateFlow<String?>(null)
    val recording: StateFlow<String?> = _recording

    private val replayer by lazy { SessionReplayer(sessionStore, engine) }
    private val _replaying = MutableStateFlow<String?>(null)
    val replaying: StateFlow<String?> = _replaying

    /**
     * Replay a recorded session deterministically through the estimator with the
     * LIVE sensor stream isolated (routed to a no-op sink) so only the recorded
     * data reaches the VIO. Used for tuning + regression.
     */
    suspend fun replaySession(name: String, onProgress: (Double) -> Unit = {}) {
        if (_replaying.value != null) return
        _replaying.value = name
        val prevSink = activeSink
        activeSink = NoOpSink          // drop live camera/IMU/GNSS during replay
        try {
            kotlinx.coroutines.withContext(Dispatchers.IO) { replayer.replay(name, 0.0, onProgress) }
        } finally {
            activeSink = prevSink
            _replaying.value = null
        }
    }

    @Volatile private var trajLogger: TrajectoryLogger? = null
    private val _loggingTraj = MutableStateFlow(false)
    val loggingTraj: StateFlow<Boolean> = _loggingTraj

    /** Start logging the emitted pose stream to TUM files under outDir. */
    fun startTrajectoryLog(tag: String) {
        if (trajLogger != null) return
        trajLogger = TrajectoryLogger(File(context.filesDir, "trajectories"), tag)
        _loggingTraj.value = true
    }
    fun stopTrajectoryLog(): List<File> {
        val l = trajLogger ?: return emptyList()
        l.close(); trajLogger = null; _loggingTraj.value = false
        return l.files()
    }

    /** Persist the current sparse map snapshot for offline inspection/export. */
    fun saveMap(tag: String): File? {
        val snap = engine.map() ?: return null
        val f = File(context.filesDir, "maps/map_$tag.odyxmap")
        MapPersistence.save(f, snap)
        return f
    }

    fun startRecording(name: String) {
        if (recorder != null) return
        val r = SessionRecorder(sessionStore, name)
        recorder = r
        activeSink = CompositeSink(listOf(engine, r))
        _recording.value = name
    }
    fun stopRecording() {
        activeSink = engine
        recorder?.close(); recorder = null
        _recording.value = null
    }

    private val scope = CoroutineScope(Dispatchers.Default)
    private var camera: Camera2Controller? = null
    private var imu: ImuManager? = null
    private var gnss: GnssManager? = null
    private var timeBase: TimeBase? = null
    private var pollJob: Job? = null
    private var running = false

    private val _telemetry = MutableStateFlow<TelemetrySnapshot?>(null)
    val telemetry: StateFlow<TelemetrySnapshot?> = _telemetry
    private val _map = MutableStateFlow<MapSnapshotK?>(null)
    val map: StateFlow<MapSnapshotK?> = _map
    private val _cameraFps = MutableStateFlow(0.0)
    val cameraFps: StateFlow<Double> = _cameraFps
    private val _features = MutableStateFlow<FloatArray?>(null)
    val features: StateFlow<FloatArray?> = _features

    private var settings: OdyxSettings = OdyxSettings()
    private var previewSurface: Surface? = null
    @Volatile private var cameraReported: ai.deepmost.odyx.config.CameraIntrinsics? = null

    private val pm by lazy { context.getSystemService(Context.POWER_SERVICE) as PowerManager }
    // Lazily instantiated so the API-29 listener class is never loaded on API 26–28.
    private val thermalListener by lazy {
        PowerManager.OnThermalStatusChangedListener { status ->
            engine.setThermal(status)   // PowerManager.THERMAL_STATUS_* maps to levels
            Timber.tag("Odyx").i("thermal status=%d", status)
        }
    }

    fun setPreviewSurface(s: Surface?) {
        previewSurface = s
        camera?.setPreviewSurface(s)
    }

    fun applySettings(s: OdyxSettings) {
        val prev = settings
        settings = s
        engine.loadConfig(CalibLoader.buildConfig(context, s, cameraReported))
        // Sensor-manager-level settings need the manager re-created (estimator
        // params above apply live; these don't).
        val tb = timeBase
        if (tb != null && running) {
            if (s.imuSamplingUs != prev.imuSamplingUs) {
                imu?.stop()
                imu = ImuManager(context, forwardingSink, tb, s.imuSamplingUs).also { it.start() }
            }
            if (s.useRawGnss != prev.useRawGnss) {
                gnss?.stop()
                gnss = GnssManager(context, forwardingSink, tb).also { it.start(s.useRawGnss) }
            }
        }
    }

    fun start(initialSettings: OdyxSettings) {
        if (running) return
        running = true
        settings = initialSettings
        engine.create()
        engine.loadConfig(CalibLoader.buildConfig(context, settings))
        loadVocabulary()

        // Camera owns the authoritative TimeBase (from SENSOR_INFO_TIMESTAMP_SOURCE).
        camera = Camera2Controller(
            context, forwardingSink, Size(640, 480),
            onCalibration = { c ->
                timeBase = c.timeBase
                imu = ImuManager(context, forwardingSink, c.timeBase, settings.imuSamplingUs).also { it.start() }
                gnss = GnssManager(context, forwardingSink, c.timeBase).also { it.start(settings.useRawGnss) }
                // If the user hasn't supplied a calibrated camera_intrinsics.yaml,
                // use the camera-reported intrinsics (LENS_INTRINSIC_CALIBRATION or
                // the FOV-based default) for the estimator geometry — far better
                // than the generic YAML default. (UNCALIBRATED warning stays until
                // a real checkerboard calibration.)
                cameraReported = CameraIntrinsics(
                    width = c.width, height = c.height,
                    fx = c.fx, fy = c.fy, cx = c.cx, cy = c.cy, valid = false)
                engine.loadConfig(CalibLoader.buildConfig(context, settings, cameraReported))
            },
            onFrameStats = { fps -> _cameraFps.value = fps }
        )
        camera?.setPreviewSurface(previewSurface)
        camera?.start()

        engine.setGnssMode(settings.gnssMode)
        engine.start()
        poseProvider.start()

        if (Build.VERSION.SDK_INT >= 29) {
            engine.setThermal(pm.currentThermalStatus)
            pm.addThermalStatusListener(thermalListener)
        }
        startPolling()
        Timber.tag("Odyx").i("controller started")
    }

    private fun startPolling() {
        pollJob = scope.launch {
            while (isActive) {
                _telemetry.value = engine.telemetry()
                _map.value = engine.map()
                _features.value = engine.features()
                trajLogger?.let { l -> engine.pose()?.let { p -> runCatching { l.log(p) } } }
                delay(66)    // ~15 Hz telemetry/map/overlay refresh
            }
        }
    }

    private fun loadVocabulary() {
        // Copy the bundled ORB vocabulary out of assets to a real path DBoW3 can open.
        val out = File(context.filesDir, "orbvoc.dbow3")
        try {
            if (!out.exists()) {
                context.assets.open("orbvoc.dbow3").use { input ->
                    out.outputStream().use { input.copyTo(it) }
                }
            }
            val ok = engine.loadVocabulary(out.absolutePath)
            Timber.tag("Odyx").i("vocabulary loaded=%s", ok)
        } catch (e: Exception) {
            Timber.tag("Odyx").w(e, "no ORB vocabulary asset; loop closure/reloc disabled")
        }
    }

    /** Application context, for utilities (e.g. the Allan logger). */
    fun appContext(): Context = context

    /** The settings currently applied (for live re-apply, e.g. after calibration). */
    fun currentSettings(): OdyxSettings = settings

    fun reset() = engine.reset()

    fun stop() {
        if (!running) return
        running = false
        pollJob?.cancel()
        if (Build.VERSION.SDK_INT >= 29) pm.removeThermalStatusListener(thermalListener)
        stopTrajectoryLog()
        runCatching { saveMap("last") }   // persist the map for next launch / export
        poseProvider.stop()
        camera?.stop(); imu?.stop(); gnss?.stop()
        engine.stop()
        Timber.tag("Odyx").i("controller stopped")
    }

    fun destroy() { stop(); engine.destroy() }
}
