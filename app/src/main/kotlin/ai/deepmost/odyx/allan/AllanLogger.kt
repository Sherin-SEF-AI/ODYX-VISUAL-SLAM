package ai.deepmost.odyx.allan

import android.content.Context
import android.hardware.Sensor
import android.hardware.SensorEvent
import android.hardware.SensorEventListener
import android.hardware.SensorManager
import android.os.Handler
import android.os.HandlerThread
import timber.log.Timber
import java.io.File
import kotlin.math.sqrt

/**
 * In-app Allan-variance calibrator. Record N minutes of STATIC IMU, compute the
 * white-noise density and bias random walk per axis, and write imu_noise.yaml so
 * a new phone can be calibrated without external tooling (Kalibr/allan_variance_ros).
 *
 * Method: overlapping Allan deviation σ(τ). White-noise density N = σ at τ=1 s
 * (slope −1/2 region). Bias random walk K from the +1/2 slope region using
 * σ(τ) = K·√(τ/3) ⇒ K = σ(τ)·√(3/τ). Reports the average across the 3 axes.
 */
class AllanLogger(private val context: Context) : SensorEventListener {

    interface Listener {
        fun onProgress(seconds: Double, total: Double)
        fun onDone(result: Result)
        fun onError(msg: String)
    }

    data class Result(val accN: Double, val gyrN: Double, val accW: Double, val gyrW: Double, val sampleHz: Double)

    private val sm = context.getSystemService(Context.SENSOR_SERVICE) as SensorManager
    private val accel = sm.getDefaultSensor(Sensor.TYPE_ACCELEROMETER_UNCALIBRATED)
        ?: sm.getDefaultSensor(Sensor.TYPE_ACCELEROMETER)
    private val gyro = sm.getDefaultSensor(Sensor.TYPE_GYROSCOPE_UNCALIBRATED)
        ?: sm.getDefaultSensor(Sensor.TYPE_GYROSCOPE)

    private var thread: HandlerThread? = null
    private val acc = Array(3) { ArrayList<Double>() }
    private val gyr = Array(3) { ArrayList<Double>() }
    private var t0 = 0L; private var tLast = 0L
    private var listener: Listener? = null
    private var durationS = 300.0
    @Volatile private var running = false

    fun start(durationMinutes: Double, listener: Listener) {
        if (accel == null || gyro == null) { listener.onError("missing IMU sensors"); return }
        this.listener = listener
        durationS = durationMinutes * 60.0
        acc.forEach { it.clear() }; gyr.forEach { it.clear() }
        t0 = 0; running = true
        val t = HandlerThread("odyx-allan").also { it.start() }
        thread = t
        val h = Handler(t.looper)
        sm.registerListener(this, accel, SensorManager.SENSOR_DELAY_FASTEST, h)
        sm.registerListener(this, gyro, SensorManager.SENSOR_DELAY_FASTEST, h)
    }

    fun cancel() { running = false; finish(false) }

    override fun onSensorChanged(e: SensorEvent) {
        if (!running) return
        if (t0 == 0L) t0 = e.timestamp
        tLast = e.timestamp
        when (e.sensor.type) {
            Sensor.TYPE_ACCELEROMETER_UNCALIBRATED, Sensor.TYPE_ACCELEROMETER ->
                for (i in 0..2) acc[i].add(e.values[i].toDouble())
            Sensor.TYPE_GYROSCOPE_UNCALIBRATED, Sensor.TYPE_GYROSCOPE ->
                for (i in 0..2) gyr[i].add(e.values[i].toDouble())
        }
        val elapsed = (tLast - t0) * 1e-9
        listener?.onProgress(elapsed, durationS)
        if (elapsed >= durationS) { running = false; finish(true) }
    }

    override fun onAccuracyChanged(sensor: Sensor?, accuracy: Int) {}

    private fun finish(ok: Boolean) {
        sm.unregisterListener(this)
        thread?.quitSafely(); thread = null
        if (!ok) return
        val dt = (tLast - t0) * 1e-9
        val fsAcc = acc[0].size / dt
        val fsGyr = gyr[0].size / dt
        val fs = (fsAcc + fsGyr) / 2.0

        val accN = (0..2).map { whiteNoise(acc[it], fsAcc) }.average()
        val gyrN = (0..2).map { whiteNoise(gyr[it], fsGyr) }.average()
        val accW = (0..2).map { randomWalk(acc[it], fsAcc) }.average()
        val gyrW = (0..2).map { randomWalk(gyr[it], fsGyr) }.average()

        val r = Result(accN, gyrN, accW, gyrW, fs)
        writeYaml(r)
        Timber.tag("Allan").i("accN=%.4g gyrN=%.4g accW=%.4g gyrW=%.4g fs=%.1f", accN, gyrN, accW, gyrW, fs)
        listener?.onDone(r)
    }

    // overlapping Allan deviation at averaging time tau (in samples m)
    private fun allanDev(x: List<Double>, m: Int): Double {
        val n = x.size
        if (m < 1 || 2 * m >= n) return Double.NaN
        // cumulative sums of cluster averages
        var sum = 0.0; var cnt = 0
        var i = 0
        while (i + 2 * m < n) {
            var a1 = 0.0; var a2 = 0.0
            for (k in 0 until m) { a1 += x[i + k]; a2 += x[i + m + k] }
            a1 /= m; a2 /= m
            val d = a2 - a1
            sum += d * d; cnt++
            i += m   // non-overlapping clusters (stable + cheap on phone data)
        }
        return if (cnt > 0) sqrt(sum / (2.0 * cnt)) else Double.NaN
    }

    /** White-noise density N = σ(τ=1s). */
    private fun whiteNoise(x: List<Double>, fs: Double): Double {
        if (fs <= 0 || x.size < 100) return 0.0
        val m = fs.toInt().coerceAtLeast(1)        // τ = 1 s
        val s = allanDev(x, m)
        return if (s.isFinite()) s else 0.0
    }

    /** Bias random walk K from the +1/2 slope: σ(τ)=K√(τ/3) ⇒ K=σ·√(3/τ). */
    private fun randomWalk(x: List<Double>, fs: Double): Double {
        if (fs <= 0 || x.size < 1000) return 0.0
        val tauS = 20.0                            // sample a long τ in the RW region
        val m = (tauS * fs).toInt().coerceAtLeast(1)
        val s = allanDev(x, m)
        return if (s.isFinite()) s * sqrt(3.0 / tauS) else 0.0
    }

    private fun writeYaml(r: Result) {
        val dir = File(context.filesDir, "calib").apply { mkdirs() }
        File(dir, "imu_noise.yaml").writeText(
            """
            # Generated by ODYX in-app Allan-variance logger.
            accel_noise_density: ${"%.6e".format(r.accN)}
            gyro_noise_density: ${"%.6e".format(r.gyrN)}
            accel_random_walk: ${"%.6e".format(r.accW)}
            gyro_random_walk: ${"%.6e".format(r.gyrW)}
            gravity: 9.81
            sample_rate_hz: ${"%.1f".format(r.sampleHz)}
            """.trimIndent()
        )
    }
}
