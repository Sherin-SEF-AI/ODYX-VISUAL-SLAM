package ai.deepmost.odyx.imu

import android.content.Context
import android.hardware.Sensor
import android.hardware.SensorEvent
import android.hardware.SensorEventListener
import android.hardware.SensorManager
import android.os.Handler
import android.os.HandlerThread
import ai.deepmost.odyx.core.SensorSink
import ai.deepmost.odyx.time.TimeBase
import timber.log.Timber

/**
 * Streams TYPE_ACCELEROMETER_UNCALIBRATED + TYPE_GYROSCOPE_UNCALIBRATED at the
 * fastest steady rate on a dedicated HandlerThread, pairs each gyro sample with
 * the most recent accel sample, and pushes EVERY sample to native (preintegration
 * requires the full stream — never drop). Timestamps are unified up front.
 */
class ImuManager(
    private val context: Context,
    private val engine: SensorSink,
    private val timeBase: TimeBase,
    private val samplingPeriodUs: Int = 5_000   // ~200 Hz
) : SensorEventListener {

    private val sm = context.getSystemService(Context.SENSOR_SERVICE) as SensorManager
    private val accel: Sensor? =
        sm.getDefaultSensor(Sensor.TYPE_ACCELEROMETER_UNCALIBRATED)
            ?: sm.getDefaultSensor(Sensor.TYPE_ACCELEROMETER)
    private val gyro: Sensor? =
        sm.getDefaultSensor(Sensor.TYPE_GYROSCOPE_UNCALIBRATED)
            ?: sm.getDefaultSensor(Sensor.TYPE_GYROSCOPE)

    private var thread: HandlerThread? = null
    private var handler: Handler? = null

    // latest accelerometer reading (uncalibrated: indices 0..2 are raw)
    @Volatile private var ax = 0f
    @Volatile private var ay = 0f
    @Volatile private var az = 0f
    @Volatile private var haveAccel = false

    fun start() {
        if (accel == null || gyro == null) {
            Timber.tag("ImuManager").e("missing IMU sensors (accel=%s gyro=%s)", accel, gyro)
            return
        }
        val t = HandlerThread("odyx-imu").also { it.start() }
        thread = t
        handler = Handler(t.looper)
        sm.registerListener(this, accel, samplingPeriodUs, handler)
        sm.registerListener(this, gyro, samplingPeriodUs, handler)
        Timber.tag("ImuManager").i("started accel=%s gyro=%s @ %dus", accel.name, gyro.name, samplingPeriodUs)
    }

    fun stop() {
        sm.unregisterListener(this)
        thread?.quitSafely()
        thread = null; handler = null; haveAccel = false
    }

    override fun onSensorChanged(e: SensorEvent) {
        when (e.sensor.type) {
            Sensor.TYPE_ACCELEROMETER_UNCALIBRATED, Sensor.TYPE_ACCELEROMETER -> {
                ax = e.values[0]; ay = e.values[1]; az = e.values[2]; haveAccel = true
            }
            Sensor.TYPE_GYROSCOPE_UNCALIBRATED, Sensor.TYPE_GYROSCOPE -> {
                if (!haveAccel) return
                val t = timeBase.imuToUnified(e.timestamp)
                engine.pushImu(t, ax, ay, az, e.values[0], e.values[1], e.values[2])
            }
        }
    }

    override fun onAccuracyChanged(sensor: Sensor?, accuracy: Int) {}
}
