package ai.deepmost.odyx.gnss

import android.Manifest
import android.content.Context
import android.content.pm.PackageManager
import android.location.GnssMeasurementsEvent
import android.location.GnssNavigationMessage
import android.location.LocationManager
import android.os.Build
import android.os.Handler
import android.os.HandlerThread
import androidx.core.content.ContextCompat
import ai.deepmost.odyx.core.SensorSink
import ai.deepmost.odyx.time.TimeBase
import com.google.android.gms.location.LocationServices
import com.google.android.gms.location.LocationRequest
import com.google.android.gms.location.Priority
import com.google.android.gms.location.LocationCallback
import com.google.android.gms.location.LocationResult
import timber.log.Timber

/**
 * GNSS ingestion: FusedLocationProvider fixes (loose) + LocationManager raw
 * GnssMeasurement/GnssNavigationMessage callbacks (tight). All timestamps are
 * reduced to the unified base. Raw callbacks require API 27+ for reliability.
 */
class GnssManager(
    private val context: Context,
    private val engine: SensorSink,
    private val timeBase: TimeBase
) {
    private val lm = context.getSystemService(Context.LOCATION_SERVICE) as LocationManager
    private val fused = LocationServices.getFusedLocationProviderClient(context)
    private var thread: HandlerThread? = null
    private var handler: Handler? = null
    @Volatile var rawSupported = false; private set

    private val locationCallback = object : LocationCallback() {
        override fun onLocationResult(result: LocationResult) {
            val loc = result.lastLocation ?: return
            val tNs = if (Build.VERSION.SDK_INT >= 29) timeBase.gnssToUnified(loc.elapsedRealtimeNanos)
                      else timeBase.gnssToUnified(null)
            engine.pushGnssFix(
                tNs, loc.latitude, loc.longitude,
                if (loc.hasAltitude()) loc.altitude else 0.0,
                if (loc.hasAccuracy()) loc.accuracy.toDouble() else 30.0,
                if (Build.VERSION.SDK_INT >= 26 && loc.hasVerticalAccuracy())
                    loc.verticalAccuracyMeters.toDouble() else 30.0,
                0, loc.hasAltitude()
            )
        }
    }

    private val measCallback = object : GnssMeasurementsEvent.Callback() {
        override fun onGnssMeasurementsReceived(event: GnssMeasurementsEvent) {
            rawSupported = true
            val clk = event.clock
            val measurements = event.measurements
            val n = measurements.size
            if (n == 0) return
            val svid = IntArray(n); val con = IntArray(n); val cn0 = DoubleArray(n)
            val state = IntArray(n); val rx = LongArray(n); val rxu = LongArray(n)
            val prr = DoubleArray(n); val prru = DoubleArray(n); val cf = DoubleArray(n); val toff = LongArray(n)
            measurements.forEachIndexed { i, m ->
                svid[i] = m.svid; con[i] = m.constellationType; cn0[i] = m.cn0DbHz
                state[i] = m.state; rx[i] = m.receivedSvTimeNanos
                rxu[i] = m.receivedSvTimeUncertaintyNanos
                prr[i] = m.pseudorangeRateMetersPerSecond
                prru[i] = m.pseudorangeRateUncertaintyMetersPerSecond
                cf[i] = if (Build.VERSION.SDK_INT >= 26 && m.hasCarrierFrequencyHz())
                            m.carrierFrequencyHz.toDouble() else 1575.42e6
                toff[i] = m.timeOffsetNanos.toLong()
            }
            val fullBiasValid = clk.hasFullBiasNanos()
            engine.pushGnssRaw(
                timeBase.gnssToUnified(null), clk.timeNanos,
                if (fullBiasValid) clk.fullBiasNanos else 0L,
                if (clk.hasBiasNanos()) clk.biasNanos else 0.0,
                if (clk.hasDriftNanosPerSecond()) clk.driftNanosPerSecond else 0.0,
                if (clk.hasLeapSecond()) clk.leapSecond else 0,
                fullBiasValid,
                svid, con, cn0, state, rx, rxu, prr, prru, cf, toff
            )
        }
        override fun onStatusChanged(status: Int) {
            Timber.tag("Gnss").i("measurement status=%d", status)
        }
    }

    private val navCallback = object : GnssNavigationMessage.Callback() {
        override fun onGnssNavigationMessageReceived(msg: GnssNavigationMessage) {
            engine.pushNav(
                // GnssNavigationMessage type packs constellation in the high byte.
                (msg.type shr 8) and 0xFF, msg.svid, msg.type, msg.submessageId,
                msg.messageId, msg.data
            )
        }
    }

    fun start(useRaw: Boolean) {
        if (ContextCompat.checkSelfPermission(context, Manifest.permission.ACCESS_FINE_LOCATION)
            != PackageManager.PERMISSION_GRANTED) {
            Timber.tag("Gnss").e("location permission not granted"); return
        }
        val t = HandlerThread("odyx-gnss").also { it.start() }
        thread = t; handler = Handler(t.looper)

        val req = LocationRequest.Builder(Priority.PRIORITY_HIGH_ACCURACY, 200L)
            .setMinUpdateIntervalMillis(100L).build()
        try { fused.requestLocationUpdates(req, locationCallback, t.looper) }
        catch (e: SecurityException) { Timber.tag("Gnss").e(e, "fused updates") }

        if (useRaw && Build.VERSION.SDK_INT >= 27) {
            try {
                lm.registerGnssMeasurementsCallback(measCallback, handler!!)
                lm.registerGnssNavigationMessageCallback(navCallback, handler!!)
            } catch (e: SecurityException) { Timber.tag("Gnss").e(e, "raw register") }
        }
    }

    fun stop() {
        fused.removeLocationUpdates(locationCallback)
        try { lm.unregisterGnssMeasurementsCallback(measCallback) } catch (_: Exception) {}
        try { lm.unregisterGnssNavigationMessageCallback(navCallback) } catch (_: Exception) {}
        thread?.quitSafely(); thread = null; handler = null
    }
}
