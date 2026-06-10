package ai.deepmost.odyx.service

import android.app.Notification
import android.app.NotificationChannel
import android.app.NotificationManager
import android.app.Service
import android.content.Context
import android.content.Intent
import android.content.pm.ServiceInfo
import android.os.Build
import android.os.IBinder
import android.os.PowerManager
import androidx.core.app.NotificationCompat
import ai.deepmost.odyx.OdyxApp
import ai.deepmost.odyx.R
import ai.deepmost.odyx.config.SettingsStore
import kotlinx.coroutines.CoroutineScope
import kotlinx.coroutines.Dispatchers
import kotlinx.coroutines.flow.first
import kotlinx.coroutines.launch
import timber.log.Timber

/**
 * Foreground service that keeps capture + estimation alive with the screen off.
 * Holds a partial wake lock and runs the shared [OdyxController].
 */
class CaptureService : Service() {

    private val scope = CoroutineScope(Dispatchers.Main)
    private var wakeLock: PowerManager.WakeLock? = null

    override fun onCreate() {
        super.onCreate()
        createChannel()
        startForegroundCompat()
        val pm = getSystemService(Context.POWER_SERVICE) as PowerManager
        wakeLock = pm.newWakeLock(PowerManager.PARTIAL_WAKE_LOCK, "odyx:capture").apply {
            setReferenceCounted(false); acquire(60 * 60 * 1000L)
        }
        scope.launch {
            val settings = SettingsStore(applicationContext).settings.first()
            OdyxApp.controller().start(settings)
        }
        Timber.tag("CaptureService").i("started")
    }

    override fun onStartCommand(intent: Intent?, flags: Int, startId: Int): Int = START_STICKY
    override fun onBind(intent: Intent?): IBinder? = null

    override fun onDestroy() {
        OdyxApp.controller().stop()
        wakeLock?.let { if (it.isHeld) it.release() }
        Timber.tag("CaptureService").i("stopped")
        super.onDestroy()
    }

    private fun startForegroundCompat() {
        val n: Notification = NotificationCompat.Builder(this, CHANNEL)
            .setContentTitle("ODYX")
            .setContentText("Visual-inertial SLAM running")
            .setSmallIcon(R.drawable.ic_stat_odyx)
            .setOngoing(true)
            .build()
        if (Build.VERSION.SDK_INT >= 34) {
            startForeground(
                NOTIF_ID, n,
                ServiceInfo.FOREGROUND_SERVICE_TYPE_CAMERA or
                    ServiceInfo.FOREGROUND_SERVICE_TYPE_LOCATION
            )
        } else if (Build.VERSION.SDK_INT >= 29) {
            startForeground(NOTIF_ID, n,
                ServiceInfo.FOREGROUND_SERVICE_TYPE_CAMERA or
                    ServiceInfo.FOREGROUND_SERVICE_TYPE_LOCATION)
        } else {
            startForeground(NOTIF_ID, n)
        }
    }

    private fun createChannel() {
        val nm = getSystemService(NotificationManager::class.java)
        val ch = NotificationChannel(CHANNEL, "ODYX Capture", NotificationManager.IMPORTANCE_LOW)
        nm.createNotificationChannel(ch)
    }

    companion object {
        private const val CHANNEL = "odyx_capture"
        private const val NOTIF_ID = 1001
        fun start(ctx: Context) {
            val i = Intent(ctx, CaptureService::class.java)
            if (Build.VERSION.SDK_INT >= 26) ctx.startForegroundService(i) else ctx.startService(i)
        }
        fun stop(ctx: Context) { ctx.stopService(Intent(ctx, CaptureService::class.java)) }
    }
}
