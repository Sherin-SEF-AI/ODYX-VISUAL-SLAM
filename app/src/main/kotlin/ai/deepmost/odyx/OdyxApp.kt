package ai.deepmost.odyx

import android.app.Application
import ai.deepmost.odyx.core.OdyxController
import timber.log.Timber

/**
 * Application entry point. Holds the single [OdyxController] shared by the UI and
 * the foreground capture service, and initialises logging.
 */
class OdyxApp : Application() {
    val controller: OdyxController by lazy { OdyxController(this) }

    override fun onCreate() {
        super.onCreate()
        if (BuildConfig.DEBUG) Timber.plant(Timber.DebugTree())
        instance = this
        Timber.tag("ODYX").i("ODYX %s started", BuildConfig.VERSION_NAME)
    }

    companion object {
        @Volatile lateinit var instance: OdyxApp
            private set
        fun controller(): OdyxController = instance.controller
    }
}
