package ai.deepmost.odyx.config

import android.content.Context
import androidx.datastore.preferences.core.booleanPreferencesKey
import androidx.datastore.preferences.core.doublePreferencesKey
import androidx.datastore.preferences.core.edit
import androidx.datastore.preferences.core.intPreferencesKey
import androidx.datastore.preferences.core.stringPreferencesKey
import androidx.datastore.preferences.preferencesDataStore
import kotlinx.coroutines.flow.Flow
import kotlinx.coroutines.flow.map

private val Context.dataStore by preferencesDataStore(name = "odyx_settings")

/** User-tunable settings persisted via DataStore (mirrors the SETTINGS screen). */
data class OdyxSettings(
    val gnssMode: GnssCoupling = GnssCoupling.LOOSE,
    val rollingShutter: Boolean = true,
    val onlineTd: Boolean = true,
    val onlineExtr: Boolean = true,
    val loopEnabled: Boolean = true,
    val windowKeyframes: Int = 12,
    val targetFeatures: Int = 150,
    val gridCols: Int = 6,
    val gridRows: Int = 5,
    val kfParallaxPx: Double = 12.0,
    val kfTrackedRatio: Double = 0.5,
    val ransacThreshPx: Double = 1.5,
    val dbowScoreThresh: Double = 0.045,
    val loopMinInliers: Int = 25,
    val huberPx: Double = 1.5,
    val maxLandmarks: Int = 400,
    val maxKeyframesDb: Int = 300,
    val imuSamplingUs: Int = 5000,
    val calibProfile: String = "default",
    val useRawGnss: Boolean = true
)

class SettingsStore(private val context: Context) {
    private object K {
        val gnss = intPreferencesKey("gnss_mode")
        val rs = booleanPreferencesKey("rolling_shutter")
        val td = booleanPreferencesKey("online_td")
        val extr = booleanPreferencesKey("online_extr")
        val loop = booleanPreferencesKey("loop_enabled")
        val win = intPreferencesKey("window_kf")
        val feat = intPreferencesKey("target_features")
        val gc = intPreferencesKey("grid_cols")
        val gr = intPreferencesKey("grid_rows")
        val par = doublePreferencesKey("kf_parallax")
        val trk = doublePreferencesKey("kf_tracked_ratio")
        val ran = doublePreferencesKey("ransac_px")
        val dbow = doublePreferencesKey("dbow_score")
        val li = intPreferencesKey("loop_min_inliers")
        val hub = doublePreferencesKey("huber_px")
        val ml = intPreferencesKey("max_landmarks")
        val mkdb = intPreferencesKey("max_kf_db")
        val imu = intPreferencesKey("imu_us")
        val calib = stringPreferencesKey("calib_profile")
        val raw = booleanPreferencesKey("use_raw_gnss")
    }

    val settings: Flow<OdyxSettings> = context.dataStore.data.map { p ->
        OdyxSettings(
            gnssMode = GnssCoupling.entries.getOrElse(p[K.gnss] ?: 1) { GnssCoupling.LOOSE },
            rollingShutter = p[K.rs] ?: true,
            onlineTd = p[K.td] ?: true,
            onlineExtr = p[K.extr] ?: true,
            loopEnabled = p[K.loop] ?: true,
            windowKeyframes = p[K.win] ?: 12,
            targetFeatures = p[K.feat] ?: 150,
            gridCols = p[K.gc] ?: 6,
            gridRows = p[K.gr] ?: 5,
            kfParallaxPx = p[K.par] ?: 12.0,
            kfTrackedRatio = p[K.trk] ?: 0.5,
            ransacThreshPx = p[K.ran] ?: 1.5,
            dbowScoreThresh = p[K.dbow] ?: 0.045,
            loopMinInliers = p[K.li] ?: 25,
            huberPx = p[K.hub] ?: 1.5,
            maxLandmarks = p[K.ml] ?: 400,
            maxKeyframesDb = p[K.mkdb] ?: 300,
            imuSamplingUs = p[K.imu] ?: 5000,
            calibProfile = p[K.calib] ?: "default",
            useRawGnss = p[K.raw] ?: true
        )
    }

    suspend fun update(s: OdyxSettings) {
        context.dataStore.edit { p ->
            p[K.gnss] = s.gnssMode.ordinal; p[K.rs] = s.rollingShutter
            p[K.td] = s.onlineTd; p[K.extr] = s.onlineExtr; p[K.loop] = s.loopEnabled
            p[K.win] = s.windowKeyframes; p[K.feat] = s.targetFeatures
            p[K.gc] = s.gridCols; p[K.gr] = s.gridRows
            p[K.par] = s.kfParallaxPx; p[K.trk] = s.kfTrackedRatio; p[K.ran] = s.ransacThreshPx
            p[K.dbow] = s.dbowScoreThresh; p[K.li] = s.loopMinInliers
            p[K.hub] = s.huberPx; p[K.ml] = s.maxLandmarks; p[K.mkdb] = s.maxKeyframesDb
            p[K.imu] = s.imuSamplingUs; p[K.calib] = s.calibProfile; p[K.raw] = s.useRawGnss
        }
    }
}
