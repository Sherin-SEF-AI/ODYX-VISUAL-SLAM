package ai.deepmost.odyx.ui.settings

import androidx.compose.foundation.background
import androidx.compose.foundation.layout.*
import androidx.compose.foundation.rememberScrollState
import androidx.compose.foundation.verticalScroll
import androidx.compose.material3.*
import androidx.compose.runtime.*
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.unit.dp
import ai.deepmost.odyx.config.GnssCoupling
import ai.deepmost.odyx.config.OdyxSettings
import ai.deepmost.odyx.config.SettingsStore
import ai.deepmost.odyx.core.OdyxController
import ai.deepmost.odyx.ui.common.Hairline
import ai.deepmost.odyx.ui.common.Readout
import ai.deepmost.odyx.ui.common.SectionHeader
import ai.deepmost.odyx.ui.theme.OdyxAccent
import ai.deepmost.odyx.ui.theme.OdyxBlack
import ai.deepmost.odyx.ui.theme.OdyxText
import kotlinx.coroutines.flow.first
import kotlinx.coroutines.launch

@Composable
fun SettingsScreen(controller: OdyxController, store: SettingsStore, modifier: Modifier = Modifier,
                   onOpenCalib: () -> Unit = {}) {
    val scope = rememberCoroutineScope()
    var s by remember { mutableStateOf(OdyxSettings()) }
    LaunchedEffect(Unit) { s = store.settings.first() }

    fun apply(newS: OdyxSettings) {
        s = newS
        scope.launch { store.update(newS) }
        controller.applySettings(newS)
        controller.engine.setGnssMode(newS.gnssMode)
        controller.engine.setOnlineCalib(newS.onlineTd, newS.onlineExtr)
        controller.engine.setRollingShutter(newS.rollingShutter)
        controller.engine.setLoopClosure(newS.loopEnabled)
    }

    Column(modifier.fillMaxSize().background(OdyxBlack).verticalScroll(rememberScrollState())) {
        SectionHeader("GNSS")
        Row(Modifier.fillMaxWidth().padding(12.dp), horizontalArrangement = Arrangement.spacedBy(8.dp)) {
            GnssCoupling.entries.forEach { mode ->
                FilterChip(selected = s.gnssMode == mode, onClick = { apply(s.copy(gnssMode = mode)) },
                    label = { Text(mode.name) })
            }
        }
        ToggleRow("raw GNSS (tight)", s.useRawGnss) { apply(s.copy(useRawGnss = it)) }
        Hairline()

        SectionHeader("ESTIMATOR")
        ToggleRow("online td", s.onlineTd) { apply(s.copy(onlineTd = it)) }
        ToggleRow("online extrinsics", s.onlineExtr) { apply(s.copy(onlineExtr = it)) }
        ToggleRow("rolling-shutter comp", s.rollingShutter) { apply(s.copy(rollingShutter = it)) }
        ToggleRow("loop closure", s.loopEnabled) { apply(s.copy(loopEnabled = it)) }
        SliderRow("window keyframes", s.windowKeyframes.toFloat(), 6f, 20f) {
            apply(s.copy(windowKeyframes = it.toInt()))
        }
        SliderRow("target features", s.targetFeatures.toFloat(), 60f, 300f) {
            apply(s.copy(targetFeatures = it.toInt()))
        }
        SliderRow("kf parallax px", s.kfParallaxPx.toFloat(), 5f, 30f) {
            apply(s.copy(kfParallaxPx = it.toDouble()))
        }
        SliderRow("kf tracked ratio", s.kfTrackedRatio.toFloat(), 0.2f, 0.9f) {
            apply(s.copy(kfTrackedRatio = it.toDouble()))
        }
        SliderRow("detect grid cols", s.gridCols.toFloat(), 3f, 10f) {
            apply(s.copy(gridCols = it.toInt()))
        }
        SliderRow("detect grid rows", s.gridRows.toFloat(), 3f, 10f) {
            apply(s.copy(gridRows = it.toInt()))
        }
        SliderRow("RANSAC px", s.ransacThreshPx.toFloat(), 0.5f, 4f) {
            apply(s.copy(ransacThreshPx = it.toDouble()))
        }
        SliderRow("Huber px", s.huberPx.toFloat(), 0.5f, 4f) { apply(s.copy(huberPx = it.toDouble())) }
        Hairline()

        SectionHeader("LOOP CLOSURE")
        SliderRow("DBoW score", s.dbowScoreThresh.toFloat(), 0.01f, 0.2f) {
            apply(s.copy(dbowScoreThresh = it.toDouble()))
        }
        SliderRow("min inliers", s.loopMinInliers.toFloat(), 10f, 60f) {
            apply(s.copy(loopMinInliers = it.toInt()))
        }
        Hairline()

        SectionHeader("CAPS")
        SliderRow("max landmarks", s.maxLandmarks.toFloat(), 100f, 800f) {
            apply(s.copy(maxLandmarks = it.toInt()))
        }
        SliderRow("max keyframe DB", s.maxKeyframesDb.toFloat(), 100f, 600f) {
            apply(s.copy(maxKeyframesDb = it.toInt()))
        }
        SliderRow("IMU sampling µs", s.imuSamplingUs.toFloat(), 2500f, 10000f) {
            apply(s.copy(imuSamplingUs = it.toInt()))
        }
        Hairline()

        SectionHeader("CALIBRATION")
        Readout("profile", s.calibProfile)
        Row(Modifier.fillMaxWidth().padding(12.dp), horizontalArrangement = Arrangement.spacedBy(8.dp)) {
            Button(onClick = onOpenCalib) { Text("CHECKERBOARD CALIB") }
        }
        Text(
            "Run in-app checkerboard calibration (camera intrinsics) above, or the " +
                "Allan-variance logger (Sessions ▸ Allan) for imu_noise.yaml. Calib YAMLs " +
                "can also be edited in assets/calib/.",
            color = OdyxText, style = MaterialTheme.typography.bodySmall,
            modifier = Modifier.padding(12.dp)
        )
    }
}

@Composable
private fun ToggleRow(label: String, value: Boolean, onChange: (Boolean) -> Unit) {
    Row(Modifier.fillMaxWidth().padding(horizontal = 12.dp, vertical = 4.dp),
        horizontalArrangement = Arrangement.SpaceBetween, verticalAlignment = Alignment.CenterVertically) {
        Text(label, color = OdyxText, style = MaterialTheme.typography.bodyMedium)
        Switch(checked = value, onCheckedChange = onChange)
    }
}

@Composable
private fun SliderRow(label: String, value: Float, min: Float, max: Float, onChange: (Float) -> Unit) {
    Column(Modifier.fillMaxWidth().padding(horizontal = 12.dp, vertical = 2.dp)) {
        Row(Modifier.fillMaxWidth(), horizontalArrangement = Arrangement.SpaceBetween) {
            Text(label, color = OdyxText, style = MaterialTheme.typography.bodyMedium)
            Text("%.2f".format(value), color = OdyxAccent, style = MaterialTheme.typography.bodyMedium)
        }
        Slider(value = value, onValueChange = onChange, valueRange = min..max)
    }
}
