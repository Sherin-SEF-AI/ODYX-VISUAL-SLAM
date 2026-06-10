package ai.deepmost.odyx.ui.telemetry

import androidx.compose.foundation.background
import androidx.compose.foundation.layout.Column
import androidx.compose.foundation.layout.fillMaxSize
import androidx.compose.foundation.rememberScrollState
import androidx.compose.foundation.verticalScroll
import androidx.compose.runtime.Composable
import androidx.compose.runtime.getValue
import androidx.compose.runtime.mutableStateListOf
import androidx.compose.runtime.remember
import androidx.compose.ui.Modifier
import androidx.lifecycle.compose.collectAsStateWithLifecycle
import ai.deepmost.odyx.core.OdyxController
import ai.deepmost.odyx.ui.common.*
import ai.deepmost.odyx.ui.theme.*

@Composable
fun TelemetryScreen(controller: OdyxController, modifier: Modifier = Modifier) {
    val tele by controller.telemetry.collectAsStateWithLifecycle()
    val t = tele
    val optHist = remember { mutableStateListOf<Float>() }
    val bgHist = remember { mutableStateListOf<Float>() }
    val baHist = remember { mutableStateListOf<Float>() }
    val velHist = remember { mutableStateListOf<Float>() }
    val driftHist = remember { mutableStateListOf<Float>() }
    fun push(l: MutableList<Float>, v: Float) { l.add(v); if (l.size > 120) l.removeAt(0) }
    t?.let {
        push(optHist, it.optimizeMs.toFloat())
        push(bgHist, it.gyroBias.getOrElse(0) { 0f })
        push(baHist, it.accelBias.getOrElse(0) { 0f })
        push(velHist, kotlin.math.sqrt(it.vel.fold(0f) { a, x -> a + x * x }))
        push(driftHist, it.driftProxy.toFloat())
    }

    Column(modifier.fillMaxSize().background(OdyxBlack).verticalScroll(rememberScrollState())) {
        if (t == null) { Readout("telemetry", "waiting…"); return@Column }

        SectionHeader("STATE")
        Readout("tracking", t.tracking.name, accent = true)
        Readout("init", t.init.name)
        Readout("gnss mode", listOf("OFF", "LOOSE", "TIGHT")[t.gnssMode.coerceIn(0, 2)])
        if (t.calibDefault) Readout("calibration", "DEFAULT (uncalibrated)", valueColor = OdyxBad)
        Hairline()

        SectionHeader("FRONT-END / BACKEND")
        Readout("features tracked", "${t.nTracked}")
        Readout("keyframes (window)", "${t.nKeyframes} / ${t.windowSize}")
        Readout("landmarks", "${t.nLandmarks}")
        Readout("frontend", "${f(t.frontendMs, 2)} ms")
        Readout("optimize", "${f(t.optimizeMs, 2)} ms", accent = true)
        Sparkline(optHist, OdyxAccent, "optimize ms")
        Hairline()

        SectionHeader("RATES")
        Readout("camera", "${f(t.frameRate, 1)} Hz")
        Readout("imu", "${f(t.imuRate, 1)} Hz")
        Readout("gnss", "${f(t.gnssRate, 2)} Hz")
        Hairline()

        SectionHeader("CALIBRATION / STATE")
        Readout("td (cam↔imu)", "${f(t.td * 1000.0, 3)} ms", accent = true)
        Readout("gyro bias", v3(t.gyroBias))
        Sparkline(bgHist, OdyxGood, "gyro bias x")
        Readout("accel bias", v3(t.accelBias))
        Sparkline(baHist, OdyxGood, "accel bias x")
        Readout("velocity", v3(t.vel, 3))
        Sparkline(velHist, OdyxAccent, "speed |v| m/s")
        Readout("gravity resid", "${f(t.gravityResid, 4)}")
        Hairline()

        SectionHeader("GNSS")
        Readout("sats", "${t.gnssSats}")
        Readout("fix type", listOf("none", "loose", "tight")[t.gnssFixType.coerceIn(0, 2)])
        Readout("reported acc", "${f(t.gnssAcc, 2)} m")
        Readout("ENU aligned", if (t.enuAligned) "YES" else "no",
            valueColor = if (t.enuAligned) OdyxGood else OdyxTextDim)
        Readout("ENU resid", "${f(t.enuResid, 3)} m")
        Readout("drift proxy", "${f(t.driftProxy, 3)} m")
        Sparkline(driftHist, OdyxWarn, "VIO↔GNSS drift m")
        Hairline()

        SectionHeader("SYSTEM")
        Readout("loop closures", "${t.loopCount}", accent = t.loopCount > 0)
        Readout("thermal", thermalLabel(t.thermal),
            valueColor = if (t.thermal >= 2) OdyxWarn else OdyxTextDim)
        Readout("frame stride", "${t.frameStride}")
        Readout("target features", "${t.targetFeatures}")
    }
}

private fun thermalLabel(s: Int) = when (s) {
    0 -> "none"; 1 -> "light"; 2 -> "moderate"; 3 -> "severe"; else -> "critical"
}
