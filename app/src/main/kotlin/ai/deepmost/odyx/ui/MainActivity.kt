package ai.deepmost.odyx.ui

import android.Manifest
import android.os.Build
import android.os.Bundle
import androidx.activity.ComponentActivity
import androidx.activity.compose.setContent
import androidx.compose.foundation.layout.fillMaxSize
import androidx.compose.foundation.layout.padding
import androidx.compose.material.icons.Icons
import androidx.compose.material.icons.filled.*
import androidx.compose.material3.*
import androidx.compose.runtime.*
import androidx.compose.ui.Modifier
import androidx.compose.ui.graphics.vector.ImageVector
import androidx.compose.ui.unit.dp
import ai.deepmost.odyx.OdyxApp
import ai.deepmost.odyx.config.SettingsStore
import ai.deepmost.odyx.service.CaptureService
import ai.deepmost.odyx.ui.map.MapScreen
import ai.deepmost.odyx.ui.sessions.SessionsScreen
import ai.deepmost.odyx.ui.settings.SettingsScreen
import ai.deepmost.odyx.ui.slam.SlamScreen
import ai.deepmost.odyx.ui.telemetry.TelemetryScreen
import ai.deepmost.odyx.ui.theme.OdyxTheme
import com.google.accompanist.permissions.ExperimentalPermissionsApi
import com.google.accompanist.permissions.rememberMultiplePermissionsState

class MainActivity : ComponentActivity() {

    private val controller get() = OdyxApp.controller()

    @OptIn(ExperimentalPermissionsApi::class)
    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        val store = SettingsStore(applicationContext)

        setContent {
            OdyxTheme {
                val perms = buildList {
                    add(Manifest.permission.CAMERA)
                    add(Manifest.permission.ACCESS_FINE_LOCATION)
                    if (Build.VERSION.SDK_INT >= 33) add(Manifest.permission.POST_NOTIFICATIONS)
                    if (Build.VERSION.SDK_INT >= 31) add(Manifest.permission.HIGH_SAMPLING_RATE_SENSORS)
                }
                val state = rememberMultiplePermissionsState(perms)

                LaunchedEffect(state.allPermissionsGranted) {
                    if (state.allPermissionsGranted) {
                        CaptureService.start(this@MainActivity)   // foreground capture + controller
                        // Debug harness (adb-scriptable, no UI taps needed):
                        //   --ez odyx_record true     -> auto-record this session
                        //   --es odyx_replay <name>   -> replay a recorded session
                        val c = OdyxApp.controller()
                        if (intent?.getBooleanExtra("odyx_record", false) == true) {
                            kotlinx.coroutines.delay(1500)
                            c.startRecording("auto")
                            kotlinx.coroutines.delay(30000)   // record 30 s, then flush
                            c.stopRecording()
                        }
                        intent?.getStringExtra("odyx_replay")?.let { name ->
                            kotlinx.coroutines.delay(1500)
                            c.replaySession(name)
                        }
                    } else {
                        state.launchMultiplePermissionRequest()
                    }
                }

                if (!state.allPermissionsGranted) {
                    PermissionRationale { state.launchMultiplePermissionRequest() }
                } else {
                    OdyxScaffold(store)
                }
            }
        }
    }

    override fun onDestroy() {
        super.onDestroy()
        if (isFinishing) {
            CaptureService.stop(this)
            controller.stop()
        }
    }
}

private enum class Tab(val label: String, val icon: ImageVector) {
    SLAM("SLAM", Icons.Filled.CenterFocusStrong),
    MAP("MAP", Icons.Filled.Map),
    TELE("TELEM", Icons.Filled.Analytics),
    SESS("SESS", Icons.Filled.FolderOpen),
    SET("SET", Icons.Filled.Settings)
}

@Composable
private fun OdyxScaffold(store: SettingsStore) {
    val controller = OdyxApp.controller()
    var tab by remember { mutableStateOf(Tab.SLAM) }
    var showCalib by remember { mutableStateOf(false) }
    if (showCalib) {
        ai.deepmost.odyx.ui.calib.CheckerboardCalibScreen(controller, onDone = { showCalib = false })
        return
    }
    Scaffold(
        bottomBar = {
            NavigationBar {
                Tab.entries.forEach { t ->
                    NavigationBarItem(
                        selected = tab == t,
                        onClick = { tab = t },
                        icon = { Icon(t.icon, t.label) },
                        label = { Text(t.label, style = MaterialTheme.typography.labelSmall) }
                    )
                }
            }
        }
    ) { pad ->
        val m = Modifier.fillMaxSize().padding(pad)
        when (tab) {
            Tab.SLAM -> SlamScreen(controller, m)
            Tab.MAP -> MapScreen(controller, m)
            Tab.TELE -> TelemetryScreen(controller, m)
            Tab.SESS -> SessionsScreen(controller, m)
            Tab.SET -> SettingsScreen(controller, store, m, onOpenCalib = { showCalib = true })
        }
    }
}

@Composable
private fun PermissionRationale(onRequest: () -> Unit) {
    androidx.compose.foundation.layout.Column(
        Modifier.fillMaxSize().padding(24.dp),
        verticalArrangement = androidx.compose.foundation.layout.Arrangement.Center
    ) {
        Text("ODYX needs CAMERA, LOCATION (GNSS), high-rate motion sensors and " +
            "notifications to run on-device visual-inertial SLAM.",
            style = MaterialTheme.typography.bodyLarge)
        androidx.compose.foundation.layout.Spacer(Modifier.padding(8.dp))
        Button(onClick = onRequest) { Text("GRANT PERMISSIONS") }
    }
}
