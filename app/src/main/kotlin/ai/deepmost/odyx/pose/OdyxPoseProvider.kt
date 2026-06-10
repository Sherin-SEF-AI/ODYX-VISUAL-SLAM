package ai.deepmost.odyx.pose

import ai.deepmost.odyx.core.OdyxEngine
import ai.deepmost.odyx.jni.TrackingState
import kotlinx.coroutines.CoroutineScope
import kotlinx.coroutines.Dispatchers
import kotlinx.coroutines.Job
import kotlinx.coroutines.delay
import kotlinx.coroutines.flow.Flow
import kotlinx.coroutines.flow.MutableStateFlow
import kotlinx.coroutines.flow.filterNotNull
import kotlinx.coroutines.isActive
import kotlinx.coroutines.launch

/**
 * Polls the native estimator at ~60 Hz and republishes its pose as a [Flow].
 * This is the concrete ODYX [PoseProvider] — the object LANYX/FERYX inject in
 * place of their ARCore provider.
 */
class OdyxPoseProvider(
    private val engine: OdyxEngine,
    private val scope: CoroutineScope = CoroutineScope(Dispatchers.Default),
    private val hz: Int = 60
) : PoseProvider {

    private val _poses = MutableStateFlow<OdyxPose?>(null)
    override val poses: Flow<OdyxPose> = _poses.filterNotNull()
    private var job: Job? = null

    override fun latest(): OdyxPose? = _poses.value

    override fun start() {
        if (job != null) return
        val periodMs = (1000L / hz).coerceAtLeast(8L)
        job = scope.launch {
            while (isActive) {
                engine.pose()?.let { p ->
                    val tele = engine.telemetry()
                    val state = when (tele?.tracking) {
                        TrackingState.NOMINAL -> PoseTrackingState.TRACKING
                        TrackingState.INITIALIZING -> PoseTrackingState.INITIALIZING
                        TrackingState.UNSTABLE -> PoseTrackingState.UNSTABLE
                        TrackingState.LOST -> PoseTrackingState.LOST
                        else -> PoseTrackingState.UNINITIALIZED
                    }
                    val pose = OdyxPose(
                        timestampNs = p.tNs,
                        localRotation = p.q,
                        localPosition = p.p,
                        velocity = p.v,
                        enuPosition = p.enu,
                        poseCovariance = doubleArrayOf(p.covRotTrace, p.covPosTrace),
                        valid = p.valid,
                        trackingState = state
                    )
                    _poses.value = pose
                }
                delay(periodMs)
            }
        }
    }

    override fun stop() { job?.cancel(); job = null }
}
