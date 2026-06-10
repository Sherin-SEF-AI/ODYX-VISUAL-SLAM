package ai.deepmost.odyx.pose

import kotlinx.coroutines.flow.Flow

/** Tracking lifecycle exposed to consumers. */
enum class PoseTrackingState { UNINITIALIZED, INITIALIZING, TRACKING, UNSTABLE, LOST }

/**
 * 6-DoF pose sample. [localPose] is the gravity-aligned local world (metric VIO
 * frame). [enuPose] is the globally-georeferenced ENU position (null until GNSS
 * anchoring succeeds). [poseCovariance] is the 6x6 (rot,pos) covariance row-major.
 */
data class OdyxPose(
    val timestampNs: Long,
    val localRotation: FloatArray,    // quaternion w,x,y,z
    val localPosition: FloatArray,    // x,y,z (m)
    val velocity: FloatArray,
    val enuPosition: FloatArray?,     // East,North,Up (m) or null
    val poseCovariance: DoubleArray,  // length 6 (pos-trace, rot-trace summary) or full
    val valid: Boolean,
    val trackingState: PoseTrackingState
)

/**
 * The contract other in-house apps (LANYX, FERYX) program against — a drop-in
 * replacement for the ARCore-backed provider. They collect [poses] and never
 * touch ARCore.
 */
interface PoseProvider {
    val poses: Flow<OdyxPose>
    fun latest(): OdyxPose?
    fun start()
    fun stop()
}
