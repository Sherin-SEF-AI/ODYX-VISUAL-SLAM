package ai.deepmost.odyx.config

/** Camera intrinsics + radtan distortion (k1,k2,p1,p2,k3). */
data class CameraIntrinsics(
    val width: Int = 640,
    val height: Int = 480,
    val fx: Double = 480.0,
    val fy: Double = 480.0,
    val cx: Double = 320.0,
    val cy: Double = 240.0,
    val dist: DoubleArray = doubleArrayOf(0.0, 0.0, 0.0, 0.0, 0.0),
    val valid: Boolean = false
)

/** IMU continuous-time noise model (Allan-variance derived). */
data class ImuNoise(
    val accN: Double = 8.0e-3,
    val gyrN: Double = 1.2e-3,
    val accW: Double = 4.0e-4,
    val gyrW: Double = 2.0e-5,
    val g: Double = 9.81
)

/** Camera-IMU extrinsics T_bc (quaternion w,x,y,z + translation) + initial td. */
data class Extrinsics(
    val qw: Double = 1.0, val qx: Double = 0.0, val qy: Double = 0.0, val qz: Double = 0.0,
    val tx: Double = 0.0, val ty: Double = 0.0, val tz: Double = 0.0,
    val td: Double = 0.0,
    val valid: Boolean = false
)

enum class GnssCoupling(val code: Int) { OFF(0), LOOSE(1), TIGHT(2) }

/**
 * Full estimator configuration. [toParams] serialises to the exact 45-element
 * DoubleArray layout consumed by nativeLoadConfig (see odyx_jni.cpp).
 */
data class OdyxConfig(
    val cam: CameraIntrinsics = CameraIntrinsics(),
    val imu: ImuNoise = ImuNoise(),
    val extr: Extrinsics = Extrinsics(),
    val leverArm: DoubleArray = doubleArrayOf(0.0, 0.0, 0.0),
    val targetFeatures: Int = 150,
    val gridCols: Int = 6,
    val gridRows: Int = 5,
    val kfParallaxPx: Double = 12.0,
    val kfTrackedRatio: Double = 0.5,
    val windowKeyframes: Int = 12,
    val maxLandmarks: Int = 400,
    val huberPx: Double = 1.5,
    val onlineTd: Boolean = true,
    val onlineExtr: Boolean = true,
    val rollingShutter: Boolean = true,
    val loopEnabled: Boolean = true,
    val dbowScoreThresh: Double = 0.045,
    val loopMinInliers: Int = 25,
    val gnssMode: GnssCoupling = GnssCoupling.LOOSE,
    val maxKeyframesDb: Int = 300,
    val ransacThreshPx: Double = 1.5,
    val gnssMatchWindowS: Double = 0.35
) {
    fun toParams(): DoubleArray = doubleArrayOf(
        cam.width.toDouble(), cam.height.toDouble(),
        cam.fx, cam.fy, cam.cx, cam.cy,
        cam.dist[0], cam.dist[1], cam.dist[2], cam.dist[3], cam.dist[4],
        if (cam.valid) 1.0 else 0.0,
        imu.accN, imu.gyrN, imu.accW, imu.gyrW, imu.g,
        extr.qw, extr.qx, extr.qy, extr.qz,
        extr.tx, extr.ty, extr.tz,
        extr.td, if (extr.valid) 1.0 else 0.0,
        leverArm[0], leverArm[1], leverArm[2],
        targetFeatures.toDouble(), gridCols.toDouble(), gridRows.toDouble(),
        kfParallaxPx, kfTrackedRatio,
        windowKeyframes.toDouble(), maxLandmarks.toDouble(), huberPx,
        if (onlineTd) 1.0 else 0.0, if (onlineExtr) 1.0 else 0.0, if (rollingShutter) 1.0 else 0.0,
        if (loopEnabled) 1.0 else 0.0, dbowScoreThresh, loopMinInliers.toDouble(),
        gnssMode.code.toDouble(), maxKeyframesDb.toDouble(),
        // appended (indices 45,46) — keep native ConfigMarshal in sync
        ransacThreshPx, gnssMatchWindowS
    )
}
