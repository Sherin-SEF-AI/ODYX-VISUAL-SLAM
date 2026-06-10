package ai.deepmost.odyx.ui.map

import android.opengl.GLES20
import android.opengl.GLSurfaceView
import android.opengl.Matrix
import ai.deepmost.odyx.jni.MapSnapshotK
import ai.deepmost.odyx.jni.PoseSnapshot
import java.nio.ByteBuffer
import java.nio.ByteOrder
import java.nio.FloatBuffer
import javax.microedition.khronos.egl.EGLConfig
import javax.microedition.khronos.opengles.GL10
import kotlin.math.cos
import kotlin.math.sin

/**
 * Minimal GLES2 renderer for the sparse map: trajectory polyline (local =
 * neutral, ENU = accent), landmark cloud, current pose frustum, loop edges,
 * GNSS markers. Orbit / pan / pinch-zoom driven from [MapGLView].
 */
class MapRenderer : GLSurfaceView.Renderer {

    @Volatile var snapshot: MapSnapshotK? = null
    @Volatile var pose: PoseSnapshot? = null
    @Volatile var showEnu = false
    @Volatile var followVehicle = true

    // camera controls
    @Volatile var yaw = 0.6f
    @Volatile var pitch = 0.5f
    @Volatile var dist = 8f
    @Volatile var panX = 0f
    @Volatile var panY = 0f

    private var program = 0
    private var aPos = 0
    private var uMvp = 0
    private var uColor = 0
    private var uPointSize = 0
    private val mvp = FloatArray(16)
    private val proj = FloatArray(16)
    private val view = FloatArray(16)
    private val model = FloatArray(16)

    private val vert = "uniform mat4 uMvp; uniform float uPointSize; attribute vec4 aPos;" +
        "void main(){ gl_Position = uMvp * aPos; gl_PointSize = uPointSize; }"
    private val frag = "precision mediump float; uniform vec4 uColor;" +
        "void main(){ gl_FragColor = uColor; }"

    override fun onSurfaceCreated(gl: GL10?, config: EGLConfig?) {
        GLES20.glClearColor(0.04f, 0.04f, 0.043f, 1f)
        GLES20.glEnable(GLES20.GL_DEPTH_TEST)
        val vs = compile(GLES20.GL_VERTEX_SHADER, vert)
        val fs = compile(GLES20.GL_FRAGMENT_SHADER, frag)
        program = GLES20.glCreateProgram().also {
            GLES20.glAttachShader(it, vs); GLES20.glAttachShader(it, fs); GLES20.glLinkProgram(it)
        }
        aPos = GLES20.glGetAttribLocation(program, "aPos")
        uMvp = GLES20.glGetUniformLocation(program, "uMvp")
        uColor = GLES20.glGetUniformLocation(program, "uColor")
        uPointSize = GLES20.glGetUniformLocation(program, "uPointSize")
    }

    override fun onSurfaceChanged(gl: GL10?, w: Int, h: Int) {
        GLES20.glViewport(0, 0, w, h)
        Matrix.perspectiveM(proj, 0, 50f, w.toFloat() / h, 0.05f, 1000f)
    }

    override fun onDrawFrame(gl: GL10?) {
        GLES20.glClear(GLES20.GL_COLOR_BUFFER_BIT or GLES20.GL_DEPTH_BUFFER_BIT)
        GLES20.glUseProgram(program)

        // orbit camera around the current pose (or origin)
        val p = pose
        val cx = if (followVehicle && p != null) p.p[0] else 0f
        val cy = if (followVehicle && p != null) p.p[1] else 0f
        val cz = if (followVehicle && p != null) p.p[2] else 0f
        val ex = cx + dist * cos(pitch) * cos(yaw)
        val ey = cy + dist * cos(pitch) * sin(yaw)
        val ez = cz + dist * sin(pitch)
        Matrix.setLookAtM(view, 0, ex, ey, ez, cx + panX, cy + panY, cz, 0f, 0f, 1f)
        Matrix.setIdentityM(model, 0)
        Matrix.multiplyMM(mvp, 0, view, 0, model, 0)
        Matrix.multiplyMM(mvp, 0, proj, 0, mvp, 0)
        GLES20.glUniformMatrix4fv(uMvp, 1, false, mvp, 0)

        val s = snapshot

        // grid floor
        drawLines(buildGrid(), 0.16f, 0.16f, 0.18f, 1f, GLES20.GL_LINES)

        if (s != null) {
            // trajectory
            val traj = if (showEnu && s.enuValid && s.trajEnu.isNotEmpty()) s.trajEnu else s.trajLocal
            if (showEnu && s.enuValid) drawLineStrip(traj, 1f, 0.48f, 0.10f, 1f)   // accent
            else drawLineStrip(traj, 0.55f, 0.68f, 1f, 1f)                          // neutral

            // landmarks (4 floats each -> take xyz)
            if (s.landmarks.isNotEmpty()) {
                val pts = FloatArray(s.landmarks.size / 4 * 3)
                var j = 0
                var i = 0
                while (i + 2 < s.landmarks.size) { pts[j++] = s.landmarks[i]; pts[j++] = s.landmarks[i+1]; pts[j++] = s.landmarks[i+2]; i += 4 }
                drawPoints(pts, 0.7f, 0.7f, 0.72f, 1f, 4f)
            }
            // GNSS markers (only meaningful in ENU)
            if (showEnu && s.gnss.isNotEmpty()) drawPoints(s.gnss, 0.21f, 0.77f, 0.42f, 1f, 7f)

            // loop edges: connect keyframe a<->b using local trajectory order
            if (s.loopEdges.isNotEmpty() && s.trajLocal.size >= 3) {
                val verts = ArrayList<Float>()
                val nKf = s.trajLocal.size / 3
                var e = 0
                while (e + 1 < s.loopEdges.size) {
                    val a = s.loopEdges[e].toInt(); val b = s.loopEdges[e+1].toInt(); e += 2
                    // edges store keyframe indices; map to nearest trajectory slot
                    val ai = (a % nKf).coerceIn(0, nKf - 1); val bi = (b % nKf).coerceIn(0, nKf - 1)
                    for (k in 0..2) verts.add(s.trajLocal[ai*3+k])
                    for (k in 0..2) verts.add(s.trajLocal[bi*3+k])
                }
                drawLines(verts.toFloatArray(), 1f, 0.48f, 0.10f, 1f, GLES20.GL_LINES)
            }
        }

        // current pose frustum (accent)
        if (p != null && p.valid) drawFrustum(p)
    }

    // ---- draw helpers ----
    private fun buf(a: FloatArray): FloatBuffer =
        ByteBuffer.allocateDirect(a.size * 4).order(ByteOrder.nativeOrder())
            .asFloatBuffer().apply { put(a); position(0) }

    private fun drawArray(a: FloatArray, mode: Int, r: Float, g: Float, b: Float, al: Float, ps: Float) {
        if (a.size < 3) return
        val fb = buf(a)
        GLES20.glEnableVertexAttribArray(aPos)
        GLES20.glVertexAttribPointer(aPos, 3, GLES20.GL_FLOAT, false, 0, fb)
        GLES20.glUniform4f(uColor, r, g, b, al)
        GLES20.glUniform1f(uPointSize, ps)
        GLES20.glDrawArrays(mode, 0, a.size / 3)
        GLES20.glDisableVertexAttribArray(aPos)
    }
    private fun drawPoints(a: FloatArray, r: Float, g: Float, b: Float, al: Float, ps: Float) =
        drawArray(a, GLES20.GL_POINTS, r, g, b, al, ps)
    private fun drawLineStrip(a: FloatArray, r: Float, g: Float, b: Float, al: Float) =
        drawArray(a, GLES20.GL_LINE_STRIP, r, g, b, al, 1f)
    private fun drawLines(a: FloatArray, r: Float, g: Float, b: Float, al: Float, mode: Int) =
        drawArray(a, mode, r, g, b, al, 1f)

    private fun drawFrustum(p: PoseSnapshot) {
        // small camera frustum at the current pose, oriented by quaternion
        val q = p.q; val t = p.p
        val rot = quatToMat(q[0], q[1], q[2], q[3])
        val s = 0.3f
        val corners = arrayOf(
            floatArrayOf(0f, 0f, 0f),
            floatArrayOf(s, s, 2*s), floatArrayOf(-s, s, 2*s),
            floatArrayOf(-s, -s, 2*s), floatArrayOf(s, -s, 2*s)
        )
        val w = Array(corners.size) { i ->
            val c = corners[i]
            floatArrayOf(
                rot[0]*c[0]+rot[1]*c[1]+rot[2]*c[2]+t[0],
                rot[3]*c[0]+rot[4]*c[1]+rot[5]*c[2]+t[1],
                rot[6]*c[0]+rot[7]*c[1]+rot[8]*c[2]+t[2]
            )
        }
        val idx = intArrayOf(0,1, 0,2, 0,3, 0,4, 1,2, 2,3, 3,4, 4,1)
        val v = FloatArray(idx.size * 3)
        for (i in idx.indices) { v[i*3]=w[idx[i]][0]; v[i*3+1]=w[idx[i]][1]; v[i*3+2]=w[idx[i]][2] }
        drawLines(v, 1f, 0.48f, 0.10f, 1f, GLES20.GL_LINES)
    }

    private fun buildGrid(): FloatArray {
        val v = ArrayList<Float>(); val n = 10; val step = 1f
        for (i in -n..n) {
            v.add(i*step); v.add(-n*step); v.add(0f); v.add(i*step); v.add(n*step); v.add(0f)
            v.add(-n*step); v.add(i*step); v.add(0f); v.add(n*step); v.add(i*step); v.add(0f)
        }
        return v.toFloatArray()
    }

    private fun quatToMat(w: Float, x: Float, y: Float, z: Float): FloatArray {
        val n = Math.sqrt((w*w+x*x+y*y+z*z).toDouble()).toFloat().coerceAtLeast(1e-6f)
        val qw=w/n; val qx=x/n; val qy=y/n; val qz=z/n
        return floatArrayOf(
            1-2*(qy*qy+qz*qz), 2*(qx*qy-qz*qw),   2*(qx*qz+qy*qw),
            2*(qx*qy+qz*qw),   1-2*(qx*qx+qz*qz), 2*(qy*qz-qx*qw),
            2*(qx*qz-qy*qw),   2*(qy*qz+qx*qw),   1-2*(qx*qx+qy*qy)
        )
    }

    private fun compile(type: Int, src: String): Int {
        val s = GLES20.glCreateShader(type)
        GLES20.glShaderSource(s, src); GLES20.glCompileShader(s)
        return s
    }
}
