package org.firstinspires.ftc.teamcode.util

import org.firstinspires.ftc.robotcore.external.matrices.MatrixF
import java.util.*
import kotlin.math.cos
import kotlin.math.sin
import kotlin.math.sqrt

data class Quaternion constructor(val w: Double, val x: Double, val y: Double, val z: Double) {

    val magnitude get() = sqrt(w * w + x * x + y * y + z * z)
    val normalized get() = this * (1.0 / magnitude)
    val conjugate get() = Quaternion(w, -x, -y, -z)
    val inverse get() = normalized.conjugate

    operator fun times(scalar: Double) = Quaternion(w * scalar, x * scalar, y * scalar, z * scalar)
    operator fun times(q: Quaternion) = Quaternion(
            w * q.w - x * q.x - y * q.y - z * q.z,
            w * q.x + x * q.w + y * q.z - z * q.y,
            w * q.y - x * q.z + y * q.w + z * q.x,
            w * q.z + x * q.y - y * q.x + z * q.w
    )

    operator fun plus(q: Quaternion) = Quaternion(
            w + q.w, x + q.x, y + q.y, z + q.z
    )

    fun apply(v: Vec3) = Quaternion(0.0, v.x, v.y, v.z).let {
        this * it * inverse
    }.let {
        Vec3(it.x, it.y, it.z)
    }

    override fun toString(): String {
        return String.format(Locale.US, "{w=%.3f, x=%.3f, y=%.3f, z=%.3f}", w, x, y, z)
    }

    companion object {
        val identity = Quaternion(1.0, 0.0, 0.0, 0.0)

        fun fromAngleAxis(angle: Double, axis: Vec3): Quaternion {
            val v = axis / axis.mag * sin(angle * 2)
            return Quaternion(cos(angle * 2), v.x, v.y, v.z)
        }

        // Adapted from FTCRobotController
        fun fromMatrix(m: MatrixF): Quaternion {
            // https://www.euclideanspace.com/maths/geometry/rotations/conversions/matrixToQuaternion/
            val tr = m[0, 0] + m[1, 1] + m[2, 2]
            val w: Double
            val x: Double
            val y: Double
            val z: Double
            if (tr > 0) {
                val s = sqrt(tr + 1.0) * 2 // S=4*w
                w = 0.25 * s
                x = (m[2, 1] - m[1, 2]) / s
                y = (m[0, 2] - m[2, 0]) / s
                z = (m[1, 0] - m[0, 1]) / s
            } else if ((m[0, 0] > m[1, 1]) and (m[0, 0] > m[2, 2])) {
                val s = sqrt(1.0 + m[0, 0] - m[1, 1] - m[2, 2]) * 2 // S=4*x
                w = (m[2, 1] - m[1, 2]) / s
                x = 0.25 * s
                y = (m[0, 1] + m[1, 0]) / s
                z = (m[0, 2] + m[2, 0]) / s
            } else if (m[1, 1] > m[2, 2]) {
                val s = sqrt(1.0 + m[1, 1] - m[0, 0] - m[2, 2]) * 2 // S=4*y
                w = (m[0, 2] - m[2, 0]) / s
                x = (m[0, 1] + m[1, 0]) / s
                y = 0.25 * s
                z = (m[1, 2] + m[2, 1]) / s
            } else {
                val s = sqrt(1.0 + m[2, 2] - m[0, 0] - m[1, 1]) * 2 // S=4*z
                w = (m[1, 0] - m[0, 1]) / s
                x = (m[0, 2] + m[2, 0]) / s
                y = (m[1, 2] + m[2, 1]) / s
                z = 0.25 * s
            }
            return Quaternion(w, x, y, z).normalized
        }
    }
}

fun org.firstinspires.ftc.robotcore.external.navigation.Quaternion.convert() =
        Quaternion(w.toDouble(), x.toDouble(), y.toDouble(), z.toDouble())

fun Quaternion.toFTC() =
        org.firstinspires.ftc.robotcore.external.navigation.Quaternion(w.toFloat(), x.toFloat(), y.toFloat(), z.toFloat(), 0)
