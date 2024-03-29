package org.firstinspires.ftc.teamcode.util

import org.firstinspires.ftc.robotcore.external.matrices.VectorF
import kotlin.math.cos
import kotlin.math.sin
import kotlin.math.sqrt

data class Vec2(
        val x: Double,
        val y: Double,
) {
    operator fun plus(other: Vec2) = Vec2(this.x + other.x, this.y + other.y)
    operator fun minus(other: Vec2) = this + -other
    operator fun times(k: Double) = Vec2(x * k, y * k)
    operator fun times(other: Vec2) = Vec2(this.x * other.x, this.y * other.y)
    operator fun unaryMinus() = this * -1.0
    operator fun div(other: Double) = this * (1.0 / other)

    /** Take the dot product between `this` and `other`. */
    infix fun dot(other: Vec2) = this.x * other.x + this.y * other.y

    /** Take the "cross product" between `this` and `other`, which isn't technically a cross product,
     * but it does basically the same thing in a sense. */
    infix fun fakeCrossProduct(other: Vec2) = this.x * other.y - this.y * other.x

    val magSq get() = this dot this
    val mag get() = sqrt(magSq)
    val norm
        get() = if (magSq < 1e-10) {
            zero
        } else {
            this / mag
        }

    /**
     * Return a rotated `Vec2`.
     *
     * Angles are in Radians, positive is counter-clockwise
     */
    fun rotate(angle: Double) =
            Vec2(
                    x * cos(angle) - y * sin(angle),
                    x * sin(angle) + y * cos(angle),
            )

    companion object {
        val zero = Vec2(0.0, 0.0)
        val X = Vec2(1.0, 0.0)
        val Y = Vec2(0.0, 1.0)

        fun grad(f: (Vec2) -> Double, pos: Vec2, dr: Double = 0.1) = Vec2(
                (f(pos + X * dr / 2.0) - f(pos - X * dr / 2.0)) / dr * 2,
                (f(pos + Y * dr / 2.0) - f(pos - Y * dr / 2.0)) / dr,
        )
    }
}

/**
 * Represents a location in the world.
 *
 * Position coordinate system:
 * - (0,0) -> center of the field
 * - +x -> towards red alliance human area
 * - +y -> 90deg ccw of that
 * - +z -> up
 *
 * Rotations:
 * - 0 -> facing forward to +x
 * - counter clockwise
 *
 */
data class Vec2Rot(
        val v: Vec2,
        /** Rotation in radians, positive is counter-clockwise */
        val r: Double,
) {
    constructor(
            x: Double,
            y: Double,
            /** Rotation in radians, positive is counter-clockwise */
            r: Double,
    ) : this(Vec2(x, y), r)

    operator fun plus(other: Vec2Rot) = Vec2Rot(this.v + other.v, this.r + other.r)
    operator fun minus(other: Vec2Rot) = this + -other
    operator fun times(k: Double) = Vec2Rot(v * k, r * k)
    operator fun unaryMinus() = this * -1.0
    operator fun div(other: Double) = this * (1.0 / other)
    inline fun transformP(block: (Vec2) -> Vec2) = Vec2Rot(block(v), r)

    infix fun componentwiseTimes(other: Vec2Rot) =
            Vec2Rot(v.x * other.v.x, v.y * other.v.y, r * other.r)

    fun clampComponents(maxMag: Double) =
            Vec2Rot(v.x.clamp(-maxMag, maxMag), v.y.clamp(-maxMag, maxMag), r.clamp(-maxMag, maxMag))

    companion object {
        val zero = Vec2Rot(Vec2.zero, 0.0)
    }
}


data class Vec3(
        val x: Double,
        val y: Double,
        val z: Double,
) {
    operator fun plus(other: Vec3) = Vec3(this.x + other.x, this.y + other.y, this.z + other.z)
    operator fun minus(other: Vec3) = this + -other
    operator fun times(k: Double) = Vec3(x * k, y * k, z * k)
    operator fun times(other: Vec3) = Vec3(this.x * other.x, this.y * other.y, this.z * other.z)
    operator fun unaryMinus() = this * -1.0
    operator fun div(other: Double) = this * (1.0 / other)

    /** Take the dot product between `this` and `other`. */
    infix fun dot(other: Vec3) = this.x * other.x + this.y * other.y + this.z * other.z

    val magSq get() = this dot this
    val mag get() = sqrt(magSq)

    /**
     * Return a [Vec3] rotated around the Z-axis.
     *
     * Angles are in Radians, positive is counter-clockwise viewed from above.
     */
    fun rotateZ(angle: Double) =
            Vec3(
                    x * cos(angle) - y * sin(angle),
                    x * sin(angle) + y * cos(angle),
                    z,
            )

    /**
     * Return a [Vec3] that has been rotated by the given quaternion.
     */
    fun rotateByQuaternion(q: Quaternion) = q.apply(this)

    companion object {
        val zero = Vec3(0.0, 0.0, 0.0)
        val zVec = Vec3(0.0, 0.0, 1.0)
    }
}

fun VectorF.toVec3() = Vec3(this[0].toDouble(), this[1].toDouble(), this[2].toDouble())
fun Vec3.toVectorF() = VectorF(this.x.toFloat(), this.y.toFloat(), this.z.toFloat())
