package org.firstinspires.ftc.teamcode.util

import kotlin.math.cos
import kotlin.math.sin

data class Vec2(
        val x: Double,
        val y: Double,
) {
    operator fun plus(other: Vec2) = Vec2(this.x + other.x, this.y + other.y)
    operator fun minus(other: Vec2) = this + -other
    operator fun times(k: Double) = Vec2(x * k, y * k)
    operator fun unaryMinus() = this * -1.0
    operator fun div(other: Double) = this * (1.0 / other)
    /** Take the dot product between `this` and `other`. */
    infix fun dot(other: Vec2) = this.x * other.x + this.y * other.y

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
        val zero = Vec2(0.0,0.0)
    }
}

data class Vec2Rot(
        val v: Vec2,
        /** Rotation in radians, positive is counter-clockwise */
        val r: Double,
) {
    operator fun plus(other: Vec2Rot) = Vec2Rot(this.v + other.v, this.r + other.r)
    operator fun minus(other: Vec2Rot) = this + -other
    operator fun times(k: Double) = Vec2Rot(v * k, r * k)
    operator fun unaryMinus() = this * -1.0
    operator fun div(other: Double) = this * (1.0 / other)
    inline fun transformP(block: (Vec2) -> Vec2) = Vec2Rot(block(v), r)

    companion object {
        val zero = Vec2Rot(Vec2.zero, 0.0)
    }
}


fun k() {
    val a = Vec2Rot(Vec2(0.0,0.0),0.0)

    a.transformP { it.rotate(1.0) }
}