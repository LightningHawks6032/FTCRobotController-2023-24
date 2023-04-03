package org.firstinspires.ftc.teamcode.util

data class Vec2(
        val x: Double,
        val y: Double,
) {
    operator fun plus(other: Vec2) = Vec2(this.x + other.x, this.y + other.y)
    operator fun minus(other: Vec2) = Vec2(this.x - other.x, this.y - other.y)
    operator fun times(other: Double) = Vec2(this.x * other, this.y * other)
    operator fun div(other: Double) = this * (1.0/other)
    infix fun dot(other: Vec2) = this.x * other.x + this.y * other.y
}