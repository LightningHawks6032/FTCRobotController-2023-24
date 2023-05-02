package org.firstinspires.ftc.teamcode.hardware.motion

import org.firstinspires.ftc.teamcode.util.Vec2Rot

abstract class Odometry {

    var vel = Vec2Rot.zero
        private set
    var pos = Vec2Rot.zero

    fun tick(dt: Double) {
        if (dt == 0.0) return

        val delta = readDeltaFromImpl(dt)

        vel = delta?.vel ?: Vec2Rot.zero
        pos += delta?.dPos ?: Vec2Rot.zero
    }

    protected abstract fun readDeltaFromImpl(dt: Double): Delta?

    data class Delta(val dPos: Vec2Rot, val dt: Double) {
        val vel get() = dPos / dt
    }
}