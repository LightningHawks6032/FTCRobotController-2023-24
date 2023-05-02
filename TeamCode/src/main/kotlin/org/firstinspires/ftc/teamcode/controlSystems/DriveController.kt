package org.firstinspires.ftc.teamcode.controlSystems

import org.firstinspires.ftc.teamcode.hardware.motion.DriveImpl
import org.firstinspires.ftc.teamcode.hardware.motion.Odometry
import org.firstinspires.ftc.teamcode.util.Vec2Rot
import kotlin.math.max

class DriveController(
        private val output: DriveImpl,
        private val input: Odometry,
) {

    var currentTarget = Vec2Rot.zero

    fun tick(dt: Double) {
        input.tick(dt)

        // TODO: actual controls lol
        val pow = (currentTarget - input.pos).transformP { it.rotate(-input.pos.r) }
        output.power = pow / max(1.0, pow.v.mag)
    }
}