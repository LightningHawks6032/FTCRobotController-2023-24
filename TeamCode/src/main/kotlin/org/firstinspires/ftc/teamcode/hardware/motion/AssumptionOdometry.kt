package org.firstinspires.ftc.teamcode.hardware.motion

import org.firstinspires.ftc.teamcode.util.Vec2Rot

/** A testing tool for guessing the robot's position based on drive commands alone. */
class AssumptionOdometry : IOdometry() {
    var drive: IDrive? = null

    override fun assertPosition(newPos: Vec2Rot) {
        // no-op
    }

    override fun nudge(newPos: Vec2Rot) {
        // no-op
    }

    override fun tick(dt: Double) {
        val driveRn = drive
        if (driveRn != null) {
            acc = driveRn.power.transformP { it.rotate(pos.r) } * 50.0 - vel * 0.1
            vel += acc * dt
            pos += vel * dt
        }
    }
}