package org.firstinspires.ftc.teamcode.hardware.motion

import org.firstinspires.ftc.teamcode.util.Transform2D
import org.firstinspires.ftc.teamcode.util.Vec2Rot

/**
 * A class that allows the robot to know where it is at all times.
 *
 * Field-relative robot position follows the following conventions:
 *   +x
 */
abstract class IOdometry {

    open fun axesFunctional() = false

    /** Field-relative robot acceleration. */
    var acc = Vec2Rot.zero
        protected set

    /** Field-relative robot velocity. */
    var vel = Vec2Rot.zero
        protected set

    /** Field-relative robot position. */
    var pos = Vec2Rot.zero
        protected set

    /** Recalculate the Robot's position. */
    abstract fun tick(dt: Double)

    /** Assert the position of the Robot, resetting internal state as necessary. */
    abstract fun assertPosition(newPos: Vec2Rot)
    /** Assert the position, but assume it's close to the previous one, so attempt
     * to fix state to work with the new value. */
    abstract fun nudge(newPos: Vec2Rot)
    /** Optional function that allows odometry systems to report that they have no valid data. */
    open fun hasPositionInfo() = true

    data class Delta(val dPos: Vec2Rot, val dt: Double) {
        val vel get() = dPos / dt
    }

    /** Utility for converting between robot and global space, based on the current */
    fun robot2worldTransform() = Transform2D.local2outerFromLocation(pos)
}