package org.firstinspires.ftc.teamcode.hardware.motion

import org.firstinspires.ftc.teamcode.util.Vec2Rot

interface IDrive {
    /** Desired motor power in robot space */
    var power: Vec2Rot
    /** Driving motor force in newtons in robot local space. */
    fun setForce(force: Vec2Rot, currentVel: Vec2Rot)
}