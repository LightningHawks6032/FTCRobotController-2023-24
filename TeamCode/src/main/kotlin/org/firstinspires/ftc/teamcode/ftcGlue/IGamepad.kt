package org.firstinspires.ftc.teamcode.ftcGlue

import org.firstinspires.ftc.teamcode.util.Vec2

interface IGamepad {

    val x: Boolean
    val y: Boolean
    val a: Boolean
    val b: Boolean

    val dpadDown: Boolean
    val dpadUp: Boolean
    val dpadLeft: Boolean
    val dpadRight: Boolean

    val stickLeftButton: Boolean
    val stickLeft: Vec2
    val stickRightButton: Boolean
    val stickRight: Vec2

    val bumperLeft: Boolean
    val bumperRight: Boolean

    val triggerLeft: Double
    val triggerRight: Double
}