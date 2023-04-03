package org.firstinspires.ftc.teamcode.test.ftcGlue

import org.firstinspires.ftc.teamcode.ftcGlue.IGamepad
import org.firstinspires.ftc.teamcode.util.Vec2

class CIGamepad : IGamepad {
    override var x = false
    override var y = false
    override var a = false
    override var b = false
    override var dpadDown = false
    override var dpadUp = false
    override var dpadLeft = false
    override var dpadRight = false
    override var stickLeftButton = false
    override var stickLeft = Vec2(0.0,0.0)
    override var stickRightButton = false
    override var stickRight = Vec2(0.0,0.0)
    override var bumperLeft = false
    override var bumperRight = false
    override var triggerLeft = 0.0
    override var triggerRight = 0.0
}