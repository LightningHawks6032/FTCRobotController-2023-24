package org.firstinspires.ftc.teamcode.ftcGlue

// https://en.wikipedia.org/wiki/I,_Robot

interface IRobot<T: Any> {
    fun impl(hardwareMap: IHardwareMap): T
}