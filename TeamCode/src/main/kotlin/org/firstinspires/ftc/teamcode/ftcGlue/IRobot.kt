package org.firstinspires.ftc.teamcode.ftcGlue

interface IRobot<T: Any> {
    fun impl(hardwareMap: IHardwareMap): T
}