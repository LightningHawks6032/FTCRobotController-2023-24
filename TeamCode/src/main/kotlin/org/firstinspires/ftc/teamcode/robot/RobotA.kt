package org.firstinspires.ftc.teamcode.robot

import org.firstinspires.ftc.teamcode.ftcGlue.IHardwareMap
import org.firstinspires.ftc.teamcode.hardware.Motor

class RobotA {
    val motor = Motor("a")

    inner class Impl(hardwareMap: IHardwareMap) {
        val motor = this@RobotA.motor.Impl(hardwareMap)
    }
}