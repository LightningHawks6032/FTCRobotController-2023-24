package org.firstinspires.ftc.teamcode.robot

import org.firstinspires.ftc.teamcode.ftcGlue.IHardwareMap
import org.firstinspires.ftc.teamcode.ftcGlue.IRobot
import org.firstinspires.ftc.teamcode.hardware.Motor

class RobotA : IRobot<RobotA.Impl> {
    val motor = Motor("a")

    inner class Impl(hardwareMap: IHardwareMap) {
        val motor = this@RobotA.motor.Impl(hardwareMap)

        fun init() {
            // could do something here, just as an example
        }

        fun doThing() {
            motor.power = 1.0
        }
    }
    override fun impl(hardwareMap: IHardwareMap) = Impl(hardwareMap)

    companion object {
        val instance = RobotA()
    }
}