package org.firstinspires.ftc.teamcode.robot

import org.firstinspires.ftc.teamcode.ftcGlue.IHardwareMap
import org.firstinspires.ftc.teamcode.ftcGlue.IRobot
import org.firstinspires.ftc.teamcode.hardware.Motor
import org.firstinspires.ftc.teamcode.hardware.commonAssembly.MecanumDrive
import org.firstinspires.ftc.teamcode.util.Vec2
import org.firstinspires.ftc.teamcode.util.Vec2Rot

class RobotA : IRobot<RobotA.Impl> {
    val drive = MecanumDrive(
            Motor.PhysicalSpec.GOBILDA_5202_0002_0005,
            MecanumDrive.ReversalPattern(
                    right = true
            ),
            MecanumDrive.Ids.default
    )

    inner class Impl(hardwareMap: IHardwareMap) {
        val drive = this@RobotA.drive.Impl(hardwareMap)

        fun init() {
            // could do something here, just as an example
        }

        fun doThing() {
            drive.power = Vec2Rot(Vec2(0.0,1.0),0.0)
        }
    }
    override fun impl(hardwareMap: IHardwareMap) = Impl(hardwareMap)

    companion object {
        val instance = RobotA()
    }
}