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
    val encoder = Motor(
            "enc",
            Motor.PhysicalSpec.REV_THROUGH_BORE_ENCODER,
            Motor.Config.default
    )

    inner class Impl(hardwareMap: IHardwareMap) {
        val drive = this@RobotA.drive.Impl(hardwareMap)
        val encoder = this@RobotA.encoder.Impl(hardwareMap)

        fun init() {
            // could do something here, just as an example
        }

        fun doThing() {
            drive.power = Vec2Rot(Vec2(0.0,1.0), -encoder.pos)
        }
    }
    override fun impl(hardwareMap: IHardwareMap) = Impl(hardwareMap)

    companion object {
        val instance = RobotA()
    }
}