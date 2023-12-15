package org.firstinspires.ftc.teamcode.robot.barthalomew

import org.firstinspires.ftc.teamcode.ftcGlue.IHardwareMap
import org.firstinspires.ftc.teamcode.ftcGlue.IRobot
import org.firstinspires.ftc.teamcode.hardware.Motor
import org.firstinspires.ftc.teamcode.hardware.motion.MecanumDrive
import org.firstinspires.ftc.teamcode.util.MM_TO_IN
import org.firstinspires.ftc.teamcode.util.Vec2Rot

class BarthalomewRobot : IRobot<BarthalomewRobot.Impl> {
    private val mecanum = MecanumDrive(
            Vec2Rot.zero,
            Motor.PhysicalSpec.GOBILDA_5202_0002_0005,
            MecanumDrive.ReversalPattern(right = true),
            wheelRadiusInches = 96.0 / 2 * MM_TO_IN,
            wheelDisplacementInches = 10.0, // this robot is disassembled irl so idk lol
            MecanumDrive.Ids.default,
    )

    inner class Impl(hardwareMap: IHardwareMap) {
        private val mecanum = this@BarthalomewRobot.mecanum.Impl(hardwareMap)

        fun drive(power: Vec2Rot) {
            mecanum.power = power
        }
    }

    override fun impl(hardwareMap: IHardwareMap) = Impl(hardwareMap)

    companion object {
        val main = BarthalomewRobot()
    }
}