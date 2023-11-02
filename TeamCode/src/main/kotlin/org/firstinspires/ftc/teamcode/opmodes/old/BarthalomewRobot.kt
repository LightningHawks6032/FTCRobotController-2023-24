package org.firstinspires.ftc.teamcode.opmodes.old

import org.firstinspires.ftc.teamcode.ftcGlue.IHardwareMap
import org.firstinspires.ftc.teamcode.ftcGlue.IRobot
import org.firstinspires.ftc.teamcode.hardware.Motor
import org.firstinspires.ftc.teamcode.hardware.motion.MecanumDrive
import org.firstinspires.ftc.teamcode.util.Vec2Rot

class BarthalomewRobot : IRobot<BarthalomewRobot.Impl> {
    private val mecanum = MecanumDrive(
            Vec2Rot.zero,
            Motor.PhysicalSpec.GOBILDA_5202_0002_0005,
            MecanumDrive.ReversalPattern(right = true),
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