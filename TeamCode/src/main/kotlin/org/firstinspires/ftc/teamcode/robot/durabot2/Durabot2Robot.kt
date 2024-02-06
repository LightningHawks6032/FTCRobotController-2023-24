package org.firstinspires.ftc.teamcode.robot.durabot2

import org.firstinspires.ftc.teamcode.controlSystems.DriveController
import org.firstinspires.ftc.teamcode.controlSystems.PID1D
import org.firstinspires.ftc.teamcode.ftcGlue.IHardwareMap
import org.firstinspires.ftc.teamcode.ftcGlue.IRobot
import org.firstinspires.ftc.teamcode.hardware.Motor
import org.firstinspires.ftc.teamcode.hardware.motion.MecanumDrive
import org.firstinspires.ftc.teamcode.hardware.motion.MecanumWheelOdometry
import org.firstinspires.ftc.teamcode.util.MM_TO_IN
import org.firstinspires.ftc.teamcode.util.Vec2Rot
import kotlin.math.PI

object Durabot2Robot : IRobot<Durabot2Robot.Impl> {
    val mecanumSpec = MecanumDrive(
            Vec2Rot.zero,
            Motor.PhysicalSpec.GOBILDA_5202_0002_0005,
            MecanumDrive.ReversalPattern(left = true),
            96.0 / 2.0 * MM_TO_IN,
            165.0 * MM_TO_IN,
            MecanumDrive.Ids("fr", "fl", "br", "bl"),
    )
    val odometry = MecanumWheelOdometry(mecanumSpec, 0.12 * PI)

    override fun impl(hardwareMap: IHardwareMap) = Impl(hardwareMap)
    class Impl(hardwareMap: IHardwareMap) {
        private val mecanum by lazy { mecanumSpec.Impl(hardwareMap) }
        private val odometry by lazy {
            Durabot2Robot.odometry.also { it.Impl(hardwareMap) }
        }
        val drive by lazy {
            DriveController(
                    mecanum, odometry,
                    PID1D.Coefficients(
                            P = 0.5,
                            I = 0.3,
                            D = 0.3,
                            iDecay = 1.0,
                            bias = 0.05,
                            biasSlope = 1.5,
                    ),
                    PID1D.Coefficients(
                            P = 0.5,
                            I = 0.4,
                            D = 0.3,
                            iDecay = 0.5,
                            bias = 0.05,
                            biasSlope = 1.5,
                    ),
                    Vec2Rot(7.0, 7.0, 8.0),
            )
        }
    }

    const val OP_GROUP_NAME = "durabot"
}