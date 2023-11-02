package org.firstinspires.ftc.teamcode.opmodes.hbot

import org.firstinspires.ftc.teamcode.controlSystems.DriveController
import org.firstinspires.ftc.teamcode.ftcGlue.IHardwareMap
import org.firstinspires.ftc.teamcode.ftcGlue.IRobot
import org.firstinspires.ftc.teamcode.hardware.Motor
import org.firstinspires.ftc.teamcode.hardware.motion.CompoundOdometry
import org.firstinspires.ftc.teamcode.hardware.motion.MecanumDrive
import org.firstinspires.ftc.teamcode.hardware.motion.ThreeWheelOdometry
import org.firstinspires.ftc.teamcode.util.Vec2
import org.firstinspires.ftc.teamcode.util.Vec2Rot
import org.firstinspires.ftc.teamcode.util.Vec3
import org.firstinspires.ftc.teamcode.vision.apriltag.AprilTagInfoBuilder
import org.firstinspires.ftc.teamcode.vision.apriltag.AprilTagTracking
import kotlin.math.PI

object HBotRobot : IRobot<HBotRobot.Impl> {
    private val mecanum = MecanumDrive(
            Vec2Rot.zero,
            Motor.PhysicalSpec.GOBILDA_5202_0002_0005,
            MecanumDrive.ReversalPattern(right = true),
            MecanumDrive.Ids(
                    fr = "fr", // front-right motor
                    fl = "fl", // front-left motor
                    br = "br", // back-right motor
                    bl = "bl", // back-left motor
            ),
    )

    const val MM_TO_IN = 0.03937008
    //
    private val threeWheelOdometry = ThreeWheelOdometry(
            Vec2Rot(Vec2(6.0,-2.0) * MM_TO_IN, 0.0),
            yReaderPos = 179.0 * MM_TO_IN,
            x0ReaderPos = 94.0 * MM_TO_IN,
            x1ReaderPos = -98.0 * MM_TO_IN,
            Motor.PhysicalSpec.GOBILDA_ODOMETRY_POD,
            1.75 / 2.0,
            ThreeWheelOdometry.ReversalPattern(
                    // TODO
            ),
            ThreeWheelOdometry.Ids(mecanum.ids.fr, mecanum.ids.fl, mecanum.ids.bl),
    )

    private val aprilTagTracking = AprilTagTracking(
            AprilTagInfoBuilder {
                addTagsCenterStage()
            },
            cameras = arrayOf(
                    // TODO measure
                    AprilTagTracking.CameraPlacement(Vec3(0.0, 6.0, 2.0), PI/2), // front cam : 0
                    AprilTagTracking.CameraPlacement(Vec3(0.0, -6.0, 2.0), -PI/2), // back cam : 1
            )
    )

    private val odometry = CompoundOdometry(
            hiPassOdo = threeWheelOdometry,
            loPassOdo = aprilTagTracking.odometry,
            0.5
    )


    class Impl(hardwareMap: IHardwareMap) {
        val aprilTagCam0 = aprilTagTracking.VisionImpl(0)
        val aprilTagCam1 = aprilTagTracking.VisionImpl(1)
        init {
            threeWheelOdometry.Impl(hardwareMap)
        }

        val drive = DriveController(mecanum.Impl(hardwareMap), odometry)

        fun updateAprilTag(impl: AprilTagTracking.VisionImpl) =
                impl.updateEstimates(odometry.pos)

        fun tickOdo(dt: Double) = odometry.tick(dt)
    }
    override fun impl(hardwareMap: IHardwareMap) = Impl(hardwareMap)
}