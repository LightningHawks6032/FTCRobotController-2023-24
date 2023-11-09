package org.firstinspires.ftc.teamcode.robot.hbot

import org.firstinspires.ftc.teamcode.LOpMode
import org.firstinspires.ftc.teamcode.controlSystems.DriveController
import org.firstinspires.ftc.teamcode.controlSystems.PID1D
import org.firstinspires.ftc.teamcode.ftcGlue.IHardwareMap
import org.firstinspires.ftc.teamcode.ftcGlue.IRobot
import org.firstinspires.ftc.teamcode.hardware.Motor
import org.firstinspires.ftc.teamcode.hardware.motion.CompoundOdometry
import org.firstinspires.ftc.teamcode.hardware.motion.MecanumDrive
import org.firstinspires.ftc.teamcode.hardware.motion.ThreeWheelOdometry
import org.firstinspires.ftc.teamcode.robot.hbot.subassemblies.HBotIntake
import org.firstinspires.ftc.teamcode.robot.hbot.subassemblies.HBotOuttake
import org.firstinspires.ftc.teamcode.util.MM_TO_IN
import org.firstinspires.ftc.teamcode.util.Vec2
import org.firstinspires.ftc.teamcode.util.Vec2Rot
import org.firstinspires.ftc.teamcode.util.Vec3
import org.firstinspires.ftc.teamcode.vision.Vision
import org.firstinspires.ftc.teamcode.vision.apriltag.AprilTagInfoBuilder
import org.firstinspires.ftc.teamcode.vision.apriltag.AprilTagTracking
import org.firstinspires.ftc.teamcode.vision.createVisionLoop
import kotlin.math.PI

/**

DCMotor Mappings:
- "fr": front right drive motor, front odometry wheel
- "fl": front left drive motor
- "br": back right drive motor, right odometry wheel
- "bl": back left drive motor, left odometry wheel

- "ar": right intake arm motor
- "al": left intake arm motor

- "l0": out take lifter linear slide motor
- "l1": out take lifter linear slide motor (the other one)

Servo Mappings:
TODO TBD

 */
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
    private val threeWheelOdometry = ThreeWheelOdometry(
            Vec2Rot(Vec2(6.0, -2.0) * MM_TO_IN, PI / 2), // yReaderPos is in front
            yReaderPos = 179.0 * MM_TO_IN,
            x0ReaderPos = 94.0 * MM_TO_IN,
            x1ReaderPos = -98.0 * MM_TO_IN,
            Motor.PhysicalSpec.GOBILDA_ODOMETRY_POD,
            1.75 / 2.0,
            ThreeWheelOdometry.ReversalPattern(
                    // TODO
            ),
            ThreeWheelOdometry.Ids(mecanum.ids.fr, mecanum.ids.bl, mecanum.ids.br),
    )

    private val aprilTagTracking = AprilTagTracking(
            AprilTagInfoBuilder {
                addTagsCenterStage()
            },
            cameras = arrayOf(
                    // TODO measure
                    AprilTagTracking.CameraPlacement(Vec3(0.0, 6.0, 2.0), PI / 2), // front cam : 0
                    AprilTagTracking.CameraPlacement(Vec3(0.0, -6.0, 2.0), -PI / 2), // back cam : 1
            )
    )

    private val odometry = CompoundOdometry(
            hiPassOdo = threeWheelOdometry,
            loPassOdo = aprilTagTracking.odometry,
            0.5
    )

    private val visionCam0 = Vision("cam front")
    private val visionCam1 = Vision("cam back")

    private val intakeRef = HBotIntake(
            false,
            PID1D.Coefficients(
                    P = 1.0,
                    I = 0.2,
                    D = 0.5,
                    iDecay = 4.0,
                    biasSlope = 10.0,
                    bias = 0.2,
            )
    )
    private val outtakeRef = HBotOuttake(
            false,
            PID1D.Coefficients(
                    P = 1.0,
                    I = 0.2,
                    D = 0.5,
                    iDecay = 4.0,
                    biasSlope = 10.0,
                    bias = 0.2,
            )
    )


    class Impl(hardwareMap: IHardwareMap) {
        // lazy load camera stuff because it's really resource heavy and we may not want to use it in TeleOp
        private val cam0 by lazy { visionCam0.Impl(hardwareMap) }
        private val cam1 by lazy { visionCam1.Impl(hardwareMap) }
        val aprilTagCam0 by lazy { aprilTagTracking.VisionImpl(0) }
        val aprilTagCam1 by lazy { aprilTagTracking.VisionImpl(1) }

        val drive = run {
            threeWheelOdometry.Impl(hardwareMap)
            DriveController(mecanum.Impl(hardwareMap), odometry)
        }

        val intake = intakeRef.Impl(hardwareMap)
        val outtake = outtakeRef.Impl(hardwareMap)

        suspend fun createVisionLoops(
                scope: LOpMode<Impl>.RunScope,
                condition: () -> Boolean = { scope.duringRun },
        ) {
            for ((cam, aprilTag) in listOf(
                    Pair(cam0, aprilTagCam0),
                    Pair(cam1, aprilTagCam1),
            )) {
                scope.createVisionLoop(cam, listOf(aprilTag), condition = condition) {
                    aprilTag.updateEstimates(odometry.pos)
                }
            }
        }
    }

    override fun impl(hardwareMap: IHardwareMap) = Impl(hardwareMap)
}