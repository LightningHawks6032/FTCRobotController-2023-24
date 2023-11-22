package org.firstinspires.ftc.teamcode.robot.arbot

import org.firstinspires.ftc.teamcode.LOpMode
import org.firstinspires.ftc.teamcode.controlSystems.DriveController
import org.firstinspires.ftc.teamcode.controlSystems.PID1D
import org.firstinspires.ftc.teamcode.ftcGlue.IHardwareMap
import org.firstinspires.ftc.teamcode.ftcGlue.IRobot
import org.firstinspires.ftc.teamcode.hardware.Motor
import org.firstinspires.ftc.teamcode.hardware.motion.CompoundOdometry
import org.firstinspires.ftc.teamcode.hardware.motion.MecanumDrive
import org.firstinspires.ftc.teamcode.hardware.motion.ThreeWheelOdometry
import org.firstinspires.ftc.teamcode.robot.arbot.subassemblies.ArBotIntake
import org.firstinspires.ftc.teamcode.robot.arbot.subassemblies.ArBotOuttake
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

 KEY: `CH[3]` &rarr; Control Hub Port 3, `EH[0]` &rarr; Expansion Hub Port 0

DCMotor Mappings:
- "fr" (`CH[2]`): front right drive motor, front odometry wheel
- "fl" (`CH[3]`): front left drive motor
- "br" (`CH[1]`): back right drive motor, right odometry wheel
- "bl" (`CH[0]`): back left drive motor, left odometry wheel

- "ar": (`EH[0]`) right intake arm motor
- "al": (`EH[1]`) left intake arm motor

- "l0": (`EH[2]`) out take lifter linear slide motor
- "l1": (`EH[3]`) out take lifter linear slide motor (the other one)

Servo Mappings:
- "ir" (`CH[0]`): intake input servo, right
- "il" (`CH[1]`): intake input servo, left
- "tr" (`CH[2]`): intake transfer/mid-take servo
- "or" (`CH[4]`): outtake dropper servo, right
- "ol" (`CH[5]`): outtake dropper servo, left

 */
object ArBotRobot : IRobot<ArBotRobot.Impl> {
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
            Vec2Rot((Vec2(6.0 - 115.0, -2.0) * MM_TO_IN).rotate(PI / 2), PI / 2), // yReaderPos is in front
            yReaderPos = (179.0 + 115.0) * MM_TO_IN,
            x0ReaderPos = 94.0 * MM_TO_IN,
            x1ReaderPos = -98.0 * MM_TO_IN,
            Motor.PhysicalSpec.GOBILDA_ODOMETRY_POD,
            1.75 / 2.0,
            ThreeWheelOdometry.ReversalPattern( // 115
                    // TODO
                    y = false,
                    x0 = false,
                    x1 = true,
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

    private val intakeRef = ArBotIntake(
            true,
            PID1D.Coefficients(
                    P = 1.0,
                    I = 0.2,
                    D = 0.5,
                    iDecay = 4.0,
                    biasSlope = 10.0,
                    bias = 0.2,
            )
    )
    private val outtakeRef = ArBotOuttake(
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