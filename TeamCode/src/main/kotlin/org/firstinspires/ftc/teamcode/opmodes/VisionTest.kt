package org.firstinspires.ftc.teamcode.opmodes

import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.LOpMode
import org.firstinspires.ftc.teamcode.ftcGlue.IHardwareMap
import org.firstinspires.ftc.teamcode.ftcGlue.IRobot
import org.firstinspires.ftc.teamcode.ftcGlue.WithTelemetry
import org.firstinspires.ftc.teamcode.util.Quaternion
import org.firstinspires.ftc.teamcode.util.UnstableUnfinished
import org.firstinspires.ftc.teamcode.util.Vec3
import org.firstinspires.ftc.teamcode.vision.Vision
import org.firstinspires.ftc.teamcode.vision.apriltag.AprilTagInfoBuilder
import org.firstinspires.ftc.teamcode.vision.apriltag.AprilTagTracking
import org.firstinspires.ftc.teamcode.vision.createVisionLoop
import org.firstinspires.ftc.teamcode.vision.tfod.TFObjectDetection
import org.firstinspires.ftc.vision.VisionPortal
import kotlin.math.roundToInt

val visionTestRobot = VisionTestRobot()

@TeleOp
class VisionTest : LOpMode<VisionTestRobot.Impl>(visionTestRobot, {
//    selectDebugBool("invertTagRot")
    var cam0Ready = false
    val aprilTagTelemetry = WithTelemetry.Partial()
    val tfodTelemetry = WithTelemetry.Partial()

    withTelemetry {
        ln("init began")
    }
    createLoop({ duringInit || duringRun }) {
        robot.aprilTag.odometry.tick(dt)

        withTelemetry {
            if (!cam0Ready) {
                ln("camera starting")
            } else if (duringInit) {
                ln("camera ready")
            } else if (duringRun) {
                +aprilTagTelemetry
                +tfodTelemetry
            }
        }
    }
    createVisionLoop(
            robot.vision,
            listOf(robot.aprilTagCam0, robot.tfodCam0),
            condition = { duringInit || duringRun },
    ) { vision ->
        cam0Ready = vision.cameraState == VisionPortal.CameraState.STREAMING
        if (!cam0Ready) CONTINUE
        if (duringRun) {
            robot.aprilTagCam0.decimation = 1.0

            // Get new detections, if none, wait.
            val detections = robot.aprilTagCam0.updateEstimates(robot.aprilTag.odometry.pos)
            if (detections != null) {
                // Report
                aprilTagTelemetry {
                    ln("cam0 avg solve time: ${robot.aprilTagCam0.perTagAvgPoseSolveTime}ms")
                    if (detections.isEmpty()) {
                        ln("no detections")
                    }
                    for (detection in detections) {
                        ln("-- tag #${detection.id} ---------")

                    }
                    ln("x: ${robot.aprilTag.odometry.pos.v.x}")
                    ln("y: ${robot.aprilTag.odometry.pos.v.y}")
                    ln("r: ${robot.aprilTag.odometry.pos.r}")
                    ln("-------------------")
                }
                @OptIn(UnstableUnfinished::class)
                for (tagLoc in robot.aprilTag.tagPositionEstimates.values) {
                    println("${tagLoc.id} :: ${tagLoc.pos}")
                }
            }

            // Get Tensorflow Recognitions, if none, wait.
            val recognitions = robot.tfodCam0.freshRecognitions
            if (recognitions != null) {
                // Report our findings
                tfodTelemetry {
                    ln("tensorflow? yes. (${recognitions.size}}")
                    for (recognition in recognitions) {
                        ln("${recognition.label}: (${
                            recognition.let { ((it.left + it.right)/2).roundToInt() }
                        }, ${
                            recognition.let { ((it.bottom + it.top)/2).roundToInt() }
                        })")
                    }
                }
            }
        } else {
            robot.aprilTagCam0.decimation = 10.0
        }
    }
    waitForStart()
})

class VisionTestRobot : IRobot<VisionTestRobot.Impl> {
    val vision = Vision("Webcam 1")
    val tfod = TFObjectDetection()
    val aprilTag = AprilTagTracking(
            AprilTagInfoBuilder {
                addTagsCenterStage()
                addForRobotPos(
                        584,
                        "test",
                        4.0,
                        Vec3(0.0, 0.0, 0.0),
                        Quaternion(1.0, 0.0, 0.0, 0.0),
                )
//                addForRobotPos(584, "test", 4.0, Vec3(10.0, 10.0, 10.0), Quaternion(1.0, 0.0, 0.0, 0.0))
                addForRobotPos(
                        583,
                        "test2",
                        4.0,
                        Vec3(0.0, 0.0, 0.0),
                        Quaternion(0.707, 0.0, 0.0, 0.707),
                )
//                addForRobotPos( 583, "test2", 4.0, Vec3(10.0, 10.0, 0.0), Quaternion(0.707, 0.0, 0.0, 0.707))
                addForTagPos(585, "loc", 6.0)
            },
            cameras = arrayOf(
                    AprilTagTracking.CameraPlacement(Vec3(0.0, 0.0, 0.0), 0.0)
            )
    )

    inner class Impl(hardwareMap: IHardwareMap) {
        val vision = this@VisionTestRobot.vision.Impl(hardwareMap)
        val tfodCam0 = tfod.VisionImpl()
        val aprilTag by this@VisionTestRobot::aprilTag // forward reference through inner class
        val aprilTagCam0 = aprilTag.VisionImpl(0)
    }

    override fun impl(hardwareMap: IHardwareMap) = Impl(hardwareMap)
}