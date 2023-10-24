package org.firstinspires.ftc.teamcode.opmodes

import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.LOpMode
import org.firstinspires.ftc.teamcode.ftcGlue.IHardwareMap
import org.firstinspires.ftc.teamcode.ftcGlue.IRobot
import org.firstinspires.ftc.teamcode.util.Quaternion
import org.firstinspires.ftc.teamcode.util.UnstableUnfinished
import org.firstinspires.ftc.teamcode.util.Vec3
import org.firstinspires.ftc.teamcode.vision.Vision
import org.firstinspires.ftc.teamcode.vision.apriltag.AprilTagInfoBuilder
import org.firstinspires.ftc.teamcode.vision.apriltag.AprilTagTracking
import org.firstinspires.ftc.teamcode.vision.apriltag.VisAprilTag
import org.firstinspires.ftc.teamcode.vision.createVisionLoop
import org.firstinspires.ftc.vision.VisionPortal

val visionTestRobot = VisionTestRobot()

@TeleOp
class VisionTest : LOpMode<VisionTestRobot.Impl>(visionTestRobot, {
//    selectDebugBool("invertTagRot")
    withTelemetry {
        ln("init began")
    }
    createLoop({ duringInit || duringRun }) {
        robot.aprilTagOdometry.tick(dt)
    }
    createVisionLoop(
            robot.vision,
            listOf(robot.visAprilTag),
            condition = { duringInit || duringRun },
    ) { vision ->
        if (vision.cameraState != VisionPortal.CameraState.STREAMING) {
            withTelemetry {
                ln("camera starting")
            }
            CONTINUE
        }
        if (duringRun) {
            robot.visAprilTag.decimation = 1.0

            // Get new detections, if none, wait.
            val detections = robot.visAprilTag.freshDetections ?: CONTINUE
            robot.aprilTagTracking.updateEstimates(0, detections, robot.aprilTagOdometry.pos)
            // Report
            withTelemetry {
                if (detections.isEmpty()) {
                    ln("no detections")
                }
                for (detection in detections) {
                    ln("-- tag #${detection.id} ---------")

                }
                ln("x: ${robot.aprilTagOdometry.pos.v.x}")
                ln("y: ${robot.aprilTagOdometry.pos.v.y}")
                ln("r: ${robot.aprilTagOdometry.pos.r}")
                ln("-------------------")
            }
            @OptIn(UnstableUnfinished::class)
            for (tagLoc in robot.aprilTagTracking.tagPositionEstimates.values) {
                println("${tagLoc.id} :: ${tagLoc.pos}")
            }
        } else {
            robot.visAprilTag.decimation = 10.0
            withTelemetry {
                ln("camera online")
            }
        }
    }
    waitForStart()
})

class VisionTestRobot : IRobot<VisionTestRobot.Impl> {
    val vision = Vision("Webcam 1")
    private val aprilTagInfo = AprilTagInfoBuilder {
        addTagsCenterStage()
        addForRobotPos(
                584,
                "test",
                4.0,
                Vec3(0.0, 0.0, 0.0),
                Quaternion(1.0, 0.0, 0.0, 0.0),
        )
//        addForRobotPos(
//                584,
//                "test",
//                4.0,
//                Vec3(10.0, 10.0, 10.0),
//                Quaternion(1.0, 0.0, 0.0, 0.0),
//        )
        addForRobotPos(
                583,
                "test2",
                4.0,
                Vec3(0.0, 0.0, 0.0),
                Quaternion(0.707, 0.0, 0.0, 0.707),
        )
//        addForRobotPos(
//                583,
//                "test2",
//                4.0,
//                Vec3(10.0, 10.0, 0.0),
//                Quaternion(0.707, 0.0, 0.0, 0.707),
//        )
        addForTagPos(585, "loc", 6.0)
    }
    val visAprilTag = VisAprilTag(aprilTagInfo)
    val aprilTagTracking = AprilTagTracking(
            aprilTagInfo,
            AprilTagTracking.CameraPlacement(Vec3(0.0, 0.0, 0.0), 0.0)
    )
    val aprilTagOdometry = aprilTagTracking.Odometry()

    inner class Impl(hardwareMap: IHardwareMap) {
        val vision = this@VisionTestRobot.vision.Impl(hardwareMap)
        val visAprilTag = this@VisionTestRobot.visAprilTag.Instance()
        val aprilTagTracking = this@VisionTestRobot.aprilTagTracking
        val aprilTagOdometry = this@VisionTestRobot.aprilTagOdometry
    }

    override fun impl(hardwareMap: IHardwareMap) = Impl(hardwareMap)
}