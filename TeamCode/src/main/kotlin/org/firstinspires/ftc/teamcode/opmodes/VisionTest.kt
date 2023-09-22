package org.firstinspires.ftc.teamcode.opmodes

import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.robotcore.external.matrices.VectorF
import org.firstinspires.ftc.robotcore.external.navigation.Quaternion
import org.firstinspires.ftc.teamcode.LOpMode
import org.firstinspires.ftc.teamcode.ftcGlue.IHardwareMap
import org.firstinspires.ftc.teamcode.ftcGlue.IRobot
import org.firstinspires.ftc.teamcode.hardware.motion.AprilTagOdometry
import org.firstinspires.ftc.teamcode.util.Vec3
import org.firstinspires.ftc.teamcode.util.selectDebugBool
import org.firstinspires.ftc.teamcode.vision.VisAprilTag
import org.firstinspires.ftc.teamcode.vision.Vision
import org.firstinspires.ftc.teamcode.vision.createVisionLoop
import org.firstinspires.ftc.vision.VisionPortal

val visionTestRobot = VisionTestRobot()

@TeleOp
class VisionTest : LOpMode<VisionTestRobot.Impl>(visionTestRobot, {
    selectDebugBool("invertTagRot")
    withTelemetry {
        ln("init began")
    }
    createLoop({ duringInit || duringRun }) {
        robot.aprilTagOdometry.tick(dt)
    }
    createVisionLoop(
            robot.vision,
            listOf(robot.aprilTag),
            condition = { duringInit || duringRun },
    ) { vision ->
        if (vision.cameraState != VisionPortal.CameraState.STREAMING) {
            withTelemetry {
                ln("camera starting")
            }
            CONTINUE
        }
        if (duringRun) {
            robot.aprilTag.decimation = 1.0

            // Get new detections, if none, wait.
            val detections = robot.aprilTag.freshDetections ?: CONTINUE
            robot.aprilTagOdometry.tickTrackedTags(0, detections)
            // Report
            withTelemetry {
                if (detections.isEmpty()) {
                    ln("no detections")
                }
                for (detection in detections) {
                    print(detection)
                    ln("-- tag #${detection.id} ---------")

                }
            }
        } else {
            robot.aprilTag.decimation = 10.0
            withTelemetry {
                ln("camera online")
            }
        }
    }
    waitForStart()
})

class VisionTestRobot : IRobot<VisionTestRobot.Impl> {
    val vision = Vision("Webcam 1")
    val aprilTagSpec = VisAprilTag(VisAprilTag.specifyTagInfo {
        addTagsCenterStage()
        addTag(
                42,
                "test",
                6.0,
                VectorF(0f, 0f, 0f, 0f),
                Quaternion(0f, 0f, 1f, 0f, 0),
        )
    })

    inner class Impl(hardwareMap: IHardwareMap) {
        val vision = this@VisionTestRobot.vision.Impl(hardwareMap)
        val aprilTag = aprilTagSpec.Instance()
        val aprilTagOdometry = AprilTagOdometry(
                AprilTagOdometry.CameraPlacement(Vec3(0.0, 0.0, 0.0), 0.0)
        )
    }

    override fun impl(hardwareMap: IHardwareMap) = Impl(hardwareMap)
}