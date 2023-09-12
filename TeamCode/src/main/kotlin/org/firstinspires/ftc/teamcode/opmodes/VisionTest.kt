package org.firstinspires.ftc.teamcode.opmodes

import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.LOpMode
import org.firstinspires.ftc.teamcode.ftcGlue.IHardwareMap
import org.firstinspires.ftc.teamcode.ftcGlue.IRobot
import org.firstinspires.ftc.teamcode.vision.VisAprilTag
import org.firstinspires.ftc.teamcode.vision.Vision
import org.firstinspires.ftc.teamcode.vision.createVisionLoop
import org.firstinspires.ftc.vision.VisionPortal
import kotlin.math.roundToInt

val visionTestRobot = VisionTestRobot()

@TeleOp
class VisionTest : LOpMode<VisionTestRobot.Impl>(visionTestRobot, {
    withTelemetry {
        ln("init began")
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
            // Report
            withTelemetry {
                if (detections.isEmpty()) {
                    ln("no detections")
                }
                for (detection in detections) {
                    print(detection)
                    ln("-- tag #${detection.id} ---------")
                    ln("[${detection.corners?.joinToString("; ") { "${it.x.roundToInt()},${it.y.roundToInt()}" }}]")
                    ln(detection.ftcPose?.let {
                        "(${it.x}, ${it.y}, ${it.z})\n(${it.pitch}, ${it.yaw}, ${it.roll})"
                    } ?: "unknown position")
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
        +VisAprilTag.TagInfo(42,"test",5.0)
    })

    inner class Impl(hardwareMap: IHardwareMap) {
        val vision = this@VisionTestRobot.vision.Impl(hardwareMap)
        val aprilTag = aprilTagSpec.Instance()
    }

    override fun impl(hardwareMap: IHardwareMap) = Impl(hardwareMap)
}