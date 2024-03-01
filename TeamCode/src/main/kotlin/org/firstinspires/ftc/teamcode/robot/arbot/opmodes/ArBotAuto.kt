package org.firstinspires.ftc.teamcode.robot.arbot.opmodes

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import org.firstinspires.ftc.teamcode.LOpMode
import org.firstinspires.ftc.teamcode.controlSystems.ActionSequence
import org.firstinspires.ftc.teamcode.controlSystems.motionPath.BezierMotionPath
import org.firstinspires.ftc.teamcode.controlSystems.motionPath.MotionPath
import org.firstinspires.ftc.teamcode.controlSystems.motionPath.buildPath
import org.firstinspires.ftc.teamcode.robot.arbot.ArBotRobot
import org.firstinspires.ftc.teamcode.robot.arbot.FTCCenterStagePOIs
import org.firstinspires.ftc.teamcode.util.CubicBezier
import org.firstinspires.ftc.teamcode.util.Vec2
import org.firstinspires.ftc.teamcode.util.Vec2Rot
import org.firstinspires.ftc.teamcode.util.promptSelectAlliance
import org.firstinspires.ftc.teamcode.vision.prop.TeamPropVisProcessor

private val POIs = FTCCenterStagePOIs

@Autonomous
class ArBotAuto : LOpMode<ArBotRobot.Impl>(ArBotRobot, {
    robot.underPixel.hold = true

    val alliance = promptSelectAlliance()
    var detectedLoc = TeamPropVisProcessor.Loc.Center
    robot.createTeamPropDetectionVisionLoop(this, alliance, { duringInit}) { loc ->
        detectedLoc = loc
        withTelemetry {
            ln("Alliance: $alliance")
            ln("Detected team prop location: $loc")
        }
    }

    val startingPos = POIs.startingPosition(alliance, 12.0, true)
    robot.drive.assertPosition(startingPos)

    val actionExecutor = ActionSequence.Executor()
    fun followPath(path: Pair<MotionPath<Vec2Rot>, ActionSequence>) {
        robot.drive.path = path.first
        actionExecutor.start(path.second)
    }

    waitForStart()

    val path0 = buildPath(startingPos, Vec2Rot.zero) {
        // go to center of upper field at t = 1.5
        mark()
//        stationaryR(Lasting(1.0.seconds))
//        bezierR(UntilSinceMark(2.5.seconds), 0.0, 0.0)
        bezierXY(UntilSinceMark(2.5.seconds), POIs.relativePos(alliance, 12.0, 36.0))


//        // place ground pixel
//        mark()
//
//        if (detectedLoc == TeamPropVisProcessor.Loc.Center) {
//            bezierR(Lasting(0.5.seconds), )
//        } else if ((detectedLoc == TeamPropVisProcessor.Loc.Right) == (alliance == Alliance.Red)) {
//            // towards the pixel board
//        } else {
//            // towards the audience
//
//        }
//
//        bezierXY(Lasting(2.5.seconds), POIs.relativePos(alliance, 12.0, 36.0))



        // wait a moment
        stationaryLasting(1.0.seconds)

        // go to the backdrop and extend the outtake
        mark()
        bezierR(Lasting(1.0.seconds), 0.0, 0.0)
        bezierXY(
                Lasting(1.0.seconds),
                POIs.relativePos(alliance, 36.0, 4.5 * 12.0),
                POIs.relativeVel(alliance, 0.0, -12.0),
        )
        bezierXY(Lasting(2.0.seconds), POIs.facingBackdropFromDistance(alliance, 11.0))
        actAtTSinceMark(0.0.seconds) {
            robot.outtake.controller.path = BezierMotionPath.TDouble(
                    CubicBezier.forEndpointTangents(3.0, 0.0, 0.0, 30.0, 0.0)
            )
//            robot.outtake.outtakeTilt = true
        }

        // drop held pixels
        mark()
        stationaryLasting(4.0.seconds)
        actAtTSinceMark(0.5.seconds) {
            robot.outtake.dropOpen = true
        }
        // retract outtake
        actAtTSinceMark(4.0.seconds) {
//            robot.outtake.outtakeTilt = false
            robot.outtake.dropOpen = false
            robot.outtake.controller.path = BezierMotionPath.TDouble(
                    CubicBezier.forEndpointTangents(4.0, 30.0, 0.0, 0.0, 0.0)
            )
        }

        // parking in backstage
        mark()
        bezierR(Lasting(1.0.seconds), 0.0, 0.0)
        bezierXY(Lasting(0.5.seconds), lastPos.v + Vec2(-8.0, 0.0), Vec2(-6.0, 0.0))
        bezierXY(Lasting(1.0.seconds),
                POIs.relativePos(alliance, 36.0, 24.0),
                POIs.relativeVel(alliance, 0.0, -5.0),
        )
        bezierXY(Lasting(1.0.seconds), POIs.relativePos(alliance, 44.0, 14.0))
        bezierXY(Lasting(1.5.seconds), POIs.relativePos(alliance, 72.0-10.0, 10.0))

        actAtTSinceMark(4.0.seconds) {
            robot.drive.shutdownOutput()
        }
    }

    createLoop {
        actionExecutor.tick(dt)
        robot.drive.tick(dt)
        robot.outtake.tick(dt)
//        robot.intake.tick(dt)
        withTelemetry {
            robot.drive.writeTelemetry(this)
        }
    }

    followPath(path0)
})