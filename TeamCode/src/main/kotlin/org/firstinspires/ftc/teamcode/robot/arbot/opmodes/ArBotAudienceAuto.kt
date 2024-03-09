package org.firstinspires.ftc.teamcode.robot.arbot.opmodes

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import org.firstinspires.ftc.teamcode.LOpMode
import org.firstinspires.ftc.teamcode.controlSystems.ActionSequence
import org.firstinspires.ftc.teamcode.controlSystems.motionPath.MotionPath
import org.firstinspires.ftc.teamcode.controlSystems.motionPath.MotionPathBuilder
import org.firstinspires.ftc.teamcode.controlSystems.motionPath.buildPath
import org.firstinspires.ftc.teamcode.robot.arbot.ArBotRobot
import org.firstinspires.ftc.teamcode.robot.arbot.FTCCenterStagePOIs
import org.firstinspires.ftc.teamcode.util.Alliance
import org.firstinspires.ftc.teamcode.util.Vec2Rot
import org.firstinspires.ftc.teamcode.util.promptSelectAlliance
import org.firstinspires.ftc.teamcode.vision.prop.TeamPropVisProcessor

private val POIs = FTCCenterStagePOIs

@Autonomous
class ArBotAudienceAuto : LOpMode<ArBotRobot.Impl>(ArBotRobot, {
    robot.groundPixel.hold = true

    val alliance = promptSelectAlliance()
    var detectedLoc = TeamPropVisProcessor.Loc.Center
    var inputAxesFunctional = false
    createLoop(condition = {duringInit}) {
        inputAxesFunctional = robot.drive.checkInputAxesFunctional()
    }
    robot.createTeamPropDetectionVisionLoop(this, alliance, { duringInit }) { loc ->
        detectedLoc = loc
        withTelemetry {
            ln("Alliance: $alliance")
            ln("Detected team prop location: $loc")
            ln("Input functioning: $inputAxesFunctional")
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

        stationaryR(Lasting(1.0.seconds))
        bezierR(UntilSinceMark(2.5.seconds), 0.0, 0.0)
        bezierXY(UntilSinceMark(2.5.seconds), POIs.relativePos(alliance, -36.0, 36.0))


        // place ground pixel
        mark()

        val groundPlacingDelay: MotionPathBuilder.TimeWithUnits
        if (detectedLoc == TeamPropVisProcessor.Loc.Center) {
            // towards the opponent
            bezierR(UntilSinceMark(1.0.seconds), FTCCenterStagePOIs.Facing.Self.angle(alliance))
            bezierXY(UntilSinceMark(1.0.seconds), POIs.relativePos(alliance, -36.0, 48.0 - 9.0 + 1.5))
            groundPlacingDelay = 1.5.seconds
            stationaryLasting(1.0.seconds)
            bezierXY(UntilSinceMark(3.0.seconds), POIs.relativePos(alliance, -38.0, 48.0 - 9.0 - 6.0))
        } else if ((detectedLoc == TeamPropVisProcessor.Loc.Right) == (alliance == Alliance.Red)) {
            // towards the pixel board
            bezierR(UntilSinceMark(2.0.seconds), FTCCenterStagePOIs.Facing.Backwards.angle(alliance))
            bezierXY(UntilSinceMark(2.0.seconds), POIs.relativePos(alliance, -24.0 - 9.0 + 1.5, 48.0 - 10.5))
            groundPlacingDelay = 2.5.seconds
            stationaryLasting(1.0.seconds)
            bezierXY(UntilSinceMark(4.0.seconds), POIs.relativePos(alliance, -24.0 - 9.0 - 3.0, 48.0 - 10.5))
        } else {
            // towards the audience
            bezierR(UntilSinceMark(1.0.seconds), FTCCenterStagePOIs.Facing.Backwards.angle(alliance))
            bezierXY(UntilSinceMark(1.0.seconds), POIs.relativePos(alliance, -48.0 - 9.0 + 1.5, 36.0 + 6.0))
            groundPlacingDelay = 1.5.seconds
            stationaryLasting(1.0.seconds)
            bezierXY(UntilSinceMark(3.0.seconds), POIs.relativePos(alliance, -48.0 - 9.0 - 3.0, 36.0 + 6.0))
            bezierXY(UntilSinceMark(5.5.seconds), POIs.relativePos(alliance, -48.0 - 9.0 - 3.0, 36.0 + 6.0 - 12.0))

        }
        actAtTSinceMark(groundPlacingDelay) {
            robot.groundPixel.hold = false
        }


//        // parking in thingy
//        mark()
//
//
//        bezierXY(UntilSinceMark(4.0.seconds),
//                POIs.relativePos(alliance, 10.0 - 6.0 * 12.0, 10.0))
//

        // done, shut down
        mark()

        actAtTSinceMark(0.0.seconds) {
            robot.drive.shutdownOutput()
        }
    }

//    robot.createAprilTagVisionLoops(this, condition = { duringRun })
    createLoop {
        val R = 1.0
        actionExecutor.tick(dt * R)
        robot.drive.tick(dt * R)
        robot.outtake.tick(dt * R)
        withTelemetry {
            robot.drive.writeTelemetry(this)
        }
    }

    followPath(path0)
})