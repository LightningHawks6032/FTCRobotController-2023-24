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

private val POIs = FTCCenterStagePOIs

@Autonomous
class ArBotAuto : LOpMode<ArBotRobot.Impl>(ArBotRobot, {

    val alliance = promptSelectAlliance()

    val startingPos = POIs.startingPosition(alliance, 12.0, true)
    robot.drive.assertPosition(startingPos)

    val actionExecutor = ActionSequence.Executor()
    fun followPath(path: Pair<MotionPath<Vec2Rot>, ActionSequence>) {
        robot.drive.path = path.first
        actionExecutor.start(path.second)
    }

    val path0 = buildPath(startingPos, Vec2Rot.zero) {
        // go to center of upper field at t = 1.5
        bezierXY(UntilAt(1.5.seconds), Vec2(36.0, 0.0))
        stationaryR(Lasting(0.2.seconds))
        bezierR(UntilAt(1.5.seconds), 0.0, 0.0)

        // wait a moment
        stationaryUntilAt(3.0.seconds)

        // go to the backdrop and extend the outtake
        bezierXY(
                Lasting(0.5.seconds),
                POIs.relativePos(alliance, 36.0, 4.5 * 12.0),
                POIs.relativeVel(alliance, 0.0, -12.0),
        )
        bezierXY(UntilAt(5.0.seconds), POIs.facingBackdropFromDistance(alliance, 12.0))
        bezierR(UntilAt(5.0.seconds), 0.0, 0.0)
        actAt(3.0.seconds) {
            robot.outtake.controller.path = BezierMotionPath.TDouble(
                    CubicBezier.forEndpointTangents(2.0, 0.0, 0.0, 20.0, 0.0)
            )
        }

        // drop held pixels
        stationaryUntilAt(6.0.seconds)
        actAt(4.5.seconds) {
            robot.outtake.outtakeTilt = true
        }
        actAt(5.5.seconds) {
            robot.outtake.dropOpen = true
        }
        // retract outtake
        actAt(6.0.seconds) {
            robot.outtake.outtakeTilt = false
            robot.outtake.dropOpen = false
            robot.outtake.controller.path = BezierMotionPath.TDouble(
                    CubicBezier.forEndpointTangents(2.0, 20.0, 0.0, 0.0, 0.0)
            )
        }

        // parking in backstage
        bezierXY(Lasting(0.5.seconds), lastPos.v + Vec2(-8.0, 0.0), Vec2(-6.0, 0.0))
        bezierXY(Lasting(1.0.seconds),
                POIs.relativePos(alliance, 36.0, 24.0),
                POIs.relativeVel(alliance, 0.0, -5.0),
        )
        bezierXY(UntilAt(8.0.seconds), POIs.relativePos(alliance, -44.0, 9.0))
        stationaryR(Lasting(0.5.seconds))
        bezierR(Lasting(1.0.seconds), FTCCenterStagePOIs.Facing.Opponent.angle(alliance), 0.0)
        stationaryR(UntilAt(8.0.seconds))
    }

    waitForStart()
    createLoop {
        actionExecutor.tick(dt)
        robot.drive.tick(dt)
        robot.outtake.tick(dt)
        robot.intake.tick(dt)
        withTelemetry {
            robot.drive.writeTelemetry(this)
        }
    }

    followPath(path0)
})