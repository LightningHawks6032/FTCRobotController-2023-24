package org.firstinspires.ftc.teamcode.test

import org.firstinspires.ftc.teamcode.controlSystems.motionPath.MotionPathBuilder
import org.firstinspires.ftc.teamcode.controlSystems.motionPath.buildPath
import org.firstinspires.ftc.teamcode.robot.arbot.FTCCenterStagePOIs
import org.firstinspires.ftc.teamcode.util.Alliance
import org.firstinspires.ftc.teamcode.util.Vec2
import org.firstinspires.ftc.teamcode.util.Vec2Rot
import org.firstinspires.ftc.teamcode.vision.prop.TeamPropVisProcessor.Loc

private val POIs = FTCCenterStagePOIs

fun main() {

    val alliance = Alliance.Red
    val detectedLoc = Loc.Right

    val startingPos = POIs.startingPosition(alliance, 12.0, true)

    val (p,_) = buildPath(startingPos, Vec2Rot.zero) {
        // go to center of upper field at t = 1.5
        mark()

        stationaryR(Lasting(1.0.seconds))
        bezierR(UntilSinceMark(2.5.seconds), 0.0, 0.0)
        bezierXY(UntilSinceMark(2.5.seconds), POIs.relativePos(alliance, 12.0, 36.0))


        // place ground pixel
        mark()

        val groundPlacingDelay: MotionPathBuilder.TimeWithUnits
        if (detectedLoc == Loc.Center) {
            // towards the opponent
            bezierR(UntilSinceMark(1.0.seconds), FTCCenterStagePOIs.Facing.Self.angle(alliance))
            bezierXY(UntilSinceMark(1.0.seconds), POIs.relativePos(alliance, 12.0, 48.0 - 9.0))
            groundPlacingDelay = 1.0.seconds
            stationaryLasting(0.25.seconds)
            bezierXY(UntilSinceMark(2.0.seconds), POIs.relativePos(alliance, 12.0, 48.0 - 9.0 - 6.0))
        } else if ((detectedLoc == Loc.Right) == (alliance == Alliance.Red)) {
            // towards the pixel board
            bezierR(UntilSinceMark(2.0.seconds), FTCCenterStagePOIs.Facing.Forwards.angle(alliance))
            bezierXY(UntilSinceMark(2.0.seconds), POIs.relativePos(alliance, 24.0 + 9.0, 36.0 + 6.0))
            groundPlacingDelay = 2.0.seconds
            stationaryLasting(0.25.seconds)
            bezierXY(UntilSinceMark(3.0.seconds), POIs.relativePos(alliance, 24.0 + 9.0 + 3.0, 36.0 + 6.0))
        } else {
            // towards the audience
            bezierR(UntilSinceMark(1.0.seconds), FTCCenterStagePOIs.Facing.Forwards.angle(alliance))
            bezierXY(UntilSinceMark(1.0.seconds), POIs.relativePos(alliance, 1.0 + 9.0, 36.0 + 6.0))
            groundPlacingDelay = 1.0.seconds
            stationaryLasting(0.25.seconds)
            bezierXY(UntilSinceMark(2.0.seconds), POIs.relativePos(alliance, 1.0 + 9.0 + 3.0, 36.0 + 6.0))

        }
        actAtTSinceMark(groundPlacingDelay) {
//            robot.underPixel.hold = false
        }


        // go to the backdrop and extend the outtake
        mark()

        val backdropXOff = when (detectedLoc) {
            Loc.Left -> +6.0
            Loc.Center -> 0.0
            Loc.Right -> -6.0
        }
        actAtTSinceMark(0.0.seconds) {
//            robot.outtake.controller.automatedGoToExtension(2.0, 24.0 / cos(40.0 / 180.0 * PI))
        }
        bezierR(Lasting(1.0.seconds), 0.0, 0.0)
        bezierXY(Lasting(2.5.seconds), POIs.facingBackdropFromDistance(alliance, 11.0, strafe = backdropXOff))


        // drop held pixels
        mark()

        actAtTSinceMark(0.5.seconds) {
//            robot.outtake.dropOpen = true
        }
        stationaryLasting(3.0.seconds)
        actAtTSinceMark(3.0.seconds) {
            // retract outtake
//            robot.outtake.dropOpen = false
//            robot.outtake.controller.automatedGoToExtension(2.0, 0.0)
        }


        // parking in backstage
        mark()

        bezierR(Lasting(1.0.seconds), 0.0, 0.0)
        bezierXY(Lasting(0.5.seconds),
                lastPos.v + Vec2(-8.0, 0.0),
                Vec2(-12.0, 0.0))
        bezierXY(Lasting(2.0.seconds),
                POIs.relativePos(alliance, 44.0, 14.0),
                POIs.relativeVel(alliance, 10.0, -3.0))
        bezierXY(Lasting(1.5.seconds),
                POIs.relativePos(alliance, 72.0 - 11.0, 11.0))


        // done, shut down
        mark()

        actAtTSinceMark(0.0.seconds) {
//            robot.drive.shutdownOutput()
        }
    }

    println("t\tx\ty\tr")
    var t = 0.0
    while (t < p.validUntilT) {
        t += 0.01291
        val (v, r) = p.sampleClamped(t).vel
        val (x, y) = v
        println("$t\t$x\t$y\t$r")
    }
}