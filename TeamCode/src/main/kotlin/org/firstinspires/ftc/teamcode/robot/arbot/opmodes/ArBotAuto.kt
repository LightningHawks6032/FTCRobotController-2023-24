package org.firstinspires.ftc.teamcode.robot.arbot.opmodes

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import org.firstinspires.ftc.teamcode.LOpMode
import org.firstinspires.ftc.teamcode.controlSystems.motionPath.buildPath
import org.firstinspires.ftc.teamcode.robot.arbot.ArBotRobot
import org.firstinspires.ftc.teamcode.util.TriggerLock
import org.firstinspires.ftc.teamcode.util.Vec2
import org.firstinspires.ftc.teamcode.util.Vec2Rot

private enum class Alliance {
    Red, Blue;

    fun startingPosition(): Vec2Rot {
        TODO()
    }

    fun allianceName() = when (this) {
        Red -> "Red"
        Blue -> "Blue"
    }
}

@Autonomous
class ArBotAuto : LOpMode<ArBotRobot.Impl>(ArBotRobot, {

    var alliance = Alliance.Red
    val allianceAccepted = TriggerLock()

    createLoop({ duringInit }) {
        watches(gamepadA.a::Watch) {
            it.pressed.once { allianceAccepted.unlock() }
        }
        withTelemetry {
            ln("Selected ${alliance.allianceName()} alliance")

            ln("B -> Red")
            if (gamepadA.b.isHeld)
                alliance = Alliance.Red
            ln("X -> Blue")
            if (gamepadA.x.isHeld)
                alliance = Alliance.Blue
        }
    }

    waitForStart()


    robot.drive.assertPosition(alliance.startingPosition())


    createLoop {
        robot.drive.tick(dt)
        robot.outtake.tick(dt)
        robot.intake.tick(dt)
        withTelemetry {
            robot.drive.writeTelemetry(this)
        }
    }

    robot.drive.path = buildPath(alliance.startingPosition(), Vec2Rot.zero) {
        bezierXY(UntilAt(5.0.seconds), Vec2(0.0, -36.0))
    }

})