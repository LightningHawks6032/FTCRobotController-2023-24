package org.firstinspires.ftc.teamcode.robot.hbot.opmodes

import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.LOpMode
import org.firstinspires.ftc.teamcode.ftcGlue.WithTelemetry
import org.firstinspires.ftc.teamcode.robot.hbot.HBotRobot
import org.firstinspires.ftc.teamcode.util.NotForCompetition
import org.firstinspires.ftc.teamcode.util.Vec2
import org.firstinspires.ftc.teamcode.util.Vec2Rot

@TeleOp(name = "Drive")
@OptIn(NotForCompetition::class)
class HBotDriveOp : LOpMode<HBotRobot.Impl>(HBotRobot, {
    val (odometry, drive) = robot.drive.debugTakeControl()

    val odoTelemetry = WithTelemetry.Partial()
    createLoop({ duringInit || duringRun }) {
        odoTelemetry {
            ln("-- odometry position -----")
            ln("x", odometry.pos.v.x)
            ln("y", odometry.pos.v.y)
            ln("r", odometry.pos.r)
            ln("--------------------------")
        }
    }
    createLoop({ duringInit }) {
        withTelemetry {
            ln("HBot Drive Initialized")
            +odoTelemetry
        }
    }

    waitForStart()

    createLoop {//lol imagine coding
        val speedButton = gamepadA.bumper.let { it.left.isHeld || it.right.isHeld }
        val speed = if (speedButton) 0.5 else 0.25

        drive.power = (Vec2Rot(
                v = gamepadA.stick.left.pos.let { Vec2(it.y, it.x) },
                r = gamepadA.stick.right.pos.x,
        ) * speed)

        withTelemetry {
            +odoTelemetry
        }
    }
})