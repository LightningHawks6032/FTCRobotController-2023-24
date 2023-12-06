package org.firstinspires.ftc.teamcode.robot.arbot.opmodes

import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.LOpMode
import org.firstinspires.ftc.teamcode.ftcGlue.WithTelemetry
import org.firstinspires.ftc.teamcode.robot.arbot.ArBotRobot
import org.firstinspires.ftc.teamcode.util.NotForCompetition
import org.firstinspires.ftc.teamcode.util.Vec2Rot

@NotForCompetition
@TeleOp
class ArBotTestDriveOp : LOpMode<ArBotRobot.Impl>(ArBotRobot, {
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
        val slowButton = gamepadA.x.isHeld
        val speed = 0.25 * (if (speedButton) 2.0 else 1.0) * (if (slowButton) 0.5 else 1.0)

//        drive.power = (Vec2Rot(
//                v = gamepadA.stick.left.pos.let { Vec2(it.y, it.x) },
//                r = gamepadA.stick.right.pos.x,
//        ) * speed)
        drive.power = (Vec2Rot(
                x = -gamepadA.stick.left.pos.y,
                y = gamepadA.trigger.let { it.left - it.right },
                r = -gamepadA.stick.right.pos.x,
        ) * speed)
        odometry.tick(dt)

        withTelemetry {
            ln("-- drive power -----------")
            ln("x: ${drive.power.v.x}")
            ln("y: ${drive.power.v.y}")
            ln("r: ${drive.power.r}")
            +odoTelemetry
        }
    }
})