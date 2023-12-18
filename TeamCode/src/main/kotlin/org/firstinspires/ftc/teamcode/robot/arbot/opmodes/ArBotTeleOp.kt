package org.firstinspires.ftc.teamcode.robot.arbot.opmodes

import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.LOpMode
import org.firstinspires.ftc.teamcode.ftcGlue.WithTelemetry
import org.firstinspires.ftc.teamcode.robot.arbot.ArBotRobot
import org.firstinspires.ftc.teamcode.util.*
import kotlin.math.PI

@NotForCompetition
@TeleOp
class ArBotTeleOp : LOpMode<ArBotRobot.Impl>(ArBotRobot, {
    val intakeArmAtOuttakeDropOff = PI/6
    val outtakeArmMaxHeight = 36.0
    robot.intake.pos = intakeArmAtOuttakeDropOff // assert that we start here

    selectDebugBool("switch_control_scheme")
    val switchControlScheme = DebugVars.bool["switch_control_scheme"] ?: false

    withTelemetry {
        ln("initialized, waiting for start")
    }
    waitForStart()

    val withDriveStatus = WithTelemetry.Partial()
    val withControlStatus = WithTelemetry.Partial()

    createLoop {
        val slowA = !gamepadA.bumper.let { it.left.isHeld || it.right.isHeld }
        val slowMultiplierA = if (slowA) 0.1 else 0.3
        val slowB = !gamepadB.bumper.let { it.left.isHeld || it.right.isHeld }
        val slowMultiplierB = if (slowB) 0.1 else 0.3
        // intake/outtake controls

        // ---------- INTAKE ----------- //
        /// Intake velocity (positive -> towards outtake)
        val intakeV = gamepadB.stick.left.pos.x * slowMultiplierB
        var intakeX by robot.intake.angleController::targetPosition
        intakeX = (intakeX + dt * intakeV).clamp(-PI/2, intakeArmAtOuttakeDropOff)
//        robot.intake.tick(dt)
        robot.intake.debugAnglePower = intakeV

        // ---------- OUTTAKE ---------- //
        /// Outtake velocity (positive -> up)
        val outtakeV = 5.0 * if (switchControlScheme) {
            gamepadA.dpad.let { (if (it.up.isHeld) 1.0 else 0.0) + (if (it.down.isHeld) -1.0 else 0.0) } * slowMultiplierA
        } else {
            gamepadB.stick.right.pos.y * slowMultiplierB
        }
        var outtakeX by robot.outtake.controller::targetPosition
        outtakeX = (outtakeX + dt * outtakeV).clamp(0.0, outtakeArmMaxHeight)
        watches(gamepadB.x::Watch) { it.pressed.bind {
            robot.outtake.outtakeTilt = !robot.outtake.outtakeTilt
        } }
        // Force-close the outtake box under a certain point to avoid damage.
        // (if expected position 0.5 seconds in the future is less than 10in)
        if (outtakeX + outtakeV * 0.5 < 10.0) {
            robot.outtake.outtakeTilt = false
        }
        robot.outtake.dropOpen = gamepadB.a.isHeld
//        robot.outtake.tick(dt)
        robot.outtake.debugLifterPower = outtakeV

        withControlStatus {
            ln("i", intakeX)
            ln("o", outtakeX)
        }
    }

    val (odometry, drive) = robot.drive.debugTakeControl()

    createLoop {
        /// drive
        val slow = !gamepadA.bumper.let { it.left.isHeld || it.right.isHeld }
        val slowMultiplier = if (slow) 0.25 else 0.5

        if (switchControlScheme) {
            // triggers to strafe
            drive.power = (Vec2Rot(
                    x = gamepadA.stick.left.pos.y,
                    y = gamepadA.trigger.let { it.right - it.left },
                    r = gamepadA.stick.right.pos.x,
            ) * slowMultiplier)
        } else {
            // single stick linear motion
            drive.power = (Vec2Rot(
                    v = gamepadA.stick.left.pos.let { Vec2(it.y, it.x) },
                    r = gamepadA.stick.right.pos.x,
            ) * slowMultiplier)
        }
//        robot.drive.tick(dt)

        /// odometry

        odometry.tick(dt)
        withDriveStatus {
            ln("pos")
            ln("x", "${odometry.pos.v.x} in")
            ln("y", "${odometry.pos.v.y} in")
            ln("r", "${odometry.pos.r} rad")
            ln()
            ln("vel")
            ln("x", "${odometry.vel.v.x} in/s")
            ln("y", "${odometry.vel.v.y} in/s")
            ln("r", "${odometry.vel.r} rad/s")
        }
    }

    createLoop {
        withTelemetry {
            + withControlStatus
            ln("------------------")
            + withDriveStatus
        }
    }

})