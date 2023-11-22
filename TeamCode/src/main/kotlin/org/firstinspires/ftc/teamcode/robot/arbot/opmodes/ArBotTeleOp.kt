package org.firstinspires.ftc.teamcode.robot.arbot.opmodes

import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.LOpMode
import org.firstinspires.ftc.teamcode.robot.arbot.ArBotRobot
import org.firstinspires.ftc.teamcode.util.*
import kotlin.math.PI

@OptIn(NotForCompetition::class)
@TeleOp
class ArBotTeleOp : LOpMode<ArBotRobot.Impl>(ArBotRobot, {
    val intakeArmAtOuttakeDropOff = PI/6
    val outtakeArmMaxHeight = 36.0
    robot.intake.pos = intakeArmAtOuttakeDropOff // assert that we start here

    selectDebugBool("switch_control_scheme")
    val switchControlScheme = DebugVars.bool["switch_control_scheme"] ?: false

    withTelemetry {
        ln("ok")
    }
    waitForStart()

    withTelemetry {
        ln("started")
    }

    createLoop {
        val slowA = !gamepadA.bumper.let { it.left.isHeld || it.right.isHeld }
        val slowMultiplierA = if (slowA) 0.1 else 0.3
        val slowB = !gamepadB.bumper.let { it.left.isHeld || it.right.isHeld }
        val slowMultiplierB = if (slowB) 0.1 else 0.3
        // intake/outtake controls

        // ---------- INTAKE ----------- //
        /// Intake velocity (positive -> towards outtake)
        val intakeV = gamepadB.stick.left.pos.x * slowMultiplierB
        var intakeX by robot.intake.controller::targetPosition
        intakeX = (intakeX + dt * intakeV).clamp(-PI/2, intakeArmAtOuttakeDropOff)
        watches(gamepadB.a::Watch) { it.pressed.bind {
            robot.intake.inputServosOpen = !robot.intake.inputServosOpen
        } }
        watches(gamepadB.b::Watch) { it.pressed.bind {
            robot.intake.transferServosOpen = !robot.intake.transferServosOpen
        } }
//        robot.intake.tick(dt)
        robot.intake.arm.power = intakeV

        // ---------- OUTTAKE ---------- //
        /// Outtake velocity (positive -> up)
        val outtakeV = 5.0 * if (switchControlScheme) {
            gamepadA.dpad.let { (if (it.up.isHeld) 1.0 else 0.0) + (if (it.down.isHeld) -1.0 else 0.0) } * slowMultiplierA
        } else {
            gamepadB.stick.right.pos.y * slowMultiplierB
        }
        var outtakeX by robot.outtake.controller::targetPosition
        outtakeX = (outtakeX + dt * outtakeV).clamp(-1.0, outtakeArmMaxHeight)
        watches(gamepadB.x::Watch) { it.pressed.bind {
            robot.outtake.outputServosOpen = !robot.outtake.outputServosOpen
        } }
//        robot.outtake.tick(dt)
        robot.outtake.lifter.power = outtakeV

        withTelemetry {
            ln("i", intakeX)
            ln("o", outtakeX)
        }
    }

    val (_odometry, drive) = robot.drive.debugTakeControl()

    createLoop {
        /// drive
        val slow = !gamepadA.bumper.let { it.left.isHeld || it.right.isHeld }
        val slowMultiplier = if (slow) 0.25 else 0.5

        drive.power = (Vec2Rot(
                v = gamepadA.stick.left.pos.let { Vec2(it.y, it.x) },
                r = gamepadA.stick.right.pos.x,
        ) * slowMultiplier)
//        robot.drive.tick(dt)
    }

})