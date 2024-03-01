package org.firstinspires.ftc.teamcode.robot.arbot.opmodes

import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.LOpMode
import org.firstinspires.ftc.teamcode.ftcGlue.WithTelemetry
import org.firstinspires.ftc.teamcode.robot.arbot.ArBotRobot
import org.firstinspires.ftc.teamcode.robot.arbot.FTCCenterStagePOIs
import org.firstinspires.ftc.teamcode.util.*
import kotlin.math.*

private val POIs = FTCCenterStagePOIs

@TeleOp
class ArBotTeleOp : LOpMode<ArBotRobot.Impl>(ArBotRobot, {
    val outtakeArmMaxHeight = 36.0
    robot.outtake.pos = 0.0
//    robot.intake.pos = 0.0 // assert that we start here

//    selectDebugBool("switch_control_scheme")
//    val switchControlScheme = DebugVars.bool["switch_control_scheme"] ?: false

    withTelemetry {
        ln("initialized, waiting for start")
    }
    waitForStart()

    val withDriveStatus = WithTelemetry.Partial()
    val withControlStatus = WithTelemetry.Partial()

    createLoop {
        val slowA = !gamepadA.bumper.let { it.left.isHeld || it.right.isHeld }
        val slowMultiplierA = if (slowA) 0.1 else 0.3
        val slowB = gamepadB.bumper.let { it.left.isHeld || it.right.isHeld }
        val slowMultiplierB = if (slowB) 0.3 else 1.0
        // thing controls

        // ---------- PLANES ----------- //
        robot.planeLauncher.launch = gamepadB.b.isHeld

        // ---------- INTAKE ----------- //
//        /// Intake velocity (positive -> down)
//        val intakeV = gamepadB.trigger.let { it.left - it.right } * slowMultiplierB
//        var intakeX by robot.intake.angleController::targetPosition
//        intakeX = (intakeX + dt * intakeV).clamp(-PI / 2, 0.0)
////        robot.intake.tick(dt)
//        @OptIn(NotForCompetition::class)
//        robot.intake.debugAnglePower = intakeV

        robot.intake.spinPower = gamepadB.dpad.let { (if (it.up.isHeld) 1.0 else 0.0) - (if (it.down.isHeld) 1.0 else 0.0) }

        // ---------- OUTTAKE ---------- //
        /// Outtake velocity (positive -> up)
        var outtakeV = 19.5 * -gamepadB.stick.right.pos.y * slowMultiplierB
        var outtakeX by robot.outtake.controller::targetPosition
        if (outtakeX < 5.0) {
            outtakeV *= 0.5
        }
        if (outtakeX < 2.0 && outtakeV <= 0.01) {
            outtakeV = -2.0
        }
        outtakeX = (outtakeX + dt * outtakeV).clamp(0.0, outtakeArmMaxHeight)
//        watches(gamepadB.x::Watch) {
//            it.pressed.bind {
//                robot.outtake.outtakeTilt = !robot.outtake.outtakeTilt
//            }
//        }

        // Force-close the outtake box under a certain point to avoid damage.
        // (if expected position 0.5 seconds in the future is less than 10in)
//        if (outtakeX + outtakeV * 0.5 < 10.0) {
//            robot.outtake.outtakeTilt = false
//        }
        robot.outtake.dropOpen = gamepadB.a.isHeld
        // reset slide position
        if (gamepadB.y.isHeld && outtakeX < 4.0) {
            robot.outtake.pos = 0.0
            @OptIn(NotForCompetition::class)
            robot.outtake.debugLifterPower = -1.0
        } else {
            robot.outtake.tick(dt)
        }

        withControlStatus {
//            ln("intake", intakeX)
            ln("outtake", outtakeX)
            ln("launch", robot.planeLauncher.launch)
        }
    }

    var pos = Vec2Rot.zero
    robot.drive.path = null
    robot.drive.assertPosition(pos)

    createLoop {
        /// drive
        val slow = !gamepadA.bumper.let { it.left.isHeld || it.right.isHeld }
        val speedLinear = if (slow) 12.0 else 36.0
        val speedAngular = if (slow) PI / 2 else PI

        // triggers to strafe
        val input = Vec2Rot(
                x = -gamepadA.stick.left.pos.y,
                y = gamepadA.trigger.let { it.left - it.right },
                r = -gamepadA.stick.right.pos.x,
        ).transformP { it.rotate(robot.drive.inputPos.r) }
//        val vel = input.componentwiseTimes(Vec2Rot(speedLinear, speedLinear, speedAngular))
//
//        pos += (vel * dt).transformP { (x, y) -> Vec2(x.coerceIn(-1.0, 1.0), y.coerceIn(-1.0, 1.0)) }
////        pos = pos.transformP { (x,y) ->
////            val radius = 9.0 * (cos(abs((abs(pos.r)%(PI/2))-PI/4))) * sqrt(0.5)
////            Vec2(
////                x.coerceIn(POIs.fieldBounds.x.first + radius, POIs.fieldBounds.x.second - radius),
////                y.coerceIn(POIs.fieldBounds.y.first + radius, POIs.fieldBounds.y.second - radius),
////            )
////        }.let {
////            POIs.coerceOutOfDangerZone(pos, robot.drive.inputVel)
////        }
//
//        robot.drive.setPowerAndTrack(input.componentwiseTimes(Vec2Rot(1.0, 1.0, 0.5)), dt)
////        robot.drive.targetPos = pos
////        robot.drive.tick(dt)
        robot.semiAuto.tick(dt, input)

        withDriveStatus {
            robot.drive.writeTelemetry(this)
        }
    }

    createLoop {
        withTelemetry {
            +withControlStatus
            ln("------------------")
            +withDriveStatus
        }
    }

})