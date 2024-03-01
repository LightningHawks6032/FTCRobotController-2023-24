package org.firstinspires.ftc.teamcode.robot.arbot.opmodes

import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.LOpMode
import org.firstinspires.ftc.teamcode.robot.OpGroupName
import org.firstinspires.ftc.teamcode.robot.arbot.ArBotRobot
import org.firstinspires.ftc.teamcode.util.NotForCompetition

@NotForCompetition
@TeleOp(name = "TestIntakeAndOuttake", group = OpGroupName.DEBUGGING)
class ArBotTestIntakeAndOuttakeOp : LOpMode<ArBotRobot.Impl>(ArBotRobot, {
//    robot.intake.pos = 0.0

    robot.outtake.pos = 0.0

    waitForStart()

    createLoop {
        /// intake
//        if (gamepadA.a.isHeld) {
//            robot.intake.tick(dt)
//        } else {
//            robot.intake.debugAnglePower = 0.0
//        }
        if (gamepadA.b.isHeld) {
            robot.outtake.tick(dt)
        } else {
            robot.outtake.debugLifterPower = 0.0
        }

        val intakeTarget = gamepadA.stick.left.pos.x
//        robot.intake.angleController.targetPosition = intakeTarget

        val spinPower = gamepadA.stick.right.pos.x
        robot.intake.spinPower = spinPower

        /// outtake
        val outtakeTarget = gamepadA.trigger.left * 10.0
        robot.outtake.controller.targetPosition = outtakeTarget

        val outtakeTilt = gamepadA.x.isHeld
        val outtakeDrop = gamepadA.y.isHeld
//        robot.outtake.outtakeTilt = outtakeTilt
        robot.outtake.dropOpen = outtakeDrop

        withTelemetry {
            ln("intake target rotation (radians)", intakeTarget)
            ln("intake spinner power", spinPower)
            ln()
            ln("outtake target rotation (radians)", outtakeTarget)
            ln("outtake tilt?", outtakeTilt)
            ln("outtake drop?", outtakeDrop)
        }
    }

})