package org.firstinspires.ftc.teamcode.robot.arbot.opmodes

import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.LOpMode
import org.firstinspires.ftc.teamcode.robot.arbot.ArBotRobot
import org.firstinspires.ftc.teamcode.util.NotForCompetition

@NotForCompetition
@TeleOp
class ArBotTestIntakeOp : LOpMode<ArBotRobot.Impl>(ArBotRobot, {
    robot.intake.pos = 0.0

    waitForStart()

    createLoop {
        robot.intake.tick(dt)

        val target = gamepadA.stick.left.pos.x
        robot.intake.angleController.targetPosition = target

        val spinPower = gamepadA.stick.right.pos.x
        robot.intake.spinPower = spinPower

        withTelemetry {
            ln("intake target rotation (radians)", target)
            ln("intake spinner power", spinPower)
        }
    }

})