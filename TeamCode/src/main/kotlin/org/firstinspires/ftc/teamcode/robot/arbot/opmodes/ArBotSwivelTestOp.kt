package org.firstinspires.ftc.teamcode.robot.arbot.opmodes

import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.LOpMode
import org.firstinspires.ftc.teamcode.robot.OpGroupName
import org.firstinspires.ftc.teamcode.robot.arbot.ArBotRobot
import org.firstinspires.ftc.teamcode.util.NotForCompetition
import org.firstinspires.ftc.teamcode.util.Vec2Rot

@NotForCompetition
@TeleOp(name = "SwivelTest", group = OpGroupName.DEBUGGING)
class ArBotSwivelTestOp : LOpMode<ArBotRobot.Impl>(ArBotRobot, {

    robot.drive.assertPosition(Vec2Rot.zero)

    withTelemetry {
        ln("swivel drive test thing")
    }

    waitForStart()
    createLoop {

        robot.drive.targetPos = Vec2Rot(gamepadA.stick.left.pos * 10.0, gamepadA.stick.right.pos.x)
        robot.drive.tick(dt)

        withTelemetry {
            ln("swivel drive test thing")
            ln("gamepadA.[left_stick -> pos, right_stick.x -> rotation]")
            robot.drive.writeTelemetry(this)
        }
    }
})