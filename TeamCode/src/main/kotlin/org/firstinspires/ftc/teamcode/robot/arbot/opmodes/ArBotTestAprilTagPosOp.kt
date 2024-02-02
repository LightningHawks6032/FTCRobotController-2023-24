package org.firstinspires.ftc.teamcode.robot.arbot.opmodes

import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.LOpMode
import org.firstinspires.ftc.teamcode.robot.arbot.ArBotRobot
import org.firstinspires.ftc.teamcode.util.NotForCompetition

@TeleOp
@OptIn(NotForCompetition::class)
class ArBotTestAprilTagPosOp : LOpMode<ArBotRobot.Impl>(ArBotRobot, {

    robot.createAprilTagVisionLoops(this, condition = { duringInit || duringRun })

    waitForStart()

    robot.drive.debugDeactivateOutput()
    createLoop{
        robot.drive.tick(dt)
        withTelemetry {
            robot.drive.writeTelemetry(this)
        }
    }
})