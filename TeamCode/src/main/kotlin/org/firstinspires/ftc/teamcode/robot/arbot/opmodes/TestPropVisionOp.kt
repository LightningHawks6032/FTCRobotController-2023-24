package org.firstinspires.ftc.teamcode.robot.arbot.opmodes

import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.LOpMode
import org.firstinspires.ftc.teamcode.robot.arbot.ArBotRobot
import org.firstinspires.ftc.teamcode.util.promptSelectAlliance

@TeleOp
class TestPropVisionOp : LOpMode<ArBotRobot.Impl>(ArBotRobot, {
    robot.createTeamPropDetectionVisionLoop(
            this,
            promptSelectAlliance(),
            condition = { duringInit || duringRun },
    ) { location ->
        withTelemetry {
            ln("Detected: $location")
        }
    }
})