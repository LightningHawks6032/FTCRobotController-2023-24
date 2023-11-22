package org.firstinspires.ftc.teamcode.robot.arbot.opmodes

import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.LOpMode
import org.firstinspires.ftc.teamcode.robot.arbot.ArBotRobot
import org.firstinspires.ftc.teamcode.util.NotForCompetition

@NotForCompetition
@TeleOp
class ArBotServoTesterOp : LOpMode<ArBotRobot.Impl>(ArBotRobot, {
    withTelemetry {
        ln("HBot servo tester")
    }
    waitForStart()
    var i = 0
    val servos = listOf(
            robot.intake.debugControlInputServos(),
            robot.intake.debugControlTransferServos(),
            robot.outtake.debugControlOutputServos(),
    )
    val labels = listOf("input","transfer","output")
    val n = servos.size
    createLoop {
        watches(gamepadA.dpad.up::Watch) {
            it.pressed.bind {
                i += 1
                if (i >= n) i = 0
            }
        }
        watches(gamepadA.dpad.down::Watch) {
            it.pressed.bind {
                i -= 1
                if (i < 0) i = n-1
            }
        }
        val j = (i%n+n)%n

        servos[i].pos = gamepadA.stick.right.pos.x

        withTelemetry {
            ln("servo group positions: $i $j ${gamepadA.dpad.up.isHeld}")
            for (k in 0 until n) {
                ln("${if (j == k) "> " else ""}${labels[k]}", servos[k].pos)
            }
        }
    }
})