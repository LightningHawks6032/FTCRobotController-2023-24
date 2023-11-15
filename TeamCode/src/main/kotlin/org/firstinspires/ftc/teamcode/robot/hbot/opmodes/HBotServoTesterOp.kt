package org.firstinspires.ftc.teamcode.robot.hbot.opmodes

import org.firstinspires.ftc.teamcode.LOpMode
import org.firstinspires.ftc.teamcode.robot.hbot.HBotRobot
import org.firstinspires.ftc.teamcode.util.NotForCompetition

@NotForCompetition
class HBotServoTesterOp : LOpMode<HBotRobot.Impl>(HBotRobot, {
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
            }
        }
        watches(gamepadA.dpad.down::Watch) {
            it.pressed.bind {
                i -= 1
            }
        }
        val j = (i%n+n)%n

        servos[i].pos = gamepadA.stick.right.pos.x

        withTelemetry {
            ln("servo group positions:")
            for (k in 0 until n) {
                ln("${if (j == k) "> " else ""}${labels[i]}", servos[i].pos)
            }
        }
    }
})