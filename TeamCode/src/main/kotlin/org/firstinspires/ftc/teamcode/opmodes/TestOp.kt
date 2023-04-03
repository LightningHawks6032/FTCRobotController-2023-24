package org.firstinspires.ftc.teamcode.opmodes

import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.LOpMode
import org.firstinspires.ftc.teamcode.robot.RobotA

// sensor input
// -> motor output

@TeleOp
class TestOp : LOpMode<RobotA.Impl>(RobotA.instance, {
    robot.init()

    var n = 0
    createLoop({ duringInit }) {
        withTelemetry {
            ln("test",n++)
        }
    }

    waitForStart()

    createLoop {
        watches({ gamepadA.a.Watch(it) }) {
            it.changed.bind { isHeld ->
                robot.motor.power = if (isHeld) 1.0 else 0.0
            }
        }
        val triggerHeldAmt = gamepadA.trigger.let { it.left + it.right }

        withTelemetry {
            ln("hello world")
            ln("the thingy", triggerHeldAmt)
        }
    }
})