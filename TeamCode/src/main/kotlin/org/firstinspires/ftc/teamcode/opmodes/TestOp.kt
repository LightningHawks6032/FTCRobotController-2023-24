package org.firstinspires.ftc.teamcode.opmodes

import com.qualcomm.robotcore.eventloop.opmode.Disabled
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.LOpMode
import org.firstinspires.ftc.teamcode.ftcGlue.IHardwareMap
import org.firstinspires.ftc.teamcode.ftcGlue.IRobot
import org.firstinspires.ftc.teamcode.hardware.Motor
import org.firstinspires.ftc.teamcode.hardware.commonAssembly.MecanumDrive
import org.firstinspires.ftc.teamcode.util.Vec2
import org.firstinspires.ftc.teamcode.util.Vec2Rot

// sensor input
// -> motor output

@TeleOp
@Disabled
class TestOp : LOpMode<RobotA.Impl>(RobotA.instance, {
    robot.init()

    var n = 0
    createLoop({ duringInit }) {
        withTelemetry {
            ln("test", n++)
        }
    }

    waitForStart()

    createLoop {
        val triggerHeldAmt = gamepadA.trigger.let { it.left + it.right }

        withTelemetry {
            ln("hello world")
            ln("the thingy", triggerHeldAmt)
        }
    }

    createLoop {
        watches({ gamepadA.dpad.up.Watch(it) }) {
            it.pressed.bind {
                robot.doThing()
            }
        }
    }

    cleanupOnStop {
        withTelemetry {
            ln("it end")
        }
    }

}, FinishedBehaviour.KEEP_ALIVE_UNTIL_LOOPS_END)
class RobotA : IRobot<RobotA.Impl> {
    val drive = MecanumDrive(
            Motor.PhysicalSpec.GOBILDA_5202_0002_0005,
            MecanumDrive.ReversalPattern(
                    right = true
            ),
            MecanumDrive.Ids.default
    )
    val encoder = Motor(
            "enc",
            Motor.PhysicalSpec.REV_THROUGH_BORE_ENCODER,
            Motor.Config.default
    )

    inner class Impl(hardwareMap: IHardwareMap) {
        private val drive = this@RobotA.drive.Impl(hardwareMap)
        private val encoder = this@RobotA.encoder.Impl(hardwareMap)

        fun init() {
            // could do something here, just as an example
        }

        fun doThing() {
            drive.power = Vec2Rot(Vec2(0.0, 1.0), -encoder.pos)
        }
    }

    override fun impl(hardwareMap: IHardwareMap) = Impl(hardwareMap)

    companion object {
        val instance = RobotA()
    }
}