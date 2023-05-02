package org.firstinspires.ftc.teamcode.opmodes

import com.qualcomm.robotcore.eventloop.opmode.Disabled
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.LOpMode
import org.firstinspires.ftc.teamcode.controlSystems.DriveController
import org.firstinspires.ftc.teamcode.ftcGlue.IHardwareMap
import org.firstinspires.ftc.teamcode.ftcGlue.IRobot
import org.firstinspires.ftc.teamcode.hardware.Motor
import org.firstinspires.ftc.teamcode.hardware.motion.MecanumDrive
import org.firstinspires.ftc.teamcode.hardware.motion.TwoWheelOdometry
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

    createLoop { robot.tick(dt) }

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
    val odo = TwoWheelOdometry(
            Vec2Rot.zero,
            6.0,
            6.0,
            Motor.PhysicalSpec.GOBILDA_5202_0002_0005,
            TwoWheelOdometry.ReversalPattern(),
            TwoWheelOdometry.Ids.default
    )
    val encoder = Motor(
            "enc",
            Motor.PhysicalSpec.REV_THROUGH_BORE_ENCODER,
            Motor.Config.default
    )

    inner class Impl(hardwareMap: IHardwareMap) {
        private val drive = DriveController(
                this@RobotA.drive.Impl(hardwareMap),
                this@RobotA.odo.also { it.Impl(hardwareMap) },
        )
        private val encoder = this@RobotA.encoder.Impl(hardwareMap)

        fun init() {
            println(encoder.pos)
            // could do something here, just as an example
        }

        fun doThing() {
            println(encoder.pos)
            drive.currentTarget = Vec2Rot(Vec2(Math.random(), Math.random()) * 12.0, 0.0)
        }

        fun tick(dt: Double) {
            drive.tick(dt)
        }
    }

    override fun impl(hardwareMap: IHardwareMap) = Impl(hardwareMap)

    companion object {
        val instance = RobotA()
    }
}