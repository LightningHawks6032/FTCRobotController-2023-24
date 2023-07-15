package org.firstinspires.ftc.teamcode.opmodes

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.Disabled
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import kotlinx.coroutines.delay
import org.firstinspires.ftc.teamcode.LOpMode
import org.firstinspires.ftc.teamcode.controlSystems.DriveController
import org.firstinspires.ftc.teamcode.controlSystems.motionPath.MotionPath
import org.firstinspires.ftc.teamcode.controlSystems.motionPath.buildPath
import org.firstinspires.ftc.teamcode.ftcGlue.IHardwareMap
import org.firstinspires.ftc.teamcode.ftcGlue.IRobot
import org.firstinspires.ftc.teamcode.hardware.Motor
import org.firstinspires.ftc.teamcode.hardware.motion.MecanumDrive
import org.firstinspires.ftc.teamcode.hardware.motion.TwoWheelOdometry
import org.firstinspires.ftc.teamcode.util.Vec2
import org.firstinspires.ftc.teamcode.util.Vec2Rot
import kotlin.time.Duration.Companion.seconds

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

@Autonomous
class TestAuto : LOpMode<RobotA.Impl>(RobotA.instance, {
    robot.init()

    waitForStart()

    createLoop {
        robot.tick(dt)
    }

    robot.followPathCompletely(buildPath(
            startPos = robot.pos,
            startVel = robot.vel,
    ) {
        bezierXY(Lasting(0.1.seconds), Vec2.zero, Vec2.zero)
        bezierR(Lasting(0.1.seconds), 0.0, 0.0)

        bezierXY(UntilAt(1.0.seconds), Vec2(6.0,24.0), Vec2(0.0, 24.0))
        stationaryR(UntilAt(1.0.seconds))

        bezierR(UntilAt(2.0.seconds), Math.PI)
        bezierXY(UntilAt(2.0.seconds), Vec2(6.0, 60.0), Vec2(0.0,0.0))

        bezierXY(UntilAt(4.0.seconds), Vec2.zero)
        stationaryR(Lasting(1.0.seconds))
        bezierR(UntilAt(4.0.seconds), 0.0)

        stationaryUntilAt(5.0.seconds)
    })

    robot.halt()
}, FinishedBehaviour.STOP)

class RobotA : IRobot<RobotA.Impl> {
    val drive = MecanumDrive(
            Motor.PhysicalSpec.GOBILDA_5202_0002_0005,
            MecanumDrive.ReversalPattern(
                    right = true
            ),
            MecanumDrive.Ids.default,
            Vec2Rot.zero,
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
        private val odo = this@RobotA.odo.also { it.Impl(hardwareMap) }
        private val drive = DriveController(
                this@RobotA.drive.Impl(hardwareMap),
                odo
        )
        private val encoder = this@RobotA.encoder.Impl(hardwareMap)

        fun init() {
            println(encoder.pos)
            odo.assertPosition(Vec2Rot.zero)
            // could do something here, just as an example
        }

        fun doThing() {
            println(encoder.pos)
            drive.targetPos = Vec2Rot(Vec2(Math.random(), Math.random()) * 12.0, 0.0)
        }

        val pos by odo::pos
        val vel by odo::vel
        suspend fun followPathCompletely(path: MotionPath<Vec2Rot>) {
            drive.path = path
            drive.t = 0.0
            delay(path.validUntilT.seconds)
        }

        var halted = false
        fun halt() {
            halted = true
            drive.shutdownOutput()
        }

        fun tick(dt: Double) {
            if (halted) {
                println("TICKING AFTER HALT!!!")
                return
            }
            drive.tick(dt)
        }
    }

    override fun impl(hardwareMap: IHardwareMap) = Impl(hardwareMap)

    companion object {
        val instance = RobotA()
    }
}