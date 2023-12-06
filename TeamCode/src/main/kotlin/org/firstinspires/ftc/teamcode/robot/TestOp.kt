package org.firstinspires.ftc.teamcode.robot

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.Disabled
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import kotlinx.coroutines.delay
import org.firstinspires.ftc.teamcode.LOpMode
import org.firstinspires.ftc.teamcode.controlSystems.DriveController
import org.firstinspires.ftc.teamcode.controlSystems.PID1D
import org.firstinspires.ftc.teamcode.controlSystems.motionPath.MotionPath
import org.firstinspires.ftc.teamcode.controlSystems.motionPath.buildPath
import org.firstinspires.ftc.teamcode.ftcGlue.IHardwareMap
import org.firstinspires.ftc.teamcode.ftcGlue.IRobot
import org.firstinspires.ftc.teamcode.hardware.Motor
import org.firstinspires.ftc.teamcode.hardware.motion.AssumptionOdometry
import org.firstinspires.ftc.teamcode.hardware.motion.MecanumDrive
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
        watches(gamepadA.dpad.up::Watch) {
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
        bezierXY(Lasting(1.0.seconds), Vec2.zero, Vec2.zero)
        bezierR(Lasting(1.0.seconds), 0.0, 0.0)

        bezierXY(UntilAt(2.0.seconds), Vec2(6.0,24.0), Vec2(0.0, 24.0))
        stationaryR(UntilAt(2.0.seconds))

        bezierR(UntilAt(3.0.seconds), Math.PI)
        bezierXY(UntilAt(3.0.seconds), Vec2(6.0, 60.0), Vec2(0.0,0.0))

        bezierXY(UntilAt(4.0.seconds), Vec2.zero)
        stationaryR(UntilAt(4.0.seconds))

        bezierR(UntilAt(5.0.seconds), 0.0)

        stationaryUntilAt(6.0.seconds)
    })

    robot.halt()
    println("!!!!!!!!!!!!!!!! EXIT")
}, FinishedBehaviour.STOP)

class RobotA : IRobot<RobotA.Impl> {
    val drive = MecanumDrive(
            Vec2Rot.zero,
            Motor.PhysicalSpec.GOBILDA_5202_0002_0005,
            MecanumDrive.ReversalPattern(
                    right = true
            ),
            MecanumDrive.Ids.default,
    )
    val dbgOdo = AssumptionOdometry()
//    val odo = TwoWheelOdometry(
//            Vec2Rot.zero,
//            6.0,
//            6.0,
//            Motor.PhysicalSpec.GOBILDA_5202_0002_0005,
//            TwoWheelOdometry.ReversalPattern(),
//            TwoWheelOdometry.Ids.default
//    )
//    val encoder = Motor(
//            "enc",
//            Motor.PhysicalSpec.REV_THROUGH_BORE_ENCODER,
//            Motor.Config.default
//    )

    inner class Impl(hardwareMap: IHardwareMap) {
//        private val odo = this@RobotA.odo.also { it.Impl(hardwareMap) }
        private val driveDrive = this@RobotA.drive.Impl(hardwareMap)
        private val odo = this@RobotA.dbgOdo.also { it.drive = driveDrive }
        private val drive = DriveController(
                driveDrive,
                odo,
                PID1D.Coefficients(0.5,0.5,0.5,4.0,0.0,0.0)
        )
//        private val encoder = this@RobotA.encoder.Impl(hardwareMap)

        fun init() {
//            println(encoder.pos)
            odo.assertPosition(Vec2Rot.zero)
            // could do something here, just as an example
        }

        fun doThing() {
//            println(encoder.pos)
            drive.targetPos = Vec2Rot(Vec2(Math.random(), Math.random()) * 12.0, 0.0)
        }

        val pos by odo::pos
        val vel by odo::vel
        suspend fun followPathCompletely(path: MotionPath<Vec2Rot>) {
            drive.path = path
            drive.t = 0.0
            println("HELLO WORLD START")
            delay(path.validUntilT.seconds)
            println("HELLO WORLD END")
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