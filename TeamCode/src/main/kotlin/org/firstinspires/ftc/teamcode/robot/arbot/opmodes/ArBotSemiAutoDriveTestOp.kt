package org.firstinspires.ftc.teamcode.robot.arbot.opmodes

import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import kotlinx.coroutines.delay
import org.firstinspires.ftc.teamcode.LOpMode
import org.firstinspires.ftc.teamcode.robot.arbot.ArBotRobot
import org.firstinspires.ftc.teamcode.util.*
import kotlin.math.PI
import kotlin.math.pow


@NotForCompetition
@TeleOp
class ArBotSemiAutoDriveTestOp : LOpMode<ArBotRobot.Impl>(ArBotRobot, {
    val runOdometryNotDriveModeVarName = "mode: odometry (true), drive (false)"
    selectDebugBool(runOdometryNotDriveModeVarName)
    waitForStart()

    val aPressed = TriggerLock(relock = true)
    createLoop {
        watches(gamepadA.a::Watch) {
            it.pressed.bind {
                aPressed.unlock()
            }
        }
    }

    val (odometry, drive) = robot.drive.debugTakeControl()

    createLoop {
        odometry.tick(dt)
    }

    if (DebugVars.bool[runOdometryNotDriveModeVarName] == true) {
        //// Testing Odometry ////
        withTelemetry {
            ln("//// Testing Odometry ////")
            ln("Press A to continue.")
        }
        aPressed.wait()

        // FORWARD (+y)
        withTelemetry {
            ln("Prepare to move the robot FORWARD precisely 12 inches.")
            ln("Make sure the robot is not moving, then press A.")
        }
        aPressed.wait()
        odometry.assertPosition(Vec2Rot.zero)
        withTelemetry {
            ln("Move the robot 12 inches FORWARD, then press A.")
        }
        aPressed.wait()
        val deltaPositiveY = odometry.pos

        // RIGHT (+x)
        withTelemetry {
            ln("Prepare to move the robot RIGHT precisely 12 inches.")
            ln("Make sure the robot is not moving, then press A.")
        }
        aPressed.wait()
        odometry.assertPosition(Vec2Rot.zero)
        withTelemetry {
            ln("Move the robot 12 inches RIGHT, then press A.")
        }
        aPressed.wait()
        val deltaPositiveX = odometry.pos

        // CCW (+r)
        withTelemetry {
            ln("Prepare to rotate the robot COUNTER CLOCKWISE precisely 180 degrees.")
            ln("Make sure the robot is not moving, then press A.")
        }
        aPressed.wait()
        odometry.assertPosition(Vec2Rot.zero)
        withTelemetry {
            ln("Rotate the robot in-place 180 degrees inches COUNTER CLOCKWISE, then press A.")
        }
        aPressed.wait()
        val deltaPositiveR = odometry.pos


        withTelemetry(printToConsole = true) {
            ln("//// Odometry Test Results ////")
            ln()

            ln("Expected (+x)", "${Vec2Rot(Vec2(1.0,0.0),0.0)}")
            ln("Received", "${deltaPositiveX.transformP { it / 12.0 }}")
            ln()
            ln("Expected (+y)", "${Vec2Rot(Vec2(0.0,1.0),0.0)}")
            ln("Received", "${deltaPositiveY.transformP { it / 12.0 }}")
            ln()
            ln("Expected (+r)", "${Vec2Rot(Vec2(0.0,0.0), 1.0)}")
            ln("Received", "${deltaPositiveR / PI}")

            ln("")
            ln("Press A to close.")
        }
        aPressed.wait()
    } else {
        //// Testing Drivetrain (using odometry) ////
        withTelemetry {
            ln("//// Testing Drivetrain (using odometry) ////")
            ln("Press A to continue.")
        }
        aPressed.wait()

        suspend fun rampPower(targetPower: Vec2Rot) {
            var power = 0.0
            while (odometry.pos.let { it.v.magSq / 12.0.pow(2) + it.r.pow(2) / PI.pow(2) } < 1.0) {
                power += 0.05 * 0.25 // ramp up to full in four seconds
                drive.power = targetPower * power
                delay(50) // 1/20 seconds
            }
        }

        // FORWARD (+y)
        withTelemetry {
            ln("The robot will now attempt to drive FORWARD.")
            ln("Make sure robot won't damage anything within about two feet, then press A to begin.")
        }
        aPressed.wait()
        withTelemetry {
            ln("Running...")
        }
        odometry.assertPosition(Vec2Rot.zero)
        rampPower(Vec2Rot(Vec2(0.0,1.0),0.0))
        val deltaPositiveY = odometry.pos

        // RIGHT (+x)
        withTelemetry {
            ln("The robot will now attempt to drive RIGHT.")
            ln("Make sure robot won't damage anything within about two feet, then press A to begin.")
        }
        aPressed.wait()
        withTelemetry {
            ln("Running...")
        }
        odometry.assertPosition(Vec2Rot.zero)
        rampPower(Vec2Rot(Vec2(1.0,0.0),0.0))
        val deltaPositiveX = odometry.pos

        // CCW (+r)
        withTelemetry {
            ln("The robot will now attempt to rotate COUNTER CLOCKWISE.")
            ln("Make sure robot won't damage anything within about two feet, then press A to begin.")
        }
        aPressed.wait()
        withTelemetry {
            ln("Running...")
        }
        odometry.assertPosition(Vec2Rot.zero)
        rampPower(Vec2Rot(Vec2.zero,1.0))
        val deltaPositiveR = odometry.pos

        withTelemetry(printToConsole = true) {
            ln("//// Drive Test Results ////")
            ln()

            ln("Expected (+x)", "${Vec2Rot(Vec2(1.0,0.0),0.0)}")
            ln("Received", "${deltaPositiveX.transformP { it / 12.0 }}")
            ln()
            ln("Expected (+y)", "${Vec2Rot(Vec2(0.0,1.0),0.0)}")
            ln("Received", "${deltaPositiveY.transformP { it / 12.0 }}")
            ln()
            ln("Expected (+r)", "${Vec2Rot(Vec2(0.0,0.0), 1.0)}")
            ln("Received", "${deltaPositiveR / PI}")

            ln()
            ln("Press A to close.")
        }
        aPressed.wait()
    }
})