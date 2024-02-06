package org.firstinspires.ftc.teamcode.robot.durabot2.opmodes

import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.LOpMode
import org.firstinspires.ftc.teamcode.robot.durabot2.Durabot2Robot
import org.firstinspires.ftc.teamcode.util.NotForCompetition
import org.firstinspires.ftc.teamcode.util.Vec2Rot

@OptIn(NotForCompetition::class)
@TeleOp(group = Durabot2Robot.OP_GROUP_NAME)
class Durabot2Drive : LOpMode<Durabot2Robot.Impl>(Durabot2Robot, {


    waitForStart()

//    var forceMode = false

    val (odometry, drive) = robot.drive.debugTakeControl()
    createLoop {
        odometry.tick(dt)
        withTelemetry {
            ln("pos: ${odometry.pos}")
        }
//        watches(gamepadA.a::Watch) {
//            it.pressed.bind {
//                forceMode = !forceMode
//            }
//        }
        val speedMode = gamepadA.bumper.let { it.left.isHeld || it.right.isHeld }
        val speed = if (speedMode) 1.0 else 0.25
        val pow = (Vec2Rot(
                x = -gamepadA.stick.left.pos.y,
                y = gamepadA.trigger.let { it.left - it.right },
                r = -gamepadA.stick.right.pos.x,
        ) * speed)

//        if (forceMode) {
//            robot.mecanum.setForce(pow)
//        }else{
        drive.power = pow
//        }
    }
})







































