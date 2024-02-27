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
//    odometry.assertPosition(Vec2Rot.zero)
    robot.drive.assertPosition(Vec2Rot.zero)
    createLoop {
        odometry.tick(dt)
        withTelemetry {
            ln("pos: ${robot.drive.inputPos}")
        }
//        watches(gamepadA.a::Watch) {
//            it.pressed.bind {
//                forceMode = !forceMode
//            }
//        }
        val speedMode = gamepadA.bumper.let { it.left.isHeld || it.right.isHeld }
        val speed = if (speedMode) 1.0 else 0.25
        val input = (Vec2Rot(
                x = -gamepadA.stick.left.pos.y,
                y = gamepadA.trigger.let { it.left - it.right },
                r = -gamepadA.stick.right.pos.x,
        ) * speed)
        val tvel = Vec2Rot(input.v * 20.0, input.r * 10.0)

//        if (forceMode) {
//            robot.mecanum.setForce(pow)
//        }else{
        drive.power = input
//        robot.drive.targetPos = tvel * dt + robot.drive.targetPos
//        robot.drive.tick(dt)
//        }
    }
})







































