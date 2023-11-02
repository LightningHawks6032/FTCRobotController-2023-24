package org.firstinspires.ftc.teamcode.robot.barthalomew

import com.qualcomm.robotcore.eventloop.opmode.Disabled
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.LOpMode
import org.firstinspires.ftc.teamcode.util.Vec2
import org.firstinspires.ftc.teamcode.util.Vec2Rot

@TeleOp
@Disabled
class BarthalomewDriveOp : LOpMode<BarthalomewRobot.Impl>(BarthalomewRobot.main, {
    waitForStart()

    withTelemetry {

    }

    createLoop {//lol imagine coding
        val speedButton = gamepadA.bumper.let { it.left.isHeld || it.right.isHeld }
        val speed = if (speedButton) 0.5 else 0.25

        robot.drive(Vec2Rot(
                v = gamepadA.stick.left.pos.let { Vec2(it.y, it.x) },
                r = gamepadA.stick.right.pos.x,
        ) * speed)
    }


})