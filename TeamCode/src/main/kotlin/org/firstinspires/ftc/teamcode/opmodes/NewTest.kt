package org.firstinspires.ftc.teamcode.opmodes

import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.LOpMode
import org.firstinspires.ftc.teamcode.ftcGlue.IHardwareMap
import org.firstinspires.ftc.teamcode.ftcGlue.IRobot

object EmptyRobot : IRobot<EmptyRobot.Impl> {
    override fun impl(hardwareMap: IHardwareMap) = Impl
    object Impl {}
}

@TeleOp
class NewTest : LOpMode<EmptyRobot.Impl>(EmptyRobot, {
    withTelemetry {
        ln("hello world")
    }
})