package org.firstinspires.ftc.teamcode.ftcGlue.ftcImpl

import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorSimple
import org.firstinspires.ftc.teamcode.ftcGlue.IDCMotor

class LIDCMotor(private val dcMotor: DcMotor) : IDCMotor {
    override var power by dcMotor::power
    override val pos by dcMotor::currentPosition

    init {
        // Always carry same config, so better config can be built on top.
        dcMotor.direction = DcMotorSimple.Direction.FORWARD
        dcMotor.mode = DcMotor.RunMode.RUN_USING_ENCODER
    }
}