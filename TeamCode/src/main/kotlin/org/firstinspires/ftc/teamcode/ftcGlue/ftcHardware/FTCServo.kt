package org.firstinspires.ftc.teamcode.ftcGlue.ftcHardware

import com.qualcomm.robotcore.hardware.Servo
import org.firstinspires.ftc.teamcode.ftcGlue.IServo

class FTCServo(private val servo: Servo) : IServo {
    override var param by servo::position

    init {
        // Always carry same config, so better config can be built on top.
        servo.direction = Servo.Direction.FORWARD
    }
}