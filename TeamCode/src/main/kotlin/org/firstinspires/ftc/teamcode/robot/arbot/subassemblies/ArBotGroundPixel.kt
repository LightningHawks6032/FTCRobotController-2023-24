package org.firstinspires.ftc.teamcode.robot.arbot.subassemblies

import org.firstinspires.ftc.teamcode.ftcGlue.IHardwareMap
import org.firstinspires.ftc.teamcode.hardware.Servo
import org.firstinspires.ftc.teamcode.util.*

class ArBotGroundPixel(
        private val servoRange: DelegateRange,
        val locationOnRobot: Vec2,
        ) {
    private val servoRef = Servo("ground pixel", continuousRotation = false, Servo.Config { reversed = false })

    inner class Impl(hardwareMap: IHardwareMap) {
        private val servo = servoRef.Impl(hardwareMap)
        val locationOnRobot = this@ArBotGroundPixel.locationOnRobot

        private var servoPos by servo::pos.delegate().remapRange(servoRange, DelegateRange(-1.0, 1.0))
        var hold = false
        set(value) {
            field = value
            servoPos = if (field) 1.0 else -1.0
        }
        @NotForCompetition
        fun debugControlServo() = servo

        init {
            // reset servo positions
            hold = false
        }
    }
}