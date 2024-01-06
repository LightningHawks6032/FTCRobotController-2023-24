package org.firstinspires.ftc.teamcode.robot.arbot.subassemblies

import org.firstinspires.ftc.teamcode.ftcGlue.IHardwareMap
import org.firstinspires.ftc.teamcode.hardware.Servo
import org.firstinspires.ftc.teamcode.util.DelegateRange
import org.firstinspires.ftc.teamcode.util.NotForCompetition
import org.firstinspires.ftc.teamcode.util.delegate
import org.firstinspires.ftc.teamcode.util.remapRange

class ArBotPlaneLauncher(
        private val servoRange: DelegateRange,
) {
    private val servoRef = Servo("c", continuousRotation = false, Servo.Config { reversed = false })

    inner class Impl(hardwareMap: IHardwareMap) {
        private val servo = servoRef.Impl(hardwareMap)

        private var servoPos by servo::pos.delegate().remapRange(servoRange, DelegateRange(-1.0, 1.0))
        var launch = false
        set(value) {
            field = value
            servoPos = if (field) 1.0 else -1.0
        }
        @NotForCompetition
        fun debugControlServo() = servo

        init {
            // reset servo positions
            launch = false
        }
    }
}