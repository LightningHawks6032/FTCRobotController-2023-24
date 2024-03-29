package org.firstinspires.ftc.teamcode.robot.arbot.subassemblies

import org.firstinspires.ftc.teamcode.ftcGlue.IHardwareMap
import org.firstinspires.ftc.teamcode.hardware.Motor

class ArBotLifter() {
//    private val servoRef = Servo("lifter", continuousRotation = false, Servo.Config { reversed = false })
    private val lifterRef = Motor("up", Motor.PhysicalSpec.GOBILDA_5202_0002_0005, Motor.Config { })

    inner class Impl(hardwareMap: IHardwareMap) {
//        private val servo = servoRef.Impl(hardwareMap)
//        private val servo = hardwareMap.getRaw("lifter", CRServo::class) ?: TOD O("cr servo 'lifter' not found")
        private val lifter = lifterRef.Impl(hardwareMap)

//        var raiseArm = 0.0
//            set(value) {
//                field = value
//
//                servo.power = field * 0.15
//            }
        var liftPower = 0.0
            set(value) {
                field = value
                lifter.power = field
            }
//        @NotForCompetition
//        fun debugControlServo() = servo

        init {
            // reset servo positions
//            raiseArm = 0.0
            liftPower = 0.0
        }
    }
}