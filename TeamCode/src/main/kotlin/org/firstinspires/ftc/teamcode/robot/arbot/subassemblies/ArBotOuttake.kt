package org.firstinspires.ftc.teamcode.robot.arbot.subassemblies

import org.firstinspires.ftc.teamcode.controlSystems.ActuatorPositionController
import org.firstinspires.ftc.teamcode.controlSystems.PID1D
import org.firstinspires.ftc.teamcode.ftcGlue.IHardwareMap
import org.firstinspires.ftc.teamcode.hardware.Motor
import org.firstinspires.ftc.teamcode.hardware.Servo
import org.firstinspires.ftc.teamcode.hardware.TandemGroup
import org.firstinspires.ftc.teamcode.util.*

private const val IN_PER_RAD_OUTTAKE_SLIDES = 1.0 // TODO

class ArBotOuttake(
        doReverse: Boolean = false,
//        private val tiltServoRange: DelegateRange,
        private val dropServoRange: DelegateRange,
        val forceScale: Double,
        private val pidCoefficients: PID1D.Coefficients,
) {
    private val lifterRef = TandemGroup.Motor(
            Motor("l0", Motor.PhysicalSpec.GOBILDA_5202_0002_0003, Motor.Config {
                reversed = doReverse
                minTorque = 2.0
            }),
            Motor("l1", Motor.PhysicalSpec.GOBILDA_5202_0002_0003, Motor.Config {
                reversed = doReverse.not()
                minTorque = 2.0
            }),
    )
//    private val tiltServoRef = Servo("t", continuousRotation = false, Servo.Config { reversed = false })
    private val dropServoRef = Servo("drop", continuousRotation = false, Servo.Config { reversed = true })

    private var outtakeTilt = false

    inner class Impl(hardwareMap: IHardwareMap) {
        private val lifter = lifterRef.Impl(hardwareMap)
//        private val tiltServo = tiltServoRef.Impl(hardwareMap)
        private val dropServo = dropServoRef.Impl(hardwareMap)

        private fun onEditZeroPos(it: ()->Unit) {
            controller.posZeroPointEdit(it)
        }

        var pos by lifter::pos.delegate()
                .withWriteEffect { doEdit -> onEditZeroPos(doEdit) }
                .times { IN_PER_RAD_OUTTAKE_SLIDES }

        val controller = ActuatorPositionController(
                pidCoefficients,
                lifter::setTorque, this::pos, lifter::pos,
                lifterRef.leaderMotorSpec,
                IN_PER_RAD_OUTTAKE_SLIDES,
                forceScale,
        )
        fun tick(dt: Double) {
            controller.tick(dt, shutdownIf = { pos, target -> pos < 0.3 && target < 0.3 })
            println("pos $pos, target ${controller.targetPosition}")
        }

//        private var tiltServoPos by tiltServo::pos.delegate().remapRange(tiltServoRange, DelegateRange(-1.0,1.0))
        private var dropServoPos by dropServo::pos.delegate().remapRange(dropServoRange, DelegateRange(-1.0,1.0))
//        var outtakeTilt by this@ArBotOuttake::outtakeTilt.delegate().withAfterWriteEffect { tilt ->
//            tiltServoPos = if (tilt) 1.0 else -0.8
//        }
        var dropOpen by this@ArBotOuttake::outtakeTilt.delegate().withAfterWriteEffect { open ->
            dropServoPos = if (open) 1.0 else -1.0
        }
//        @NotForCompetition
//        fun debugControlTiltServo() = tiltServo
        @NotForCompetition
        fun debugControlDropServo() = dropServo
        @NotForCompetition
        var debugLifterPower by lifter::power

        init {
            // reset servo positions
            outtakeTilt = this@ArBotOuttake.outtakeTilt
            dropOpen = false
        }
    }
}