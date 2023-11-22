package org.firstinspires.ftc.teamcode.robot.arbot.subassemblies

import org.firstinspires.ftc.teamcode.controlSystems.ActuatorPositionController
import org.firstinspires.ftc.teamcode.controlSystems.PID1D
import org.firstinspires.ftc.teamcode.ftcGlue.IHardwareMap
import org.firstinspires.ftc.teamcode.hardware.Motor
import org.firstinspires.ftc.teamcode.hardware.Servo
import org.firstinspires.ftc.teamcode.hardware.TandemGroup
import org.firstinspires.ftc.teamcode.util.NotForCompetition
import org.firstinspires.ftc.teamcode.util.delegate
import org.firstinspires.ftc.teamcode.util.withAfterWriteEffect
import org.firstinspires.ftc.teamcode.util.withWriteEffect


class ArBotIntake(
        doReverse: Boolean = false,
        private val pidCoefficients: PID1D.Coefficients,
) {
    private val armRef = TandemGroup.Motor(
            Motor("ar", Motor.PhysicalSpec.GOBILDA_5202_0002_0005, Motor.Config { reversed = doReverse }),
            Motor("al", Motor.PhysicalSpec.GOBILDA_5202_0002_0005, Motor.Config { reversed = doReverse.not() }),
    )
    private val inputServosRef = TandemGroup.Servo(
            Servo("ir", continuousRotation = false, Servo.Config { reversed = false }), // TODO
            Servo("il", continuousRotation = false, Servo.Config { reversed = true }),
    )
    private val transferServosRef = TandemGroup.Servo(
            Servo("tr", continuousRotation = false, Servo.Config { reversed = false }), // TODO
//            Servo("tl", continuousRotation = false, Servo.Config { reversed = true }),
    )

    private var inputServosOpen = false
    private var transferServosOpen = false

    inner class Impl(hardware: IHardwareMap) {
         val arm = armRef.Impl(hardware)
        private val inputServos = inputServosRef.Impl(hardware)
        private val transferServos = transferServosRef.Impl(hardware)

        private fun onEditZeroPos(it: () -> Unit) {
            controller.posZeroPointEdit(it)
        }

        var pos by arm::pos.delegate().withWriteEffect { onEditZeroPos(it) }
        private var power by arm::power

        val controller = ActuatorPositionController(pidCoefficients, this::power, this::pos)
        fun tick(dt: Double) = controller.tick(dt)

        var inputServosOpen by this@ArBotIntake::inputServosOpen.delegate().withAfterWriteEffect { open ->
            inputServos.pos = if (open) 0.0 else 1.0
        }
        var transferServosOpen by this@ArBotIntake::transferServosOpen.delegate().withAfterWriteEffect { open ->
            transferServos.pos = if (open) 0.0 else -1.0
        }

        @NotForCompetition
        fun debugControlInputServos() = inputServos
        @NotForCompetition
        fun debugControlTransferServos() = transferServos

        init {
            // update the servos
            inputServosOpen = this@ArBotIntake.inputServosOpen
            transferServosOpen = this@ArBotIntake.transferServosOpen
        }
    }
}