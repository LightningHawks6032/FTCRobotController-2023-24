package org.firstinspires.ftc.teamcode.robot.arbot.subassemblies

import org.firstinspires.ftc.teamcode.controlSystems.ActuatorPositionController
import org.firstinspires.ftc.teamcode.controlSystems.PID1D
import org.firstinspires.ftc.teamcode.ftcGlue.IHardwareMap
import org.firstinspires.ftc.teamcode.hardware.Motor
import org.firstinspires.ftc.teamcode.util.NotForCompetition
import org.firstinspires.ftc.teamcode.util.delegate
import org.firstinspires.ftc.teamcode.util.withWriteEffect


class ArBotIntake(
        doReverse: Boolean = false,
        val forceScale: Double,
        private val pidCoefficients: PID1D.Coefficients,
) {
    private val anglerRef = Motor("a", Motor.PhysicalSpec.GOBILDA_5202_0002_0005, Motor.Config { reversed = doReverse })
    private val spinnerRef = Motor("s", Motor.PhysicalSpec.GOBILDA_5202_0002_0001, Motor.Config { reversed = doReverse.not() })


    inner class Impl(hardware: IHardwareMap) {
        private val angler = anglerRef.Impl(hardware)
        private val spinner = spinnerRef.Impl(hardware)

        private fun onEditZeroPos(it: () -> Unit) {
            angleController.posZeroPointEdit(it)
        }

        var pos by angler::pos.delegate().withWriteEffect { onEditZeroPos(it) }

        val angleController = ActuatorPositionController(pidCoefficients, angler::setTorque, this::pos, angler::pos, forceScale)
        fun tick(dt: Double) = angleController.tick(dt)

        var spinPower by spinner::power

        @NotForCompetition
        var debugAnglePower by spinner::power
    }
}