package org.firstinspires.ftc.teamcode.robot.hbot.subassemblies

import org.firstinspires.ftc.teamcode.controlSystems.ActuatorPositionController
import org.firstinspires.ftc.teamcode.controlSystems.PID1D
import org.firstinspires.ftc.teamcode.ftcGlue.IHardwareMap
import org.firstinspires.ftc.teamcode.hardware.Motor
import org.firstinspires.ftc.teamcode.util.delegate
import org.firstinspires.ftc.teamcode.util.withWriteEffect


class HBotIntake(
        doReverse: Boolean = false,
        private val pidCoefficients: PID1D.Coefficients,
) {
    private val armRRef = Motor("ar", Motor.PhysicalSpec.GOBILDA_5202_0002_0005, Motor.Config { reversed = doReverse })
    private val armLRef = Motor("al", Motor.PhysicalSpec.GOBILDA_5202_0002_0005, Motor.Config { reversed = doReverse.not() })

    inner class Impl(hardware: IHardwareMap) {
        private val armR = armRRef.Impl(hardware)
        private val armL = armLRef.Impl(hardware)

        private fun onEditZeroPos(it: ()->Unit) {
            controller.posZeroPointEdit(it)
        }

        var pos by armR::pos.delegate().withWriteEffect { onEditZeroPos(it) }
        var power = 0.0
            set(power) {
                field = power
                armR.power = power
                armL.power = power
            }

        val controller = ActuatorPositionController(pidCoefficients, this::power, this::pos)
        fun tick(dt: Double) = controller.tick(dt)

    }
}