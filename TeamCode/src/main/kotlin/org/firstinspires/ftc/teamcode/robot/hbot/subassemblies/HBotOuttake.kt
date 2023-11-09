package org.firstinspires.ftc.teamcode.robot.hbot.subassemblies

import org.firstinspires.ftc.teamcode.controlSystems.ActuatorPositionController
import org.firstinspires.ftc.teamcode.controlSystems.PID1D
import org.firstinspires.ftc.teamcode.ftcGlue.IHardwareMap
import org.firstinspires.ftc.teamcode.hardware.Motor
import org.firstinspires.ftc.teamcode.util.delegate
import org.firstinspires.ftc.teamcode.util.times
import org.firstinspires.ftc.teamcode.util.withWriteEffect

const val IN_PER_RAD_OUTTAKE_SLIDES = 1.0 // TODO

class HBotOuttake(
        doReverse: Boolean = false,
        private val pidCoefficients: PID1D.Coefficients,
) {
    private val lifter0Ref = Motor("l0", Motor.PhysicalSpec.GOBILDA_5202_0002_0003, Motor.Config { reversed = doReverse })
    private val lifter1Ref = Motor("l1", Motor.PhysicalSpec.GOBILDA_5202_0002_0003, Motor.Config { reversed = doReverse.not() })

    inner class Impl(hardwareMap: IHardwareMap) {
        private val lifter0 = lifter0Ref.Impl(hardwareMap)
        private val lifter1 = lifter1Ref.Impl(hardwareMap)

        private fun onEditZeroPos(it: ()->Unit) {
            controller.posZeroPointEdit(it)
        }

        var pos by lifter0::pos.delegate()
                .withWriteEffect { onEditZeroPos(it) }
                .times { IN_PER_RAD_OUTTAKE_SLIDES }
        var power = 0.0
            set(power) {
                field = power
                lifter0.power = power
                lifter1.power = power
            }

        val controller = ActuatorPositionController(pidCoefficients, this::power, this::pos)
        fun tick(dt: Double) = controller.tick(dt)
    }
}