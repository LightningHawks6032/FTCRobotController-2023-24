package org.firstinspires.ftc.teamcode.robot.hbot.subassemblies

import org.firstinspires.ftc.teamcode.ftcGlue.IHardwareMap
import org.firstinspires.ftc.teamcode.hardware.Motor

class HBotOuttake(
        doReverse: Boolean = false,
) {
    private val lifter0Ref = Motor("l0", Motor.PhysicalSpec.GOBILDA_5202_0002_0003, Motor.Config { reversed = doReverse })
    private val lifter1Ref = Motor("l1", Motor.PhysicalSpec.GOBILDA_5202_0002_0003, Motor.Config { reversed = doReverse })

    inner class Impl(hardwareMap: IHardwareMap) {
        private val lifter0 = lifter0Ref.Impl(hardwareMap)
        private val lifter1 = lifter1Ref.Impl(hardwareMap)

        var power = 0.0
            set(power) {
                field = power
                lifter0.power = power
                lifter1.power = power
            }

        // TODO: PID control
    }
}