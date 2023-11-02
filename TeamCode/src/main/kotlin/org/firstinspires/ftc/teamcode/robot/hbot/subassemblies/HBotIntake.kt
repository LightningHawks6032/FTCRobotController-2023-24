package org.firstinspires.ftc.teamcode.robot.hbot.subassemblies

import org.firstinspires.ftc.teamcode.ftcGlue.IHardwareMap
import org.firstinspires.ftc.teamcode.hardware.Motor


class HBotIntake(
        doReverse: Boolean = false,
) {
    private val armRRef = Motor("ar", Motor.PhysicalSpec.GOBILDA_5202_0002_0005, Motor.Config { reversed = doReverse })
    private val armLRef = Motor("al", Motor.PhysicalSpec.GOBILDA_5202_0002_0005, Motor.Config { reversed = doReverse.not() })

    inner class Impl(hardware: IHardwareMap) {
        private val armR = armRRef.Impl(hardware)
        private val armL = armLRef.Impl(hardware)

        var power = 0.0
            set(power) {
                field = power
                armR.power = power
                armL.power = power
            }

        // TODO: PID control
    }
}

/* override var power: Vec2Rot = Vec2Rot.zero
            set(newPower) {
                field = newPower
                val localSpacePower = assembly2robotTransform.transformVelInv(newPower)
                val (_,r) = localSpacePower
                val (x,y) = localSpacePower.v

                fr.power = x + y + r
                fl.power = x - y - r
                br.power = x - y + r
                bl.power = x + y - r

 */