package org.firstinspires.ftc.teamcode.opmodes.hbot

import org.firstinspires.ftc.teamcode.ftcGlue.IHardwareMap
import org.firstinspires.ftc.teamcode.hardware.Motor


class HBotInTake() {
    //O.O

    val armA = Motor("10million", Motor.PhysicalSpec.GOBILDA_5202_0002_0005, Motor.Config {reversed = true})
    val armB = Motor("10billion", Motor.PhysicalSpec.GOBILDA_5202_0002_0005, )
    inner class Impl(hardware: IHardwareMap){


        val armA = this@HBotInTake.armA.Impl(hardware)
        val armB = this@HBotInTake.armB.Impl(hardware)

        fun setPower(power: Double){
            armA.power = power
            armB.power = power
        }

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