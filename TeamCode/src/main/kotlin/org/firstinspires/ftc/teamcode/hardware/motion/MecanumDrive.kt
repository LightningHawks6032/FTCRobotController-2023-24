package org.firstinspires.ftc.teamcode.hardware.motion

import org.firstinspires.ftc.teamcode.ftcGlue.IHardwareMap
import org.firstinspires.ftc.teamcode.hardware.Motor
import org.firstinspires.ftc.teamcode.util.LocTransform
import org.firstinspires.ftc.teamcode.util.Vec2Rot

class MecanumDrive(
        motorSpec: Motor.PhysicalSpec,
        reversalPattern: ReversalPattern,
        ids: Ids,
        locationOnRobot: Vec2Rot,
) {
    val frRef = Motor(ids.fr, motorSpec, Motor.Config { reversed = reversalPattern.fr })
    val flRef = Motor(ids.fl, motorSpec, Motor.Config { reversed = reversalPattern.fl })
    val brRef = Motor(ids.br, motorSpec, Motor.Config { reversed = reversalPattern.br })
    val blRef = Motor(ids.bl, motorSpec, Motor.Config { reversed = reversalPattern.bl })

    val transform = LocTransform.Ground.Transform(locationOnRobot)

    data class ReversalPattern(
            val all: Boolean = false,
            val left: Boolean = false,
            val right: Boolean = false,
            val front: Boolean = false,
            val back: Boolean = false,
    ) {
        val fr get() = all xor right xor front
        val fl get() = all xor left xor front
        val br get() = all xor right xor back
        val bl get() = all xor left xor back
    }
    data class Ids(
            val fr: String,
            val fl: String,
            val br: String,
            val bl: String,
    ) {
        companion object {
            val default = Ids("fr","fl","br","bl")
        }
    }

    inner class Impl(hardwareMap: IHardwareMap) : IDrive {
        private val fr = frRef.Impl(hardwareMap)
        private val fl = flRef.Impl(hardwareMap)
        private val br = brRef.Impl(hardwareMap)
        private val bl = blRef.Impl(hardwareMap)

        override var power: Vec2Rot = Vec2Rot.zero
            set(newPower) {
                field = newPower
                val localSpacePower = transform.globalToLocalVel(newPower)
                val (_,r) = localSpacePower
                val (x,y) = localSpacePower.v

                fr.power = y - x + r
                fl.power = y + x - r
                br.power = y + x + r
                bl.power = y - x - r
            }
    }
}