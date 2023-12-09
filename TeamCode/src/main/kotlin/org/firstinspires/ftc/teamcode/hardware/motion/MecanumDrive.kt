package org.firstinspires.ftc.teamcode.hardware.motion

import androidx.core.math.MathUtils
import org.firstinspires.ftc.teamcode.ftcGlue.IHardwareMap
import org.firstinspires.ftc.teamcode.hardware.Motor
import org.firstinspires.ftc.teamcode.util.Transform2D
import org.firstinspires.ftc.teamcode.util.Vec2Rot
import kotlin.math.min

class MecanumDrive(
        assemblyLocationRobotSpace: Vec2Rot,
        motorSpec: Motor.PhysicalSpec,
        reversalPattern: ReversalPattern,
        val ids: Ids,
) {
    val frRef = Motor(ids.fr, motorSpec, Motor.Config { reversed = reversalPattern.fr })
    val flRef = Motor(ids.fl, motorSpec, Motor.Config { reversed = reversalPattern.fl })
    val blRef = Motor(ids.bl, motorSpec, Motor.Config { reversed = reversalPattern.bl })
    val brRef = Motor(ids.br, motorSpec, Motor.Config { reversed = reversalPattern.br })

    val assembly2robotTransform = Transform2D.local2outerFromLocation(assemblyLocationRobotSpace)

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
                val localSpacePower = assembly2robotTransform.transformVelInv(newPower)
                val r = MathUtils.clamp(localSpacePower.r, -1.0, 1.0)
                val (x,y) = localSpacePower.v.norm * min(1.0, localSpacePower.v.mag)

                fr.power = x + y + r
                fl.power = x - y - r
                br.power = x - y + r
                bl.power = x + y - r
            }
    }
}