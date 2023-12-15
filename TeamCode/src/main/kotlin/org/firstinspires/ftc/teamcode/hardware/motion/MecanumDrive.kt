package org.firstinspires.ftc.teamcode.hardware.motion

import androidx.core.math.MathUtils
import org.firstinspires.ftc.teamcode.ftcGlue.IHardwareMap
import org.firstinspires.ftc.teamcode.hardware.Motor
import org.firstinspires.ftc.teamcode.util.CM_TO_IN
import org.firstinspires.ftc.teamcode.util.Transform2D
import org.firstinspires.ftc.teamcode.util.Vec2Rot
import kotlin.math.min
import kotlin.math.sqrt

class MecanumDrive(
        assemblyLocationRobotSpace: Vec2Rot,
        motorSpec: Motor.PhysicalSpec,
        reversalPattern: ReversalPattern,
        /** The radius of any given wheel in inches. */
        wheelRadiusInches: Double,
        /** The straight line distance from the center of this assembly to any given wheel in inches. */
        wheelDisplacementInches: Double,
        val ids: Ids,
) {
    val frRef = Motor(ids.fr, motorSpec, Motor.Config { reversed = reversalPattern.fr })
    val flRef = Motor(ids.fl, motorSpec, Motor.Config { reversed = reversalPattern.fl })
    val blRef = Motor(ids.bl, motorSpec, Motor.Config { reversed = reversalPattern.bl })
    val brRef = Motor(ids.br, motorSpec, Motor.Config { reversed = reversalPattern.br })

    val assembly2robotTransform = Transform2D.local2outerFromLocation(assemblyLocationRobotSpace)

    /** The radius of any given wheel in centimeters. */
    val wheelRadius = wheelRadiusInches * 2.54

    /** The straight line distance from the center of this assembly to any given wheel in centimeters. */
    val wheelDisplacement = wheelDisplacementInches * 2.54 / 100

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
            val default = Ids("fr", "fl", "br", "bl")
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
                val (x, y) = localSpacePower.v.norm * min(1.0, localSpacePower.v.mag)

                fr.power = x + y + r
                fl.power = x - y - r
                br.power = x - y + r
                bl.power = x + y - r
            }

        override fun setForce(force: Vec2Rot, currentVel: Vec2Rot) {
            val localSpaceForce = assembly2robotTransform.transformVelInv(force)
            val localSpaceVel = assembly2robotTransform.transformVelInv(currentVel)

            val w = localSpaceVel.r
            val v = localSpaceVel.v

            val t = localSpaceForce.r
            val f = localSpaceForce.v

            // -> rad/s
            // = ( in/s * (cm / in) + rad/s * cm) / cm * 1
            val frVel = ((v.x + v.y) / CM_TO_IN + w * wheelDisplacement) / wheelRadius * sqrt(2.0)
            val flVel = ((v.x - v.y) / CM_TO_IN - w * wheelDisplacement) / wheelRadius * sqrt(2.0)
            val brVel = ((v.x - v.y) / CM_TO_IN + w * wheelDisplacement) / wheelRadius * sqrt(2.0)
            val blVel = ((v.x + v.y) / CM_TO_IN - w * wheelDisplacement) / wheelRadius * sqrt(2.0)

            // -> N cm
            // = (N + N + Ncm / cm) * cm
            fr.setTorque((f.x + f.y + t / wheelDisplacement) * wheelRadius, frVel)
            fl.setTorque((f.x - f.y - t / wheelDisplacement) * wheelRadius, flVel)
            br.setTorque((f.x - f.y + t / wheelDisplacement) * wheelRadius, brVel)
            bl.setTorque((f.x + f.y - t / wheelDisplacement) * wheelRadius, blVel)

        }
    }
}