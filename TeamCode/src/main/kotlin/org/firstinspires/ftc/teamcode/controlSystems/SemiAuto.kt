package org.firstinspires.ftc.teamcode.controlSystems

import androidx.core.math.MathUtils
import org.firstinspires.ftc.teamcode.util.Vec2
import org.firstinspires.ftc.teamcode.util.Vec2Rot
import kotlin.math.exp
import kotlin.math.pow
import kotlin.math.sign
import kotlin.math.sqrt

class SemiAuto(
        private val drive: DriveController,
        private val obstacleSDF: (Vec2) -> Double,
        private val reaction: ReactionConfig,
        private val powerScale: Vec2Rot = Vec2Rot(1.0, 1.0, 0.5),
) {
    class ReactionConfig(
            val robotMass: Double,
            val decelerateRadius: Double,
            val repelRadius: Double,
            val repelForce: Double,
    )

    /**
     *
     * Outputs force and torque contribution of this point
     */
    private fun calculateObjectAvoidanceForce(atRelative: Vec2): Pair<Vec2Rot, Double> {
        val pos = drive.inputPos
        val vel = drive.inputVel

        // signed distance to the nearest obstacle
        val sdf = obstacleSDF(pos.v)

        // sdfGrad is the gradient of sdf at the current position
        // points away from the nearest obstacle
        val sdfGrad = Vec2.grad(obstacleSDF, pos.v, dr = 0.1)

        // x is distance to obstacle, v is velocity directly towards/away from obstacle
        val v = vel.v dot sdfGrad

        val k = 1 - (sdf / reaction.decelerateRadius).coerceIn(0.0, 1.0)
        var f = k * (-v).coerceAtLeast(0.0).pow(2) / sdf * reaction.robotMass
        f += reaction.repelForce * (1 - (sdf / reaction.repelRadius).coerceIn(0.0, 1.0)) * (-v.sign).coerceAtLeast(0.0)
        if (sdf < 0) {
            f = reaction.repelForce
        }

        // get the force applied as a vector and determine how it will apply to the robot
        val fVec = sdfGrad * f
        val linearComponent = atRelative * (atRelative dot fVec) / atRelative.magSq
        val torque = fVec fakeCrossProduct atRelative.norm // TODO might be negative

        // the lower the distance, the more it counts to the avoidance calculation
        // additionally, higher correction forces are prioritized
        val fac = exp(-sdf) * sqrt((linearComponent.magSq + torque.pow(2)))

        return Pair(Vec2Rot(linearComponent, torque), fac)
    }

    private fun calculateObjectAvoidanceForce(): Vec2Rot {
        return Vec2Rot.zero
        var objectAvoidanceForce = Vec2Rot.zero
        var facTotal = 0.001
        val count = 18
        for (i in 0 until count) {
            val k = (i.toDouble() / count - 0.5) * 18.0
            for (point in arrayOf(
                    Vec2(k, 9.0),
                    Vec2(k, -9.0),
                    Vec2(9.0, k),
                    Vec2(-9.0, k),
            )) {
                val (force, fac) = calculateObjectAvoidanceForce(
                        point.rotate(drive.inputPos.r)
                )
                objectAvoidanceForce += force * fac
                facTotal += fac
            }
        }
        return objectAvoidanceForce / facTotal
    }

    fun tick(dt: Double, input: Vec2Rot) {
        drive.setPowerAndTrack(
                (input + calculateObjectAvoidanceForce())
                        componentwiseTimes powerScale,
                dt,
        )
    }

    companion object {
        private fun wallSDF(at: Vec2, normal: Vec2, pos: Vec2) = (pos - at) dot normal
        private fun lineSDF(line: Pair<Vec2, Vec2>, pos: Vec2): Double {
            val (from, to) = line
            val span = to - from
            val localPos = pos - from
            return (localPos - span * MathUtils.clamp((localPos dot span) / span.magSq, 0.0, 1.0)).mag
        }

        private fun nearest(vararg distances: Double) = distances.min()
        private fun Double.expandedBy(expand: Double) = this - expand

        fun centerStageObstacleSDF(pos: Vec2) = nearest(
                // trusses
                nearest(
                        *intArrayOf(-2, -1, 1, 2).map {
                            lineSDF(Vec2(24.0 * it, -24.0) to Vec2(24.0 * it, 0.0), pos)
                        }.toDoubleArray()
                ),
                // walls
                nearest(
                        wallSDF(Vec2(24.0 * 3, 0.0), Vec2(-1.0, 0.0), pos),
                        wallSDF(Vec2(-24.0 * 3, 0.0), Vec2(1.0, 0.0), pos),
                        wallSDF(Vec2(0.0, 24.0 * 3), Vec2(0.0, -1.0), pos),
                        wallSDF(Vec2(0.0, -24.0 * 3), Vec2(0.0, 1.0), pos),
                )
        ).expandedBy(0.5) // 0.5 inch margins around all obstacles
    }
}

