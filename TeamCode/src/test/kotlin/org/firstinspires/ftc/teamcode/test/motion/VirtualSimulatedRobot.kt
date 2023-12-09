package org.firstinspires.ftc.teamcode.test.motion

import org.firstinspires.ftc.teamcode.hardware.motion.IDrive
import org.firstinspires.ftc.teamcode.hardware.motion.IOdometry
import org.firstinspires.ftc.teamcode.util.METER_TO_IN
import org.firstinspires.ftc.teamcode.util.Vec2Rot
import kotlin.math.*

private const val GRAVITY_ACCEL = 9.807

class VirtualSimulatedRobot(
        private val mass: Double,
        private val momentOfInertia: Double,
        private val muStatic: Double,
        private val muKinetic: Double,
        private val getForce: (Vec2Rot, Vec2Rot, VirtualSimulatedRobot) -> Vec2Rot = { p, v, r -> defaultGetForce(p, v,r) },
) : IOdometry(), IDrive {
    override var power = Vec2Rot.zero

    private var realPos = Vec2Rot.zero
    private var realVel = Vec2Rot.zero
    private var realAcc = Vec2Rot.zero

    private fun tickPhysics(dt: Double) {
        var force = robot2worldTransform().transformVelFwd(getForce(power, realVel, this))

        force -= if (realVel.v.magSq < 0.01.pow(2) && realVel.r.absoluteValue < 0.01.pow(2)) {
            // static
            Vec2Rot(
                    force.v.norm * min(force.v.mag, mass * GRAVITY_ACCEL * muStatic),
                    sign(force.r) * min(force.r, mass * GRAVITY_ACCEL * muStatic),
            )
        } else {
            // kinetic
            Vec2Rot(
                    realVel.v.norm * (mass * GRAVITY_ACCEL * muKinetic),
                    sign(realVel.r) * (mass * GRAVITY_ACCEL * muKinetic),
            )
        }

        realAcc = Vec2Rot(
                force.v / mass,
                force.r / momentOfInertia, // TODO proper torque handling, it's never this clean
        )
        realPos += realVel * dt / 2.0
        realVel += realAcc * dt
        realPos += realVel * dt / 2.0
    }

    override fun tick(dt: Double) {
        val physicsSubTicks = 10
        for (i in 0 until physicsSubTicks) {
            tickPhysics(dt / physicsSubTicks)
        }

        pos = realPos * METER_TO_IN
        vel = realVel * METER_TO_IN
        acc = realAcc * METER_TO_IN
    }

    override fun assertPosition(newPos: Vec2Rot) {
        throw Error("NOT USED IN THIS TEST")
    }

    override fun nudge(newPos: Vec2Rot) {
        throw Error("NOT USED IN THIS TEST")
    }

    companion object {
        fun defaultGetForce(power: Vec2Rot, vel: Vec2Rot, simulatedRobot: VirtualSimulatedRobot): Vec2Rot {
            val linearTopSpeed = 2
            val angularTopSpeed = 5.0

            val maxLinearVel = (4 - exp(1.45 - power.v.mag)) / 4.0 * linearTopSpeed
            val friction = GRAVITY_ACCEL * simulatedRobot.mass * simulatedRobot.muKinetic
            val forceMag = exp((maxLinearVel - vel.v.mag) / linearTopSpeed * 5.0) * friction
            val maxRotVel = (4 - exp(1.45 - power.r)) / 4.0 * angularTopSpeed
            val torqueMag = exp(((maxRotVel - vel.v.mag) / linearTopSpeed * 5.0) * 2.0) * friction
            return Vec2Rot(
                    power.v.norm * forceMag * simulatedRobot.mass,
                    sign(power.r) * torqueMag * simulatedRobot.momentOfInertia,
            )
        }
    }
}