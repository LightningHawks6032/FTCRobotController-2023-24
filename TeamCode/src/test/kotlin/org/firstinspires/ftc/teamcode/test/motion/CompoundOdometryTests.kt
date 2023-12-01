package org.firstinspires.ftc.teamcode.test.motion

import org.firstinspires.ftc.teamcode.hardware.motion.CompoundOdometry
import org.firstinspires.ftc.teamcode.hardware.motion.IOdometry
import org.firstinspires.ftc.teamcode.util.Vec2Rot
import org.junit.Test
import kotlin.math.absoluteValue
import kotlin.math.sin
import kotlin.math.tanh
import kotlin.random.Random

class CompoundOdometryTests {
    class FakeOdometry : IOdometry() {
        private var realPosLast = pos
        var realPos = pos
        override fun tick(dt: Double) {
            val posLast = pos
            val velLast = vel
            pos += realPos - realPosLast
            vel = (pos - posLast) / dt
            acc = (vel - velLast) / dt
            realPosLast = realPos
        }

        override fun assertPosition(newPos: Vec2Rot) {
            pos = newPos
            vel = Vec2Rot.zero
            acc = Vec2Rot.zero
        }

        override fun nudge(newPos: Vec2Rot) {
            acc = acc.transformP { it.rotate(newPos.r - pos.r) }
            vel = vel.transformP { it.rotate(newPos.r - pos.r) }
            pos = newPos
        }

        var hasPosition = true
        override fun hasPositionInfo() = hasPosition
    }


    @Test
    fun testCompoundOdometryNormally() {
        val detailInput = FakeOdometry()
        val stableInput = FakeOdometry()

        val combined = CompoundOdometry(detailInput, stableInput, 1.0)

        var t = 0.0
        val rng = Random(12345)

        val realTarget = Vec2Rot(10.0, -20.0, 5.0)

        while (t < 5.0) {
            val dt = rng.nextDouble(0.005, 0.05)
            t += dt

            val virtualLoc = realTarget * tanh(t)

            detailInput.realPos = virtualLoc + Vec2Rot(0.05 * t, -0.09 * t, 0.18 * t)
            stableInput.realPos = virtualLoc + Vec2Rot(rng.nextDouble(-0.5, 0.5), rng.nextDouble(-1.0, 1.0), rng.nextDouble(-0.3, 0.3))

            combined.tick(dt)
        }

        println(combined.pos)

        val error = realTarget - combined.pos
        assert(error.v.x.absoluteValue < 0.1)
        assert(error.v.y.absoluteValue < 0.1)
        assert(error.r.absoluteValue < 0.1)
    }

    @Test
    fun testCompoundOdometryStableDisabled() {
        val detailInput = FakeOdometry()
        val stableInput = FakeOdometry().also { it.hasPosition = false }

        val combined = CompoundOdometry(detailInput, stableInput, 1.0)

        var t = 0.0
        val rng = Random(12345)

        val realTarget = Vec2Rot(10.0, -20.0, 5.0)

        while (t < 5.0) {
            val dt = rng.nextDouble(0.005, 0.05)
            t += dt

            detailInput.realPos = realTarget * tanh(t) + Vec2Rot(sin(t * 10.0), sin(t * 9.0), sin(t * 4.0)) / t

            combined.tick(dt)
        }

        detailInput.realPos = realTarget
        combined.tick(0.1)

        val error = realTarget - combined.pos
        assert(error.v.x.absoluteValue < 1.0e-1)
        assert(error.v.y.absoluteValue < 1.0e-1)
        assert(error.r.absoluteValue < 1.0e-1)

    }

}