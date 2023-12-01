package org.firstinspires.ftc.teamcode.hardware.motion

import org.firstinspires.ftc.teamcode.util.Vec2Rot

/**
 * A virtual odometry object that combines the inputs of two
 * other odometry systems, taking the absolute position from
 * one (`loPassOdo`), and "fine-grain" position from the other
 * one (`hiPassOdo`).
 *
 * see the following for more info:
 * - https://en.wikipedia.org/wiki/Low-pass_filter#Discrete-time_realization
 * - https://en.wikipedia.org/wiki/High-pass_filter#Discrete-time_realization
 */
class CompoundOdometry(
        private val hiPassOdo: IOdometry,
        private val loPassOdo: IOdometry,
        /** blending filter cutoff frequency (hertz) */
        private val cutoffFreq: Double,
) : IOdometry() {

    private var yHi = hiPassOdo.pos
    private var xHiLast = hiPassOdo.pos
    private var yLo = loPassOdo.pos

    override fun tick(dt: Double) {
        val alpha = (Math.PI * 2 * cutoffFreq * dt).let { it / (it + 1.0) }
        val lastPos = pos

        hiPassOdo.tick(dt)
        loPassOdo.tick(dt)

        val xHi = hiPassOdo.pos
        val xLo = if (loPassOdo.hasPositionInfo()) {
            loPassOdo.pos
        } else {
            // If the low-passed source is unavailable, fallback to
            // just using the high-pass source.
            xHi
        }

        // Low-pass to capture general location from one source.
        yLo = (xLo * alpha) + (yLo * (1 - alpha))

        // High-pass to capture high-detail location information from another source
        yHi = (yHi + (xHi - xHiLast)) * alpha
        xHiLast = xHi

        // Pos is the sum, (*2 to account for dampening)
        // I didn't do the math for this but it works in the simulations.
        // ^^^^ this is probably wrong

        val lastVel = vel
        pos = yHi + yLo
        vel = (pos - lastPos) / dt
        acc = (vel - lastVel) / dt

        // Update the hiPassOdo's general position so it doesn't get to reporting positions based
        // on unreliable assumptions.
        if (loPassOdo.hasPositionInfo()) {
            hiPassOdo.nudge(pos)
        }
    }

    override fun assertPosition(newPos: Vec2Rot) {
        pos = newPos
        vel = Vec2Rot.zero
        acc = Vec2Rot.zero
        yHi = Vec2Rot.zero
        yLo = newPos
        hiPassOdo.assertPosition(newPos)
        loPassOdo.assertPosition(newPos)
    }

    override fun nudge(newPos: Vec2Rot) {
        pos = newPos
    }
}