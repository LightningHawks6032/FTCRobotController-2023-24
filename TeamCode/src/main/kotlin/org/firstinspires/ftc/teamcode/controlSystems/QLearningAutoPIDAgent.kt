package org.firstinspires.ftc.teamcode.controlSystems

import kotlin.math.abs
import kotlin.math.pow

/**
 * Q-Value table, state -> action. These values were found
 * with RL, don't expect them to make intuitive sense.
 */
//private val qTable = intArrayOf(1, 0, 4, 2, 1, 3, 2, 2, 4, 1, 0, 4, 3, 0, 0, 3, 1, 3, 3) // old version
//private val qTable = intArrayOf(3, 0, 0, 0, 4, 4, 0, 4, 0, 0, 1, 2, 1, 3, 3, 4, 1, 4, 3)
private val qTable = intArrayOf(3, 4, 4, 4, 4, 2, 4, 0, 0, 3, 4, 3, 4, 2, 4, 4, 3, 3, 0)


private const val THR: Double = 0.05
private fun getState(x: Double, v: Double, tx: Double, tv: Double, force: Double): Int {
    var n = 0

    if (abs(x - tx) > THR || abs(v - tv) > THR) {
        n += 1
        n += if (abs(x - tx) > THR) {
            if (x > tx) {
                0
            } else {
                2
            }
        } else {
            1
        }
        n += if (abs(v - tv) > THR) {
            if (v > tv) {
                0
            } else {
                6
            }
        } else {
            3
        }
        if (force > 0) {
            n += 9
        }
    }
    return n
}

/** PID controller auto-tuned live by a simple AI. */
class QLearningAutoPIDAgent(init_coefficients: PID1D.Coefficients = PID1D.Coefficients(0.5, 0.5, 0.5, 0.5, 0.0, 0.0)) {
    private val pid = PID1D(init_coefficients)

//    private val lastX = mutableListOf<Double>()
//    private val lastV = mutableListOf<Double>()
//    private val T = 10

    fun tick(x: Double, v: Double, tx: Double, tv: Double, dt: Double): Double {
//        while (lastX.size < T) {
//            lastX.add(x_)
//            lastV.add(v_)
//        }
//        val v = lastV.average()
//        val x = lastX.average() + v * dt * T / 2
//        lastV.removeAt(0)
//        lastX.removeAt(0)

        val power = pid.tick(x, v, tx, tv, dt) + 0.5 * (1 / ((x - tx).pow(2) + 1)) * (tv - v)
        val state = getState(x, v, tx, tv, power)
        val scale = 1.0
        when (qTable[state]) {
            0 -> {}
            1 -> pid.coefficients.P += 1.0 * dt * scale // Increase KP
            2 -> pid.coefficients.P -= 1.0 * dt * scale // Decrease KP
            3 -> pid.coefficients.I += .1 * dt * scale // Increase KI
            4 -> pid.coefficients.I -= .1 * dt * scale // Decrease KI
            5 -> pid.coefficients.D += 1.0 * dt * scale // Increase KD
            6 -> pid.coefficients.D -= 1.0 * dt * scale // Decrease KD
        }
//        println("PID: ${pid.coefficients.P} ${pid.coefficients.I} ${pid.coefficients.D}")
//        println("POWER $power")
        return power * 1.0
    }
}