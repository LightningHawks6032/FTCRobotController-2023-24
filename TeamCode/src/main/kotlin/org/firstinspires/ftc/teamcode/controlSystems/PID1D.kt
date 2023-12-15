package org.firstinspires.ftc.teamcode.controlSystems

import kotlin.math.exp
import kotlin.math.tanh

class PID1D(val coefficients: Coefficients) {
    private var xOffAccumulation = 0.0

    fun tick(
            cx: Double, cv: Double,
            tx: Double, tv: Double,
            dt: Double,
    ): Double {
        val (P,I,D,decay) = coefficients

        xOffAccumulation *= exp(-decay * dt)
        xOffAccumulation += (tx - cx) * dt

        return (P*(tx-cx) + I*xOffAccumulation + D*(tv-cv)).let {
            it + tanh(it * coefficients.biasSlope) * coefficients.bias
        }
    }


    data class Coefficients(
            var P: Double, var I: Double, var D: Double,
            val iDecay: Double,
            val biasSlope: Double = 0.0,
            val bias: Double = 0.0,
    )
}