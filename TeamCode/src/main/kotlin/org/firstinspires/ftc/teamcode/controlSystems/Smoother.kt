package org.firstinspires.ftc.teamcode.controlSystems

import org.firstinspires.ftc.teamcode.util.diff
import kotlin.math.exp
import kotlin.math.max
import kotlin.math.min
import kotlin.math.pow

private const val X_OFF_REDUCER_SCALE = 300.0
private const val X_OFF_REDUCER_SAMPLE_TIME = 0.1
private const val V_JUMP_BLEND_DECAY_SPEED = 10.0

private const val TICK_POINT_MAX_LIFE = 0.5
private const val TICK_POINT_MIN_SPACING = 0.05


class Smoother(private val resolution: Double) {
    private var xinLast = 0.0
    private var recentXIns = mutableListOf<Double>()

    var x = 0.0
        private set
    private var xLast = 0.0
    var v = 0.0
        private set
    private var vLast = 0.0
    var a = 0.0
        private set
    private var aLast = 0.0
    private var t = 0.0

    private val tickTs = mutableListOf(0.0)
    private val tickXs = mutableListOf(0.0)
    private var rising = false
    private var vOff = 0.0
    private var updateOff = false

    fun assertPos(xIn: Double) {
        xinLast = xIn
        recentXIns = mutableListOf(xIn)
        x = xIn
        xLast = xIn
        v = 0.0
        vLast = 0.0
        a = 0.0
        aLast = 0.0
        t = 0.0
        tickTs.clear()
        tickTs.add(0.0)
        tickXs.clear()
        tickXs.add(xIn)
        rising = false
        vOff = 0.0
        updateOff = false
    }

    fun tick(xIn: Double, dt: Double) {
        xLast = x
        vLast = v
        aLast = a
        t += dt
        recentXIns.add(xIn)

        if (recentXIns.size > (X_OFF_REDUCER_SAMPLE_TIME / dt).toInt() && recentXIns.size > 2) {
            recentXIns.removeAt(0)
        }

        if (xIn != xinLast && t - tickTs.last() > TICK_POINT_MIN_SPACING) {
            rising = xIn > xinLast
            tickTs.add(t)
            tickXs.add(if (rising) xIn else xinLast)
            updateOff = true
            if (tickTs.size > 3) {
                tickTs.removeAt(0)
                tickXs.removeAt(0)
            }
            xinLast = xIn
        }

        if (t - tickTs[0] > TICK_POINT_MAX_LIFE && tickTs.size >= 2) {
            updateOff = true
            tickTs.removeAt(0)
            tickXs.removeAt(0)
        }

        if (tickTs.size >= 3) {
            val p = mutableListOf<Double>()
            val diffTs = tickTs.toDoubleArray()
            var diffXs = tickXs.toDoubleArray()
            while (diffXs.size >= 2) {
                diffXs = diffXs.diff()
                diffXs = diffXs.mapIndexed { i, dx -> dx / (diffTs[diffXs.size - 1 - i] - diffTs[i]) }.toDoubleArray()
                p.add(diffXs.last())
            }

            v = 0.0
            val deltaT = t - tickTs.last()
            p.forEachIndexed { pow, k ->
                v += (pow + 1) * k * deltaT.pow(pow)
            }

            val vBoundInner = p.first() * 0.5
            val vBoundOuter = p.first() * 2.0
            v = v.coerceIn(
                    min(-resolution / TICK_POINT_MAX_LIFE, min(vBoundInner, vBoundOuter)),
                    max(resolution / TICK_POINT_MAX_LIFE, max(vBoundInner, vBoundOuter)),
            )
        } else {
            v = 0.0
        }

        if (updateOff) {
            updateOff = false
            vOff = vLast - v
        }
        vOff *= exp(-dt * V_JUMP_BLEND_DECAY_SPEED)
        v += vOff

        v -= (xLast - recentXIns.average()) * X_OFF_REDUCER_SCALE * dt

        x = xLast + v * dt
        a = (v - vLast) / dt
    }
}
