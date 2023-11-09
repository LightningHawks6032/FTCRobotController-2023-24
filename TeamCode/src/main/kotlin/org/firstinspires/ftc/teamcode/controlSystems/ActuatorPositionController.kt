package org.firstinspires.ftc.teamcode.controlSystems

import org.firstinspires.ftc.teamcode.controlSystems.motionPath.MotionPath
import org.firstinspires.ftc.teamcode.util.DeltaValue
import org.firstinspires.ftc.teamcode.util.clamp
import kotlin.reflect.KMutableProperty0
import kotlin.reflect.KProperty0

class ActuatorPositionController(
        pidCoefficients: PID1D.Coefficients,
        powerDelegate: KMutableProperty0<Double>,
        posDelegate: KProperty0<Double>,
) {

    private var power by powerDelegate

    private val pos by posDelegate
    private val deltaPos by DeltaValue.Double { pos }

    fun posZeroPointEdit(block: () -> Unit) {
        val targetPositionRel = targetPosition - pos
        block()
        deltaPos // access and discard deltaPos to prevent any weird current velocity spikes
        targetPosition = targetPositionRel + pos
    }

    private val pid = PID1D(pidCoefficients)

    var path: MotionPath<Double>? = null
    var targetPosition = pos


    private var t = 0.0
    fun tick(dt: Double) {
        t += dt

        val pos = pos
        val vel = deltaPos / dt

        val target = path?.sampleClamped(t)
                ?: MotionPath.PathPoint(targetPosition, 0.0, 0.0)

        power = pid.tick(
                pos, vel,
                target.pos, target.vel,
                dt
        ).clamp(-1.0, 1.0)
    }
}