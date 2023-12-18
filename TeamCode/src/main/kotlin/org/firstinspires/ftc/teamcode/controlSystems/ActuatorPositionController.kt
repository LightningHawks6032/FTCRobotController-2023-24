package org.firstinspires.ftc.teamcode.controlSystems

import org.firstinspires.ftc.teamcode.controlSystems.motionPath.MotionPath
import org.firstinspires.ftc.teamcode.util.DeltaValue
import kotlin.reflect.KProperty0

class ActuatorPositionController(
        pidCoefficients: PID1D.Coefficients,
        private val setForce: (Double, Double)->Unit,
        posDelegate: KProperty0<Double>,
        motorPosDelegate: KProperty0<Double>,
        val forceScale: Double,
) {
    private val pos by posDelegate
    private val deltaPos by DeltaValue.Double { pos }
    private val motorPos by motorPosDelegate
    private val motorDeltaPos by DeltaValue.Double { motorPos }

    fun posZeroPointEdit(block: () -> Unit) {
        val targetPositionRel = targetPosition - pos
        block()
        deltaPos // access and discard deltaPos to prevent any weird current velocity spikes
        targetPosition = targetPositionRel + pos
    }

//    private val pid = PID1D(pidCoefficients)
    private val pid = PID1D(pidCoefficients)

    var path: MotionPath<Double>? = null
    var targetPosition = pos


    private var t = 0.0
    fun tick(dt: Double) {
        t += dt

        val pos = pos
        val vel = deltaPos / dt
        val motorVel = motorDeltaPos / dt

        val target = path?.sampleClamped(t)
                ?: MotionPath.PathPoint(targetPosition, 0.0, 0.0)

        val force = pid.tick(
                pos, vel,
                target.pos, target.vel,
                dt
        )
        setForce(force * forceScale, motorVel)
//        println("TP: ${target.pos}, $force")
    }
}