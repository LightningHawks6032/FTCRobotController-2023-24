package org.firstinspires.ftc.teamcode.controlSystems

import org.firstinspires.ftc.teamcode.controlSystems.motionPath.MotionPath
import org.firstinspires.ftc.teamcode.hardware.Motor
import kotlin.reflect.KProperty0

class ActuatorPositionController(
        pidCoefficients: PID1D.Coefficients,
        private val setForce: (Double, Double)->Unit,
        posDelegate: KProperty0<Double>,
        motorPosDelegate: KProperty0<Double>,
        motorSpec: Motor.PhysicalSpec,
        /** pos = motorPos * posScale + C */
        posScale: Double,
        val forceScale: Double,
) {
    private val posRaw by posDelegate
    private val motorPosRaw by motorPosDelegate
    private val posGen = Smoother(motorSpec.encoderRadPerTick)
    private val motorPosGen = Smoother(posScale * motorSpec.encoderRadPerTick)

    fun posZeroPointEdit(block: () -> Unit) {
        val targetPositionRel = targetPosition - posGen.x
        block()
        posGen.assertPos(posRaw)
        motorPosGen.assertPos(motorPosRaw)
        targetPosition = targetPositionRel + posGen.x
    }

//    private val pid = PID1D(pidCoefficients)
    private val pid = PID1D(pidCoefficients)

    var path: MotionPath<Double>? = null
    var targetPosition = posGen.x


    private var t = 0.0
    fun tick(dt: Double) {
        t += dt

//        println("dt $dt")
        posGen.tick(posRaw, dt)
        motorPosGen.tick(motorPosRaw, dt)
        if (!posGen.x.isFinite()) {
//            println("RESET!")
            posGen.assertPos(posRaw)
            motorPosGen.assertPos(motorPosRaw)
        }
        val pos = posGen.x
        val vel = posGen.v
        val motorVel = motorPosGen.v
//        println("pos $pos, vel $vel")

        val target = path?.sampleClamped(t)
                ?: MotionPath.PathPoint(targetPosition, 0.0, 0.0)

        val force = pid.tick(
                pos, vel,
                target.pos, target.vel,
                dt
        )
        println("target.pos ${target.pos} force $force")
        setForce(force * forceScale, motorVel)
//        println("TP: ${target.pos}, $force")
    }
}