package org.firstinspires.ftc.teamcode.controlSystems

import org.firstinspires.ftc.teamcode.controlSystems.motionPath.MotionPath
import org.firstinspires.ftc.teamcode.hardware.motion.IDrive
import org.firstinspires.ftc.teamcode.hardware.motion.IOdometry
import org.firstinspires.ftc.teamcode.util.NotForCompetition
import org.firstinspires.ftc.teamcode.util.Vec2Rot
import kotlin.math.abs
import kotlin.math.max
import kotlin.math.pow

const val MAX_POW = 0.70710678118 // sqrt(1/2)

class DriveController(
        private val output: IDrive,
        private val input: IOdometry,
) {
    @NotForCompetition
    fun debugTakeControl(): Pair<IOdometry, IDrive> {
        disableTicking = true
        return Pair(input, output)
    }
    private var disableTicking = false

    var path: MotionPath<Vec2Rot>? = null
    var targetPos: Vec2Rot = input.pos

    private val pid = PID(PIDCoefficients(
            0.50,
            1.50,
            1.00,
            0.5,
    ))
    var pidCoefficients by pid::coefficients

    var t = 0.0
    fun tick(dt: Double) {
        if (disableTicking) return // disables ticking when [debugTakeControlOfOutput] is called.

        t += dt
        input.tick(dt)

        val target = path?.sampleClamped(t)
                ?: MotionPath.PathPoint(targetPos, Vec2Rot.zero, Vec2Rot.zero)
        targetPos = target.pos // save so robot stays in place when [path] is deleted

        val pow = pid.tick(
                input.pos, input.vel,
                target.pos, target.vel,
                dt
        )
        output.power = input.robot2worldTransform().transformVelInv(
                pow * (MAX_POW / max(MAX_POW, max(pow.v.mag, abs(pow.r))))
        )
    }

    fun shutdownOutput() {
        output.power = Vec2Rot.zero
    }

    class PID(
            var coefficients: PIDCoefficients
    ) {
        private var iValue = Vec2Rot.zero
        fun tick(
                xp: Vec2Rot, xd: Vec2Rot,
                tp: Vec2Rot, td: Vec2Rot,
                dt: Double,
        ): Vec2Rot {
            val pValue = tp - xp
            val dValue = td - xd
            iValue += pValue * dt
            iValue *= coefficients.iValueDecay.pow(dt)

            println("p : $pValue  d : $dValue  i : $iValue")

            return (pValue * coefficients.p + iValue * coefficients.i + dValue * coefficients.d)
        }
    }
    data class PIDCoefficients(
            val i: Double,
            val p: Double,
            val d: Double,
            val iValueDecay: Double,
    )
}