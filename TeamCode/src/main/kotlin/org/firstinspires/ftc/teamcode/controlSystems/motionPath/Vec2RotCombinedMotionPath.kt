package org.firstinspires.ftc.teamcode.controlSystems.motionPath

import org.firstinspires.ftc.teamcode.util.Vec2
import org.firstinspires.ftc.teamcode.util.Vec2Rot
import kotlin.math.max

class Vec2RotCombinedMotionPath(
        private val pathXY: MotionPath<Vec2>,
        private val pathR: MotionPath<Double>,
) : MotionPath<Vec2Rot> {
    override val validUntilT = max(pathXY.validUntilT, pathR.validUntilT)
    override val zero = Vec2Rot.zero
    override fun sample(t: Double): MotionPath.PathPoint<Vec2Rot> {
        val xy = pathXY.sampleClamped(t)
        val r = pathR.sampleClamped(t)
        return MotionPath.PathPoint(
                Vec2Rot(xy.pos, r.pos),
                Vec2Rot(xy.vel, r.vel),
                Vec2Rot(xy.acc, r.acc),
        )
    }
}