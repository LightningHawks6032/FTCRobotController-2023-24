package org.firstinspires.ftc.teamcode.controlSystems.motionPath

import org.firstinspires.ftc.teamcode.util.Vec2

/** A [MotionPath] that holds a constant position over time. */
abstract class StationaryMotionPath<T>(
        val pos: T,
        len: Double,
) : MotionPath<T> {

    override val validUntilT = len
    override fun sample(t: Double) = MotionPath.PathPoint(
            pos,
            zero,
            zero,
    )


    /** A [MotionPath]<[Double]> that holds a constant position over time. */
    class TDouble(
            pos: Double,
            len: Double,
    ) : StationaryMotionPath<Double>(pos, len) {
        override val zero = 0.0
    }

    /** A [MotionPath]<[Vec2]> that holds a constant position over time. */
    class TVec2(
            pos: Vec2,
            len: Double,
    ) : StationaryMotionPath<Vec2>(pos, len) {
        override val zero = Vec2.zero
    }
}
