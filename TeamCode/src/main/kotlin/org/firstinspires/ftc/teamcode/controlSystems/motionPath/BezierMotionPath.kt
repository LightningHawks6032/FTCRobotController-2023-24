package org.firstinspires.ftc.teamcode.controlSystems.motionPath

import org.firstinspires.ftc.teamcode.util.CubicBezier
import org.firstinspires.ftc.teamcode.util.Vec2

abstract class BezierMotionPath<T>(
        private val bezier: CubicBezier<T>,
) : MotionPath<T> {
    override val validUntilT by bezier::domainLen
    override fun sample(t: Double) = MotionPath.PathPoint(
            bezier.b0(t),
            bezier.b1(t),
            bezier.b2(t),
    )

    class TDouble(
            bezier: CubicBezier<Double>,
    ) : BezierMotionPath<Double>(bezier) {
        override val zero = 0.0
    }

    class TVec2(
            bezier: CubicBezier<Vec2>,
    ) : BezierMotionPath<Vec2>(bezier) {
        override val zero = Vec2.zero
    }
}
