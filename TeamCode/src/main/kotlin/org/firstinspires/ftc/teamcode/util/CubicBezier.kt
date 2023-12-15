package org.firstinspires.ftc.teamcode.util

/**
 * A cubic Bézier curve, defined by 4 points.
 *
 * [wikipedia](https://en.wikipedia.org/wiki/B%C3%A9zier_curve)
 */
abstract class CubicBezier<T>(
        /**
         * The length of the domain for the input parameter, such that
         * this bezier curve will be valid for all `t` satisfying
         * 0 <= `t` <= [domainLen]
         */
        val domainLen: Double,
        private val p0: T,
        private val p1: T,
        private val p2: T,
        private val p3: T,
) {
    /** Value of bezier curve at `t` (0 is for 0th derivative (no derivative))*/
    fun b0(t: Double) = Pair(1 - t / domainLen, t / domainLen).let { (u, v) ->
        (1 * u * u * u) * p0 + (3 * u * u * v) * p1 + (3 * u * v * v) * p2 + (1 * v * v * v) * p3
    }

    /** Value of 1st derivative bezier curve at `t` */
    fun b1(t: Double) = (1.0 / domainLen) * Pair(1 - t / domainLen, t / domainLen).let { (u, v) ->
        (3 * u * u) * (p1 - p0) + (6 * u * v) * (p2 - p1) + (3 * v * v) * (p3 - p2)
    }

    /** Value of 2nd derivative bezier curve at `t` */
    fun b2(t: Double) = (1.0 / domainLen / domainLen) * Pair(1 - t / domainLen, t / domainLen).let { (u, v) ->
        (6 * u) * (p2 - 2.0 * p1 + p0) + (6 * v) * (p3 - 2.0 * p2 + p1)
    }

    // Operator overloads for convenience
    protected operator fun T.plus(other: T) = tPlus(this, other)
    protected operator fun T.minus(other: T) = tPlus(this, -1.0 * other)
    protected operator fun Double.times(other: T) = tTimes(other, this)

    /** Provide an operator overload for `T + T`. */
    protected abstract fun tPlus(a: T, b: T): T

    /** Provide an operator overload for `T * Double`. */
    protected abstract fun tTimes(a: T, scalar: Double): T

    /** Cubic bezier implementation for [Double]s */
    class TDouble(
            len: Double, p0: Double, p1: Double, p2: Double, p3: Double
    ) : CubicBezier<Double>(len, p0, p1, p2, p3) {
        override fun tPlus(a: Double, b: Double) = a + b
        override fun tTimes(a: Double, scalar: Double) = a * scalar
    }

    /** Cubic bezier implementation for [Vec2]s */
    class TVec2(
            len: Double, p0: Vec2, p1: Vec2, p2: Vec2, p3: Vec2,
    ) : CubicBezier<Vec2>(len, p0, p1, p2, p3) {
        override fun tPlus(a: Vec2, b: Vec2) = a + b
        override fun tTimes(a: Vec2, scalar: Double) = a * scalar
    }

    companion object {
        /**
         * Define a [CubicBezier] which starts at [pa] with velocity
         * [va] and ends at [pb] with velocity [vb] over a time of
         * [domainLen] time units (usually seconds).
         */
        fun forEndpointTangents(domainLen: Double, pa: Vec2, va: Vec2, pb: Vec2, vb: Vec2): CubicBezier<Vec2> =
                TVec2(
                        domainLen,
                        pa,
                        pa + (va / 3.0) * domainLen,
                        pb - (vb / 3.0) * domainLen,
                        pb,
                )

        /**
         * Define a [CubicBezier] which starts at [pa] with velocity
         * [va] and ends at [pb] with velocity [vb] over a time of
         * [domainLen] time units (usually seconds).
         */
        fun forEndpointTangents(domainLen: Double, pa: Double, va: Double, pb: Double, vb: Double): CubicBezier<Double> =
                TDouble(
                        domainLen,
                        pa,
                        pa + (va / 3.0) * domainLen,
                        pa - (vb / 3.0) * domainLen,
                        pb,
                )
    }
}
