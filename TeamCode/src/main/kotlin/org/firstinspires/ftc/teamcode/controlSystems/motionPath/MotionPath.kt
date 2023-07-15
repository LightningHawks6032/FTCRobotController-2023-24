package org.firstinspires.ftc.teamcode.controlSystems.motionPath

import org.firstinspires.ftc.teamcode.util.Vec2
import org.firstinspires.ftc.teamcode.util.Vec2Rot

/**
 * Abstract class representing paths that various control systems may need.
 *
 * The path, defined by [sample] must be a valid value at all
 * times 0 <= `t` <= [validUntilT]]
 */
interface MotionPath<T> {
    /** Sample this path at a time [t]. */
    fun sample(t: Double): PathPoint<T>

    /**
     * Sample this path at a time [t]. And if [t]
     * is out of bounds, take target position from
     * the nearest end point (at [t] = `0` or
     * [validUntilT]).
     */
    fun sampleClamped(t: Double) = if (t < 0) {
        sample(0.0).zeroAllExceptPos()
    } else if (t > validUntilT) {
        sample(validUntilT).zeroAllExceptPos()
    } else {
        sample(t)
    }

    /**
     * A duration of time (in seconds) for which [sample] returns valid data.
     *
     * [sample] must be a valid value at all times `t` in `[0 - validUntilT]`
     *
     * **NOTE: Should not change using getters or delegates; should just be a constant!**
     */
    val validUntilT: Double

    /** The "zero" value for type [T]. (ex. `0.0` for double, and [Vec2.zero] for [Vec2])*/
    val zero: T

    /**
     * A sample of a [MotionPath].
     *
     * Contains position, velocity, and acceleration.
     * (eg. value, 1st derivative, and 2nd derivative)
     */
    data class PathPoint<T>(
            val pos: T,
            val vel: T,
            val acc: T,
    )

    private fun PathPoint<T>.zeroAllExceptPos() = PathPoint(
            pos,
            zero,
            zero,
    )
}

/**
 * Small value used for numeric derivatives in [pathPointV2R],
 * [pathPointV2], and [pathPointD]
 */
private const val DT: Double = 0.01

/**
 * Sample a [MotionPath.PathPoint]<[Vec2Rot]> at time [t] from a function, numerically calculating
 * the first and second derivatives for the velocity and acceleration.
 *
 * @param t Time to sample at (passed to [fn]).
 * @param fn A function receiving time and returning the position value.
 */
fun pathPointV2R(t: Double, fn: (Double) -> Vec2Rot) =
        MotionPath.PathPoint(
                fn(t),
                (fn(t + DT / 2) - fn(t - DT / 2)) / DT,
                ((fn(t + DT) - fn(t)) / DT - (fn(t) - fn(t - DT)) / DT) / DT,
        )

/**
 * Sample a [MotionPath.PathPoint]<[Vec2]> at time [t] from a function, numerically calculating
 * the first and second derivatives for the velocity and acceleration.
 *
 * @param t Time to sample at (passed to [fn]).
 * @param fn A function receiving time and returning the position value.
 */
fun pathPointV2(t: Double, fn: (Double) -> Vec2) =
        MotionPath.PathPoint(
                fn(t),
                (fn(t + DT / 2) - fn(t - DT / 2)) / DT,
                ((fn(t + DT) - fn(t)) / DT - (fn(t) - fn(t - DT)) / DT) / DT,
        )

/**
 * Sample a [MotionPath.PathPoint]<[Double]> at time [t] from a function, numerically calculating
 * the first and second derivatives for the velocity and acceleration.
 *
 * @param t Time to sample at (passed to [fn]).
 * @param fn A function receiving time and returning the position value.
 */
fun pathPointD(t: Double, fn: (Double) -> Double) =
        MotionPath.PathPoint(
                fn(t),
                (fn(t + DT / 2) - fn(t - DT / 2)) / DT,
                ((fn(t + DT) - fn(t)) / DT - (fn(t) - fn(t - DT)) / DT) / DT,
        )
