package org.firstinspires.ftc.teamcode.controlSystems.motionPath

import org.firstinspires.ftc.teamcode.util.Vec2

/** A [MotionPath] composed of several sub-paths in sequence. */
abstract class SequentialMotionPath<T>(
        private vararg val parts: MotionPath<T>
) : MotionPath<T> {
    init {
        if (parts.isEmpty()) {
            throw Error("Empty [CompoundMotionPath]s are not permitted.")
        }
    }

    /** The global time corresponding to the zero time of the currently selected part. */
    private var zeroT = 0.0

    /** The index of the currently selected part. */
    private var selectedPartI = 0


    private fun incrementPartI(): Boolean {
        if (selectedPartI >= parts.lastIndex) return false
        zeroT += parts[selectedPartI].validUntilT
        selectedPartI++
        return true
    }

    private fun decrementPartI(): Boolean {
        if (selectedPartI <= 0) return false
        selectedPartI--
        zeroT -= parts[selectedPartI].validUntilT
        return true
    }

    /**
     * Select the path corresponding to the current time.
     *
     * **PRECONDITION: `splineParts.isNotEmpty()`**
     */
    private fun selectPath(t: Double): MotionPath<T> {
        while ((t - zeroT) < 0)
            if (decrementPartI()) continue else break
        while ((t - zeroT) > parts[selectedPartI].validUntilT)
            if (incrementPartI()) continue else break
        return parts[selectedPartI]
    }

    override val validUntilT = parts.map { it.validUntilT }.reduce { a, b -> a + b }
    override fun sample(t: Double): MotionPath.PathPoint<T> {
        return if (t >= validUntilT) {
            // `t` is after the end of this path.
            parts.last().let { it.sample(it.validUntilT).pos }.let {
                MotionPath.PathPoint(it, zero, zero)
            }
        } else if (t < 0) {
            // `t` is before the start of this path.
            parts.first().sample(0.0).pos.let {
                MotionPath.PathPoint(it, zero, zero)
            }
        } else {
            // `t` is in this path, sample a sub-path.
            selectPath(t).sample(t - zeroT)
        }
    }

    /** A [MotionPath]<[Double]> composed of several sub-paths. */
    class TDouble(vararg paths: MotionPath<Double>) : SequentialMotionPath<Double>(*paths) {
        override val zero = 0.0
    }

    /** A [MotionPath]<[Vec2]> composed of several sub-paths. */
    class TVec2(vararg paths: MotionPath<Vec2>) : SequentialMotionPath<Vec2>(*paths) {
        override val zero = Vec2.zero
    }
}