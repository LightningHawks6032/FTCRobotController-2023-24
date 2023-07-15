package org.firstinspires.ftc.teamcode.controlSystems.motionPath

import org.firstinspires.ftc.teamcode.util.CubicBezier
import org.firstinspires.ftc.teamcode.util.Vec2
import org.firstinspires.ftc.teamcode.util.Vec2Rot

class MotionPathBuilder(startPos: Vec2Rot, startVel: Vec2Rot) {
    private val pathsXY = mutableListOf<MotionPath<Vec2>>()
    private var tEndXY = 0.0
    private var lastPosXY = startPos.v
    private var lastVelXY = startVel.v
    private val pathsR = mutableListOf<MotionPath<Double>>()
    private var tEndR = 0.0
    private var lastPosR = startPos.r
    private var lastVelR = startVel.r

    inner class Scope {
        fun addXY(path: MotionPath<Vec2>) {
            pathsXY.add(path)
            tEndXY += path.validUntilT
            val end = path.sample(path.validUntilT)
            lastPosXY = end.pos
            lastVelXY = end.vel
        }

        fun addR(path: MotionPath<Double>) {
            pathsR.add(path)
            tEndR += path.validUntilT
            val end = path.sample(path.validUntilT)
            lastPosR = end.pos
            lastVelR = end.vel
        }

        fun stationaryUntilAt(untilT: TimeWithUnits) {
            val t = untilT.seconds
            if (t > tEndXY)
                addXY(StationaryMotionPath.TVec2(lastPosXY, t - tEndXY))
            if (t > tEndR)
                addR(StationaryMotionPath.TDouble(lastPosR, t - tEndR))
        }
        fun stationaryXY(time: EventTime) {
            val duration = time.durationSecondsFromNow(tEndXY).ensureIsNotNegativeDuration()
            addXY(StationaryMotionPath.TVec2(lastPosXY, duration))
        }
        fun stationaryR(time: EventTime) {
            val duration = time.durationSecondsFromNow(tEndR).ensureIsNotNegativeDuration()
            addR(StationaryMotionPath.TDouble(lastPosR, duration))
        }
        fun bezierXY(end: EventTime, toPos: Vec2, toVel: Vec2 = Vec2.zero) {
            val duration = end.durationSecondsFromNow(tEndXY).ensureIsNotNegativeDuration()
            addXY(BezierMotionPath.TVec2(CubicBezier.forEndpointTangents(
                    duration,
                    lastPosXY, lastVelXY,
                    toPos, toVel,
            )))
        }
        fun bezierR(end: EventTime, toPos: Double, toVel: Double = 0.0) {
            val duration = end.durationSecondsFromNow(tEndXY).ensureIsNotNegativeDuration()
            addR(BezierMotionPath.TDouble(CubicBezier.forEndpointTangents(
                    duration,
                    lastPosR, lastVelR,
                    toPos, toVel,
            )))
        }

        val Double.seconds get() = TimeWithUnits.Seconds(this)
        val Double.milliseconds get() = TimeWithUnits.Milliseconds(this)

        private fun Double.ensureIsNotNegativeDuration() = this.also {
            if (this <= 0) throw Error("Illegal non-positive MotionPath duration.")
        }

        inner class Lasting(private val timeLen: TimeWithUnits) : EventTime {
            override fun durationSecondsFromNow(nowSeconds: Double) =
                    timeLen.seconds
        }
        inner class UntilAt(private val timeAt: TimeWithUnits) : EventTime {
            override fun durationSecondsFromNow(nowSeconds: Double) =
                    timeAt.seconds - nowSeconds
        }
    }
    interface EventTime {
        fun durationSecondsFromNow(nowSeconds: Double): Double
    }
    interface TimeWithUnits {
        val seconds: Double
        class Seconds(override val seconds: Double): TimeWithUnits
        class Milliseconds(millis: Double): TimeWithUnits {
            override val seconds = millis * 0.001
        }
    }

    fun build(): MotionPath<Vec2Rot> {
        return Vec2RotCombinedMotionPath(
                SequentialMotionPath.TVec2(*pathsXY.toTypedArray()),
                SequentialMotionPath.TDouble(*pathsR.toTypedArray()),
        )
    }
}

fun buildPath(startPos: Vec2Rot, startVel: Vec2Rot, cb: MotionPathBuilder.Scope.() -> Unit) =
        MotionPathBuilder(startPos, startVel).also {
            cb(it.Scope())
        }.build()