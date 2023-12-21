package org.firstinspires.ftc.teamcode.layout

import org.firstinspires.ftc.teamcode.util.Vec2

class POILine(
        val start: Vec2,
        val end: Vec2,
) {
    val length get() = (start-end).mag

    operator fun times(scale: Double) =
            POILine(
                    start * scale,
                    end * scale,
            )

    companion object {
        /** Creating a line spanning the X direction */
        fun xLine(x: Pair<Double, Double>, y: Double) = POILine(
                Vec2(x.first, y),
                Vec2(x.second, y),
        )
        /** Creating a line spanning the Y direction */
        fun yLine(x: Double, y: Pair<Double, Double>) = POILine(
                Vec2(x, y.first),
                Vec2(x, y.second),
        )
    }

    /** projects another vector onto the span of this line */
    fun project(other: Vec2): Vec2 {
        val span = (end - start).norm
        return start + span * ((other - start) dot span)
    }
}