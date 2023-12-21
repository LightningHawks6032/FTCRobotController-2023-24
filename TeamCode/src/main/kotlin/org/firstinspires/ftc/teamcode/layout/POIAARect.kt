package org.firstinspires.ftc.teamcode.layout

class POIAARect(
        val x: Pair<Double, Double>,
        val y: Pair<Double, Double>,
) {
    val width get() = x.second - x.first
    val height get() = y.second - y.first

    operator fun times(scale: Double) =
            POIAARect(
                    Pair(x.first * scale, x.second * scale),
                    Pair(y.first * scale, y.second * scale),
            )
}