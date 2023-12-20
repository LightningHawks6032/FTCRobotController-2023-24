package org.firstinspires.ftc.teamcode.util

import kotlin.math.max
import kotlin.math.min

fun Double.clamp(lower: Double, upper: Double) = min(max(this, lower), upper)


class Ops<T>(
        val add: (a: T, b: T) -> T,
        val sub: (a: T, b: T) -> T,
        val neg: (a: T) -> T,
        val scale: (a: T, v: Double) -> T,
        val zero: T,
        val one: T,
)

val Double.Companion.ops
    get() = Ops(
            { a, b -> a + b },
            { a, b -> a - b },
            { a -> -a },
            { a, v -> a * v },
            0.0,
            1.0,
    )

fun DoubleArray.diff() = if (size <= 1)
    doubleArrayOf()
else
    (0 until (size - 1))
            .map { i -> this[i + 1] - this[i] }
            .toList()
            .toDoubleArray()