package org.firstinspires.ftc.teamcode.util

import kotlin.reflect.KMutableProperty0

abstract class Integrator<T>(initial: T, protected val source: () -> T) {
    var current = initial
        protected set

    abstract fun read(dt: kotlin.Double): T


    class Double(initial: kotlin.Double, source: () -> kotlin.Double) : Integrator<kotlin.Double>(initial, source) {
        constructor(initial: kotlin.Double, source: KMutableProperty0<kotlin.Double>) : this(initial, { source.get() })

        override fun read(dt: kotlin.Double): kotlin.Double {
            current += source() * dt
            return current
        }
    }

    class IVec2Rot(initial: Vec2Rot, source: () -> Vec2Rot) : Integrator<Vec2Rot>(initial, source) {
        constructor(initial: Vec2Rot, source: KMutableProperty0<Vec2Rot>) : this(initial, { source.get() })

        override fun read(dt: kotlin.Double): Vec2Rot {
            current += source() * dt
            return current
        }
    }
}