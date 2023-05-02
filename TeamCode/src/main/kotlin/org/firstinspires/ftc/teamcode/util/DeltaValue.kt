package org.firstinspires.ftc.teamcode.util

import kotlin.reflect.KMutableProperty0
import kotlin.reflect.KProperty

abstract class DeltaValue<T>(protected val source: () -> T) {
    protected var last = source()

    abstract fun read(): T

    operator fun getValue(o: Any?, p: KProperty<*>) = read()

    class Double(source: () -> kotlin.Double) : DeltaValue<kotlin.Double>(source) {
        constructor(source: KMutableProperty0<kotlin.Double>) : this({ source.get() })

        override fun read(): kotlin.Double {
            val current = source()
            val delta = current - last
            last = current
            return delta
        }
    }
}