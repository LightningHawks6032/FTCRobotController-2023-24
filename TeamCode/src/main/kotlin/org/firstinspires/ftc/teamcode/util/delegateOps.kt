package org.firstinspires.ftc.teamcode.util

import kotlin.reflect.KProperty
import kotlin.reflect.KProperty0

interface ReadonlyDelegateAdapter<T> {
    operator fun getValue(o: Any?, p: KProperty<*>): T
}

fun KProperty0<Float>.toDouble() = transform { it.toDouble() }

fun <T,R> KProperty0<T>.transform(transfomer: (T)->R) = object : ReadonlyDelegateAdapter<R> {
    val inVal by this@transform
    override fun getValue(o: Any?, p: KProperty<*>) = transfomer(inVal)
}