package org.firstinspires.ftc.teamcode.util

import kotlin.reflect.KMutableProperty0
import kotlin.reflect.KProperty
import kotlin.reflect.KProperty0

interface ReadonlyDelegateAdapter<T> {
    operator fun getValue(o: Any?, p: KProperty<*>): T
}
interface MutableDelegateAdapter<T> {
    operator fun getValue(o: Any?, p: KProperty<*>): T
    operator fun setValue(o: Any?, p: KProperty<*>, v: T)
}

fun KProperty0<Number>.toDouble() = transform { it.toDouble() }
operator fun KProperty0<Double>.times(fac: KProperty0<Double>) =
        transform { it * fac.get() }
operator fun KMutableProperty0<Double>.times(fac: KProperty0<Double>) =
        transform(
                toOuter = { it * fac.get() },
                toInner = { it / fac.get() }
        )

fun <Inner,Outer> KProperty0<Inner>.transform(
        transfomer: (Inner)->Outer
) = object : ReadonlyDelegateAdapter<Outer> {
    val inVal by this@transform
    override fun getValue(o: Any?, p: KProperty<*>) = transfomer(inVal)
}
fun <Inner,Outer> KMutableProperty0<Inner>.transform(
        toOuter: (Inner)->Outer,
        toInner: (Outer)->Inner
) = object : MutableDelegateAdapter<Outer> {
    var inVal by this@transform
    override fun getValue(o: Any?, p: KProperty<*>) = toOuter(inVal)
    override fun setValue(o: Any?, p: KProperty<*>, v: Outer) {
        inVal = toInner(v)
    }
}