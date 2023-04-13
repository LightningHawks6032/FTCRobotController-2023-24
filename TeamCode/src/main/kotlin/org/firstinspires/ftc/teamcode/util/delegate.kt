package org.firstinspires.ftc.teamcode.util

import kotlin.reflect.KMutableProperty0
import kotlin.reflect.KProperty
import kotlin.reflect.KProperty0

/** Readonly delegation */
interface RDelegate<T> {
    operator fun getValue(o: Any?, p: KProperty<*>): T
}

/** Mutable delegation */
interface WDelegate<T> : RDelegate<T> {
    operator fun setValue(o: Any?, p: KProperty<*>, v: T)
}

fun <T> KProperty0<T>.delegate() = object : RDelegate<T> {
    val internal by this@delegate
    override fun getValue(o: Any?, p: KProperty<*>) = internal
}

fun <T> KMutableProperty0<T>.delegate() = object : WDelegate<T> {
    var internal by this@delegate
    override fun getValue(o: Any?, p: KProperty<*>) = internal
    override fun setValue(o: Any?, p: KProperty<*>, v: T) {
        println("SET INTERNAL $v")
        internal = v
        println("AFTR SET $internal")
    }
}

fun <T> RDelegate<T>.toDouble() where T : Number = transform { it.toDouble() }
operator fun RDelegate<Double>.times(fac: ()->Double) =
        transform { it * fac() }
operator fun RDelegate<Double>.plus(fac: ()->Double) =
        transform { it + fac() }

operator fun WDelegate<Double>.times(fac: ()->Double) =
        transform(
                toOuter = { it * fac() },
                toInner = { it / fac() }
        )
operator fun WDelegate<Double>.plus(fac: ()->Double) =
        transform(
                toOuter = { it + fac() },
                toInner = { it - fac() }
        )

operator fun RDelegate<Boolean>.not() =
        transform { !it }

operator fun WDelegate<Boolean>.not() =
        transform(
                toOuter = { !it },
                toInner = { !it }
        )
operator fun RDelegate<Double>.unaryMinus() =
        transform { -it }

operator fun WDelegate<Double>.unaryMinus() =
        transform(
                toOuter = { -it },
                toInner = { -it }
        )


fun <T> WDelegate<T>.conditionallyAllowWriting(
        allowedWhen: RDelegate<Boolean>,
        error: Throwable? = null
) = object : WDelegate<T> {
    var internal by this@conditionallyAllowWriting
    val connectedConditionInternal by allowedWhen
    override fun getValue(o: Any?, p: KProperty<*>) = internal
    override fun setValue(o: Any?, p: KProperty<*>, v: T) {
        println("SETTING VALUE: $v, CONNECTED: $connectedConditionInternal")
        if (connectedConditionInternal)
            internal = v
        else if (error != null)
            throw error
    }
}


fun <Inner, Outer> RDelegate<Inner>.transform(
        transfomer: (Inner) -> Outer
) = object : RDelegate<Outer> {
    val internal by this@transform
    override fun getValue(o: Any?, p: KProperty<*>) = transfomer(internal)
}

fun <Inner, Outer> WDelegate<Inner>.transform(
        toOuter: (Inner) -> Outer,
        toInner: (Outer) -> Inner
) = object : WDelegate<Outer> {
    var internal by this@transform
    override fun getValue(o: Any?, p: KProperty<*>) = toOuter(internal)
    override fun setValue(o: Any?, p: KProperty<*>, v: Outer) {
        println("setAtTimes")
        internal = toInner(v)
        println("after")
    }
}