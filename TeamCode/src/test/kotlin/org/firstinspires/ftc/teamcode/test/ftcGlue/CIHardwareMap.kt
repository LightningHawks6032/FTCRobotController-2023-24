package org.firstinspires.ftc.teamcode.test.ftcGlue

import org.firstinspires.ftc.teamcode.ftcGlue.IDCMotor
import org.firstinspires.ftc.teamcode.ftcGlue.IHardwareMap
import org.firstinspires.ftc.teamcode.ftcGlue.IServo
import org.junit.Test
import kotlin.reflect.KClass

class CIHardwareMap : IHardwareMap {
    override val dcMotors: IHardwareMap.SubMap<IDCMotor> = SubMap { CIDCMotor() }
    override val servos: IHardwareMap.SubMap<IServo> = SubMap { CIServo() }

    val requested = mutableMapOf<String,Any?>()

    // Can't do `getRaw` here, it doesn't make sense because it requires the IRL robot context
    override fun <T : Any> getRaw(id: String, clazz: KClass<T>) = null

    inner class SubMap<T : Any>(val get: ()->T?) : IHardwareMap.SubMap<T> {
        private val requestedInSubMap = mutableMapOf<String,T>()
        override fun get(id: String) = requestedInSubMap[id] ?: get()?.also {
            requested[id] = it
            requestedInSubMap[id] = it
        }
    }

    @Test
    fun subMapReturnsSameInstanceWhenSameId() {
        val idA = "a"
        val idB = "b"
        val subMap = SubMap { Any() }
        val a1 = subMap[idA]
        val a2 = subMap[idA]
        val b = subMap[idB]

        assert(a1 != b)
        assert(a1 == a2)
    }
}