package org.firstinspires.ftc.teamcode.ftcGlue

import kotlin.reflect.KClass

interface IHardwareMap {
    val dcMotors: SubMap<IDCMotor>
    val servos: SubMap<IServo>
    fun <T : Any> getRaw(id: String, clazz: KClass<T>): T?
    interface SubMap<T> {
        operator fun get(id: String): T?
    }
}