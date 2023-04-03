package org.firstinspires.ftc.teamcode.test.ftcGlue

import org.firstinspires.ftc.teamcode.ftcGlue.IDCMotor
import org.firstinspires.ftc.teamcode.ftcGlue.IHardwareMap

class CIHardwareMap : IHardwareMap {
    override val dcMotors: IHardwareMap.SubMap<IDCMotor> = SubMap { CIDCMotor() }

    val requested = mutableMapOf<String,Any?>()
    inner class SubMap<T : Any>(val get: ()->T?) : IHardwareMap.SubMap<T> {
        override fun get(id: String) = get()?.also {
            requested[id] = it
        }
    }
}