package org.firstinspires.ftc.teamcode.ftcGlue.ftcHardware

import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.teamcode.ftcGlue.IDCMotor
import org.firstinspires.ftc.teamcode.ftcGlue.IHardwareMap
import kotlin.reflect.KClass

class FTCHardwareMap(val hardwareMap: HardwareMap) : IHardwareMap {
    override val dcMotors = DCMotorSubMap()

    override fun <T : Any> getRaw(id: String, clazz: KClass<T>) = try {
        hardwareMap.get(clazz.java, id)
    } catch (e: Throwable) {
        null
    }
    inner class DCMotorSubMap : IHardwareMap.SubMap<IDCMotor> {
        override fun get(id: String) = try {
            hardwareMap.dcMotor[id]
        } catch (e: Throwable) {
            null
        }?.let {
            FTCDCMotor(it)
        }
    }
}