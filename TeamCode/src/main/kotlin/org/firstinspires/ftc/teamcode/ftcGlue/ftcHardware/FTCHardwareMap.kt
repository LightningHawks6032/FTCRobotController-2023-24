package org.firstinspires.ftc.teamcode.ftcGlue.ftcHardware

import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.teamcode.ftcGlue.IDCMotor
import org.firstinspires.ftc.teamcode.ftcGlue.IHardwareMap

class FTCHardwareMap(val hardwareMap: HardwareMap) : IHardwareMap {
    override val dcMotors = DCMotorSubMap()

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