package org.firstinspires.ftc.teamcode.hardware

import org.firstinspires.ftc.teamcode.ftcGlue.IDCMotor
import org.firstinspires.ftc.teamcode.ftcGlue.IHardwareMap

class Motor(
        private val id: String
) {

    inner class Impl(hardwareMap: IHardwareMap)
        : IDCMotor by hardwareMap.dcMotors[id] ?: TODO("not found")
    {
    }
}