package org.firstinspires.ftc.teamcode.ftcGlue

interface IHardwareMap {
    val dcMotors: SubMap<IDCMotor>
    interface SubMap<T> {
        operator fun get(id: String): T?
    }
}