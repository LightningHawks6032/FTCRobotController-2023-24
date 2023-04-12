package org.firstinspires.ftc.teamcode.test

import org.firstinspires.ftc.teamcode.ftcGlue.IDCMotor
import org.firstinspires.ftc.teamcode.ftcGlue.IHardwareMap
import org.firstinspires.ftc.teamcode.hardware.Motor
import org.firstinspires.ftc.teamcode.test.ftcGlue.CIHardwareMap
import org.junit.Test

class AssemblyTests {

    @Test
    fun implTest() {
        val hardware = CIHardwareMap()
        val asm = Assembly()
        assert(hardware.requested.isEmpty())
        val asmI = asm.Impl(hardware)
        assert(hardware.requested[ID_A] is IDCMotor)
        assert(hardware.requested[ID_B] is IDCMotor)
        println(hardware.requested)
        assert(hardware.requested.size == 2)

        val power = 0.8
        asmI.setPowers(power)
        assert(asmI.motorA.power == power)
        assert(asmI.motorB.power == power)
    }

    inner class Assembly {
        private val motorPhysicalSpec = Motor.PhysicalSpec.GOBILDA_5202_0002_0005
        val motorA = Motor(ID_A,motorPhysicalSpec)
        val motorB = Motor(ID_B,motorPhysicalSpec)
        inner class Impl(hardwareMap: IHardwareMap) {
            val motorA = this@Assembly.motorA.Impl(hardwareMap)
            val motorB = this@Assembly.motorB.Impl(hardwareMap)

            fun setPowers(power: Double) {
                motorA.power = power
                motorB.power = power
            }
        }
    }

    companion object {
        private const val ID_A = "a"
        private const val ID_B = "b"
    }
}