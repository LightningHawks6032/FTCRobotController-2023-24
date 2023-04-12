package org.firstinspires.ftc.teamcode.test.ftcGlue

import org.firstinspires.ftc.teamcode.ftcGlue.IDCMotor
import org.firstinspires.ftc.teamcode.hardware.Motor
import org.junit.Test

class CIDCMotor : IDCMotor {
    override var power: Double = 0.0
    override val pos: Int = 0

    // TODO: simulation?


    @Test
    fun reverseWorks() {
        val hardware = CIHardwareMap()
        val forward = Motor(
                "",
                Motor.PhysicalSpec.GOBILDA_5202_0002_0005,
                Motor.Config {
                    assert(!reversed) // should default to not reversed
                }
        ).Impl(hardware)
        val reversed = Motor(
                "",
                Motor.PhysicalSpec.GOBILDA_5202_0002_0005,
                Motor.Config {
                    reversed = true
                }
        ).Impl(hardware)
        val internal = hardware.dcMotors[""]!!

        forward.power = 1.0
        assert(internal.power > 0.0)
        reversed.power = 1.0
        assert(internal.power < 0.0)
    }
}