package org.firstinspires.ftc.teamcode.test.ftcGlue

import org.firstinspires.ftc.teamcode.ftcGlue.IDCMotor
import org.firstinspires.ftc.teamcode.hardware.Motor
import org.junit.Test

class CIDCMotor : IDCMotor {
    override var power: Double = 0.0
    override var pos: Int = 0

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
        val internal = hardware.dcMotors[""]!! as CIDCMotor

        internal.power = -69.0
        forward.power = 1.0
        println(internal.power)
        assert(internal.power > 0.0) { "${internal.power}" }
        reversed.power = 1.0
        assert(internal.power < 0.0)

        // position tracking reverses
        internal.pos = 100

        forward.pos = 0.0
        reversed.pos = 0.0
        assert(forward.pos == 0.0)
        assert(reversed.pos == 0.0)

        internal.pos = 101
        assert(forward.pos > 0.0)
        assert(reversed.pos < 0.0)

        forward.pos = 0.0
        reversed.pos = 0.0
        assert(forward.pos == 0.0)
        assert(reversed.pos == 0.0)
    }
}