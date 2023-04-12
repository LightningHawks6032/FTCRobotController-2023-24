package org.firstinspires.ftc.teamcode.hardware

import org.firstinspires.ftc.teamcode.ftcGlue.IHardwareMap
import org.firstinspires.ftc.teamcode.util.transform

class Motor(
        private val id: String,
        val physicalSpec: PhysicalSpec,
        config: Config = Config.default
) {
    var reversed = config.reversed

    class Config {
        var reversed = false

        companion object {
            val default get() = Config()
            operator fun invoke(block: Config.() -> Unit) =
                    Config().also(block)
        }
    }

    enum class PhysicalSpec(
            /**
             * Motor encoder resolution in `ticks / rev`.
             * - "PPR at the Output Shaft" on gobilda spec sheet
             */
            encoderResolutionPPR: Double,
    ) {
        GOBILDA_5202_0002_0005(145.1);

        /** Motor encoder resolution in `rad / tick`.*/
        val encoderRadPerTick = (2 * Math.PI) / encoderResolutionPPR // (2pi rad / 1 rev) * (rev / tick)
    }

    inner class Impl(hardwareMap: IHardwareMap) {
        private val idcMotor = hardwareMap.dcMotors[id] ?: TODO("not found")

        private val reverseConst get() = if (reversed) -1.0 else 1.0
        var power by idcMotor::power.transform(
                toOuter = { reverseConst * it },
                toInner = { reverseConst * it }
        )
        val pos get() = reverseConst * idcMotor.pos.toDouble() * physicalSpec.encoderRadPerTick
    }
}