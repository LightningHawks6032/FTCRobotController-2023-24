package org.firstinspires.ftc.teamcode.hardware

import com.qualcomm.robotcore.util.RobotLog
import org.firstinspires.ftc.teamcode.ftcGlue.IHardwareMap
import org.firstinspires.ftc.teamcode.util.conditionallyAllowWriting
import org.firstinspires.ftc.teamcode.util.delegate
import org.firstinspires.ftc.teamcode.util.times

class Motor(
        private val id: String,
        val motorSpec: PhysicalSpec,
        config: Config = Config.default
) {
    var reversed = config.reversed

    /** Whether or not this motor uses it's encoder / pos */
    val useEncoder = config.useEncoder

    /** Whether or not this motor uses it's power */
    val useMotor = !motorSpec.encoderOnly

    private var zeroPos = 0.0

    class Config {
        var reversed = false
        var useEncoder = true

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

            /** Marks this spec as only containing an encoder. */
            val encoderOnly: Boolean = false,
    ) {
        GOBILDA_5202_0002_0005(145.1),
        GOBILDA_5202_0002_0003(103.8),
        GOBILDA_ODOMETRY_POD(2000.0, encoderOnly = true),
        REV_THROUGH_BORE_ENCODER(8192.0, encoderOnly = true);

        /** Motor encoder resolution in `rad / tick`.*/
        val encoderRadPerTick = (2 * Math.PI) / encoderResolutionPPR // (2pi rad / 1 rev) * (rev / tick)
    }

    inner class Impl(hardwareMap: IHardwareMap) {
        private val idcMotor = hardwareMap.dcMotors[id] ?: TODO("not found")

        private val reverseConst get() = if (reversed) -1.0 else 1.0
        var power by (
                idcMotor::power.delegate()
                        .conditionallyAllowWriting(
                                allowedWhen = this@Motor::useMotor.delegate()
                        ).times { reverseConst }
                )

        private val scaledPosRaw get() = idcMotor.pos * motorSpec.encoderRadPerTick
        var pos
            set(value) {
                if (useEncoder)
                    zeroPos = scaledPosRaw - value * reverseConst
                else
                    RobotLog.addGlobalWarningMessage(
                            "Attempt to assert motor position with encoder disabled. [id: '$id']"
                    )
            }
            get() = if (useEncoder)
                (scaledPosRaw - zeroPos) * reverseConst
            else
                0.0
    }
}