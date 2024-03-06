package org.firstinspires.ftc.teamcode.hardware

import com.qualcomm.robotcore.util.RobotLog
import org.firstinspires.ftc.teamcode.ftcGlue.IHardwareMap
import org.firstinspires.ftc.teamcode.util.*
import kotlin.math.absoluteValue

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

    val minTorque = config.minTorque

    private var zeroPos = 0.0

    class Config {
        var reversed = false
        var useEncoder = true

        var minTorque = 0.0

        companion object {
            val default get() = Config()
            operator fun invoke(block: Config.() -> Unit) =
                    Config().also(block)
        }
    }

    companion object {
        @NotForCompetition
        const val DEBUG_ENCODER_RESOLUTION = 1000.0
    }

    enum class PhysicalSpec(
            /**
             * Motor encoder resolution in `ticks / rev`.
             * - "PPR at the Output Shaft" on gobilda spec sheet
             */
            encoderResolutionPPR: Double,

            /** Speed of the motor without anything on it's output shaft at 12 volts (motor.power = 1.0). (in RPM) */
            noLoadSpeedRPM: Double,
            /** Torque of the motor at which point is stops moving at 12 volts (motor.power = 1.0). (in Kg-cm) */
            stallTorqueKgCM: Double,

            /** Marks this spec as only containing an encoder. */
            val encoderOnly: Boolean = false,
    ) {
        GOBILDA_5202_0002_0001(28.0, 6000.0, 1.5),
        GOBILDA_5202_0002_0003(103.8, 1620.0, 5.4),
        GOBILDA_5202_0002_0005(145.1, 1150.0, 7.9),
        GOBILDA_ODOMETRY_POD(2000.0, 0.0, 0.0, encoderOnly = true),
        REV_THROUGH_BORE_ENCODER(8192.0, 0.0, 0.0, encoderOnly = true),

        @NotForCompetition
        DEBUG(DEBUG_ENCODER_RESOLUTION, 0.0, 0.0);

        /** Motor encoder resolution in `rad / tick`.*/
        val encoderRadPerTick = (2 * Math.PI) / encoderResolutionPPR // (2pi rad / 1 rev) * (rev / tick)

        /** Speed of the motor without anything on it's output shaft at 12 volts (motor.power = 1.0). (in rad / s) */
        val noLoadSpeedRadPerSec = (2 * Math.PI) * noLoadSpeedRPM / 60 // (2pi rad / 1 rev) / (60 sec / 1 min)

        /** Torque of the motor at which point is stops moving at 12 volts (motor.power = 1.0). (in Newton-cm) */
        val stallTorqueNCM = 9.80665 * stallTorqueKgCM
    }

    inner class Impl(hardwareMap: IHardwareMap) {
        private val idcMotor = hardwareMap.dcMotors[id] ?: TODO("not found motor '$id'")

        private val reverseConst get() = if (reversed) -1.0 else 1.0
        var power by (
                idcMotor::power.delegate()
                        .conditionallyAllowWriting(
                                allowedWhen = this@Motor::useMotor.delegate()
                        ).times { reverseConst }
                )

        fun setTorque(torqueNCM: Double, currentVel: Double) {
            val powerNext = (
                    torqueNCM / motorSpec.stallTorqueNCM + currentVel / motorSpec.noLoadSpeedRadPerSec
                    ).clamp(-1.0, 1.0)
            power = if (torqueNCM.absoluteValue < minTorque) {
                0.0
            } else {
                powerNext
            }
        }

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