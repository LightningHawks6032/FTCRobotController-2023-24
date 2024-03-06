package org.firstinspires.ftc.teamcode.hardware

import org.firstinspires.ftc.teamcode.ftcGlue.IHardwareMap
import org.firstinspires.ftc.teamcode.util.conditionallyZeroOut
import org.firstinspires.ftc.teamcode.util.delegate
import org.firstinspires.ftc.teamcode.util.minus
import org.firstinspires.ftc.teamcode.util.times

class Servo(
        private val id: String,
        val continuousRotation: Boolean,
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

    inner class Impl(hardwareMap: IHardwareMap) {
        private val iServo = hardwareMap.servos[id] ?: TODO("not found servo '$id'")

        private val reverseConst get() = if (reversed) -1.0 else 1.0


        /**
         * Getter/setter for the target position of the servo (zero if used on continuous rotation servo)
         *
         * Positive numbers are TODO counter clockwise looking through servo if not reversed.
         *
         * Range: `[-1, 1]`
         */
        var pos by (iServo::param.delegate() * { 2.0 } - { 1.0 }) // map position range [0,1] to [-1, 1]
                .times { reverseConst }
                .conditionallyZeroOut(0.0) { continuousRotation }
        /**
         * Getter/setter for the movement power of the servo (zero if not used on continuous rotation servo)
         *
         * Range: `[-1, 1]`
         */
        var power by iServo::param.delegate()
                .times { reverseConst }
                .conditionallyZeroOut(0.0) { !continuousRotation }
    }
}