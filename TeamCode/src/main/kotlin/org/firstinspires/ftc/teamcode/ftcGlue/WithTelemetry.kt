package org.firstinspires.ftc.teamcode.ftcGlue

import org.firstinspires.ftc.robotcore.external.Telemetry

class WithTelemetry(
        private val telemetry: Telemetry,
) {
    class Partial {
        var currentText = ""

        operator fun invoke(block: WithTelemetry.Scope.()->Unit) {
            currentText = WithTelemetry.Scope().also(block).telemetryText
        }
    }

    fun EXAMPLE(f : Telemetry) {
        val the = WithTelemetry(f)


        the {
            ln("uefsidsfui")
        }
    }

    class Scope internal constructor() {
        internal var telemetryText = ""
        fun ln(ln: String) { telemetryText += ln + "\n" }
        fun ln(ln: String, v: Any) = ln("$ln: $v")

        operator fun Partial.unaryPlus() = ln(this@unaryPlus.currentText)
    }
    operator fun invoke(block: Scope.()->Unit) {
        telemetry.clear()
        telemetry.addLine(Scope().also(block).telemetryText)
        telemetry.update()
    }
}