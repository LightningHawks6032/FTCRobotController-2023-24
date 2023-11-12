package org.firstinspires.ftc.teamcode.ftcGlue

import org.firstinspires.ftc.robotcore.external.Telemetry

class WithTelemetry(
        private val telemetry: Telemetry,
) {
    class Partial {
        var currentText = ""

        operator fun invoke(block: Scope.() -> Unit) {
            currentText = Scope().also(block).telemetryText
        }
    }

    class Scope internal constructor() {
        internal var telemetryText = ""
        fun ln(ln: String) {
            telemetryText += ln + "\n"
        }

        fun ln() = ln("")

        fun ln(ln: String, v: Any) = ln("$ln: $v")

        operator fun Partial.unaryPlus() = ln(this@unaryPlus.currentText)
    }

    operator fun invoke(printToConsole: Boolean = false, block: Scope.() -> Unit) {
        telemetry.clear()
        val text = Scope().also(block).telemetryText
        telemetry.addLine(text)
        if (printToConsole) {
            println("//////////// TELEMETRY OUTPUT ////////////\n$text")
        }
        telemetry.update()
    }
}