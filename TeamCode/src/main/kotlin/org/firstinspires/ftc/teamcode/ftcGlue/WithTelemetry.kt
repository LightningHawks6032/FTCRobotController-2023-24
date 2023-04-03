package org.firstinspires.ftc.teamcode.ftcGlue

import org.firstinspires.ftc.robotcore.external.Telemetry

class WithTelemetry(
        private val telemetry: Telemetry,
) {
    inner class Scope {
        fun ln(ln: String) = telemetry.addLine(ln)
        fun ln(ln: String, v: Any) = ln("$ln: $v")
    }
    operator fun invoke(block: Scope.()->Unit) {
        telemetry.clear()
        block(Scope())
        telemetry.update()
    }
}