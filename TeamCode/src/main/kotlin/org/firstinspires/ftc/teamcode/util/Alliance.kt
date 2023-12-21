package org.firstinspires.ftc.teamcode.util

import org.firstinspires.ftc.teamcode.LOpMode

enum class Alliance {
    Red, Blue;
    fun allianceName() = when (this) {
        Red -> "Red"
        Blue -> "Blue"
    }
}


suspend fun <R: Any> LOpMode<R>.RunScope.promptSelectAlliance(): Alliance {
    val untilSelectedLock = TriggerLock()
    var alliance: Alliance? = null

    createLoop({ untilSelectedLock.locked }) {
        watches(gamepadA.a::Watch) {
            it.pressed.bind {
                if (alliance != null)
                    untilSelectedLock.unlock()
            }
        }
        withTelemetry {
            ln(alliance?.let {
                "Selected ${it.allianceName()} alliance."
            } ?: "No alliance selected!")

            ln("gamepadA. b -> red")
            if (gamepadA.b.isHeld)
                alliance = Alliance.Red
            ln("gamepadA. x -> blue")
            if (gamepadA.x.isHeld)
                alliance = Alliance.Blue

            if (alliance == null) {
                ln()
                ln("!! SELECT AN ALLIANCE !!\n".repeat(5))
            }
        }
    }

    untilSelectedLock.wait()

    return alliance ?: throw Error("ALLIANCE WAS NOT SELECTED")
}