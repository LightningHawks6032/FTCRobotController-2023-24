package org.firstinspires.ftc.teamcode.util

import kotlinx.coroutines.delay
import org.firstinspires.ftc.teamcode.LOpMode
import org.firstinspires.ftc.teamcode.event.WatchList
import java.lang.Integer.max
import kotlin.math.pow
import kotlin.math.roundToInt

/**
 * Debugging class that allows setting global variables
 * with String names. Intended for quick iteration.
 *
 * DEBUGGING USE ONLY! This is far from a clean way to
 * make good code, it's just meant to be easy.
 */
object DebugVars {
    val int = mutableMapOf<String, Int>()
    val double = mutableMapOf<String, Double>()
    val bool = mutableMapOf<String, Boolean>()
}

private val <T : Any> LOpMode<T>.RunScope.buttonAIsHeld
    get() = gamepadA.a.isHeld

private suspend fun <T : Any> LOpMode<T>.RunScope.delayUntilBumperRelease() {
    while (buttonAIsHeld) {
        delay(20)
    }
}

suspend fun <T : Any> LOpMode<T>.RunScope.selectDebugInt(name: String, defaultVal: Int = 0) {
    var value = DebugVars.int[name] ?: defaultVal
    var incrementPlaceNumber = 0
    val changeWatchList = WatchList()
    val dpadUp = gamepadA.dpad.up.Watch(changeWatchList)
    val dpadDown = gamepadA.dpad.down.Watch(changeWatchList)
    val dpadLeft = gamepadA.dpad.left.Watch(changeWatchList)
    val dpadRight = gamepadA.dpad.right.Watch(changeWatchList)
    while (!buttonAIsHeld) {
        changeWatchList.tick()
        if (dpadLeft.justPressed) incrementPlaceNumber++
        if (dpadRight.justPressed) incrementPlaceNumber = max(incrementPlaceNumber - 1, 0)
        val incrementPlaceValue = (10.0.pow(incrementPlaceNumber)).roundToInt()
        if (dpadUp.justPressed) value += incrementPlaceNumber
        if (dpadDown.justPressed) value -= incrementPlaceNumber
        if (gamepadA.x.isHeld) value = defaultVal
        withTelemetry {
            ln("$name = $value")
            ln("[dpad(up/down)] increment by +/- $incrementPlaceValue")
            ln("[dpad(left)] increment step *= 10")
            ln("[dpad(right)] increment step /= 10")
            ln("[a] accept value")
            ln("[x] revert to default")
        }
        delay(20)
    }
    DebugVars.int[name] = value
    withTelemetry {
        ln("$name = $value")
        ln("successfully set value")
    }
    delayUntilBumperRelease()
}
suspend fun <T : Any> LOpMode<T>.RunScope.selectDebugBool(name: String, defaultVal: Boolean = false) {
    var value = DebugVars.bool[name] ?: defaultVal
    while (!buttonAIsHeld) {
        if (gamepadA.dpad.up.isHeld) value = true
        if (gamepadA.dpad.down.isHeld) value = false
        if (gamepadA.x.isHeld) value = defaultVal
        withTelemetry {
            ln("$name = $value")
            ln("[dpad(up)] set to true")
            ln("[dpad(down)] set to false")
            ln("[dpad(right)] increment step /= 10")
            ln("[a] accept value")
            ln("[x] revert to default")
        }
        delay(20)
    }
    DebugVars.bool[name] = value
    withTelemetry {
        ln("$name = $value")
        ln("successfully set value")
    }
    delayUntilBumperRelease()
}