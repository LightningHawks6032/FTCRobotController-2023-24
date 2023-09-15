package org.firstinspires.ftc.teamcode.util

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