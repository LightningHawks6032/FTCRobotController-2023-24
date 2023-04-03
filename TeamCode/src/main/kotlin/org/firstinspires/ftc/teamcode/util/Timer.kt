package org.firstinspires.ftc.teamcode.util

class Timer {
    /** shorthand for `System.currentTimeMillis()` */
    private val sysTimeMillis:Long get() = System.currentTimeMillis()

    /** The current time value, representing time to return
     * if stopped and time of start if running. */
    private var t = 0L
    /** Whether or not this timer is timing. */
    var isTiming = false
        set(value) {
            t = if (value)
                sysTimeMillis - nowMillis // time as if we started now milliseconds ago
            else
                nowMillis // hold at now
            field = value
        }

    /** The timer's current time in milliseconds. */
    val nowMillis:Long get() = if (isTiming) sysTimeMillis - t else t
    /** The timer's current time in seconds. */
    val now:Double get() = nowMillis / MILLISECONDS_PER_SECOND

    private companion object {
        const val MILLISECONDS_PER_SECOND = 1000.0
    }
}