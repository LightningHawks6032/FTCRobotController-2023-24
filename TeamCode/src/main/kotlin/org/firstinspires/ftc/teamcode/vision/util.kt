package org.firstinspires.ftc.teamcode.vision

import android.graphics.Color
import android.graphics.Paint

class SimpleLinePaint(color: Int) : Paint() {
    init {
        setColor(color)
        isAntiAlias = true
        strokeCap = Cap.ROUND
        strokeWidth = 3.0f
    }

    companion object {
        val white = SimpleLinePaint(Color.WHITE)
        val green = SimpleLinePaint(Color.GREEN)
    }
}