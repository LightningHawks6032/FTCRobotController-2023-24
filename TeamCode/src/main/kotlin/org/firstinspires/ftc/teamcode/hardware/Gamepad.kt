package org.firstinspires.ftc.teamcode.hardware

import org.firstinspires.ftc.teamcode.event.EventSender
import org.firstinspires.ftc.teamcode.event.WatchList
import org.firstinspires.ftc.teamcode.event.send
import org.firstinspires.ftc.teamcode.ftcGlue.IGamepad
import org.firstinspires.ftc.teamcode.util.Vec2
import kotlin.reflect.KProperty0

class Gamepad(
        private val gamepad: IGamepad,
) {
    val a = Button(gamepad::a)
    val b = Button(gamepad::b)
    val x = Button(gamepad::x)
    val y = Button(gamepad::y)

    val trigger = LateralInput(
            left = gamepad::triggerLeft,
            right = gamepad::triggerRight,
    )
    val bumper = LateralInput(
            left = gamepad::bumperLeft,
            right = gamepad::bumperRight,
    )
    private val stickLeft = Stick(gamepad::stickLeftButton, gamepad::stickLeft)
    private val stickRight = Stick(gamepad::stickRightButton, gamepad::stickRight)
    val stick = LateralInput(
            left = this::stickLeft,
            right = this::stickRight,
    )


    class Button(isHeld: KProperty0<Boolean>) {
        val isHeld by isHeld
        inner class Watch(
                list: WatchList,
        ): WatchList.Watchable {
            private var wasHeld = isHeld
            var justPressed = false
                private set
            var justReleased = false
                private set

            private val changedSender = EventSender.Default<Boolean>()
            val changed: EventSender<Boolean> = changedSender
            private val pressedSender = EventSender.Default<Unit>()
            val pressed: EventSender<Unit> = pressedSender
            private val releasedSender = EventSender.Default<Unit>()
            val released: EventSender<Unit> = releasedSender

            override fun tick() {
                justPressed = false
                justReleased = false
                if (wasHeld != isHeld) {
                    if (isHeld) {
                        justPressed = true
                        pressedSender.send()
                    } else {
                        justReleased = true
                        releasedSender.send()
                    }
                    changedSender.send(isHeld)
                }
                wasHeld = isHeld
            }

            init {
                list.add(this)
            }
        }
    }
    class Stick(
            button: KProperty0<Boolean>,
            pos: KProperty0<Vec2>,
    ) {
        val button = Button(button)
        val pos by pos
    }
    class LateralInput<T>(
            left: KProperty0<T>,
            right: KProperty0<T>,
    ) {
        val left by left
        val right by right
    }
}