package org.firstinspires.ftc.teamcode.test

import org.firstinspires.ftc.teamcode.event.WatchList
import org.firstinspires.ftc.teamcode.hardware.Gamepad
import org.firstinspires.ftc.teamcode.test.ftcGlue.CIGamepad
import org.junit.Test

class GamepadTests {
    @Test
    fun buttonUpdatesWorkInIsolation() {
        val source = object {
            var state = false
        }
        var state by source::state
        val button = Gamepad.Button(source::state)
        val watchList = WatchList()

        var timesReleased = 0
        var timesPressed = 0
        var timesPressedCancelled = 0
        var timesPressedOnce = 0
        var timesChanged = 0

        button.Watch(watchList).also {
            it.changed.bind { isHeld ->
                assert(isHeld == state)
                timesChanged++
            }
            it.pressed.bind {
                assert(state)
                timesPressed++
            }
            it.pressed.bind {
                timesPressedCancelled++
            }.unbind()
            it.pressed.once {
                assert(state)
                timesPressedOnce++
            }
            it.released.bind {
                assert(!state)
                timesReleased++
            }
        }

        for (newState in booleanArrayOf( // starts `false`
                false,               // >> no change
                true, true, true,    // changed : 1x  |  pressed  : 1x
                false,               // changed : 2x  |  released : 1x
                true,                // changed : 3x  |  pressed  : 2x
                false,               // changed : 4x  |  released : 2x
                true,                // changed : 5x  |  pressed  : 3x
                false, false, false, // changed : 6x  |  released : 3x
                true, true,          // changed : 7x  |  pressed  : 4x
                false,               // changed : 8x  |  released : 4x
        )) {
            state = newState
            watchList.tick()
        }

        assert(timesChanged == 8) { "changed $timesChanged times instead of 9" }
        assert(timesPressed == 4) { "pressed $timesPressed times instead of 4" }
        assert(timesReleased == 4) { "released $timesReleased times instead of 4" }
        assert(timesPressedOnce == 1) { "EventSource.once not working, callback ran $timesPressedOnce times" }
        assert(timesPressedCancelled == 0) { "Cancellation not working, callback ran $timesPressedCancelled times" }
    }

    @Test
    fun gamepadWrapperClassForwardsChangesProperly() {
        val gamepad = CIGamepad()
        val gamepadWrapped = Gamepad(gamepad)
        val watchList = WatchList()

        var evCallCount = 0

        gamepad.a = false
        gamepad.b = false
        gamepad.x = false
        gamepad.y = false
        gamepadWrapped.a.Watch(watchList).pressed.once { evCallCount++ }
        gamepadWrapped.b.Watch(watchList).pressed.once { evCallCount++ }
        gamepadWrapped.x.Watch(watchList).pressed.once { evCallCount++ }
        gamepadWrapped.y.Watch(watchList).pressed.once { evCallCount++ }
        gamepad.a = true
        gamepad.b = true
        gamepad.x = true
        gamepad.y = true

        gamepad.bumperLeft = false
        gamepad.bumperRight = false
        gamepadWrapped.bumper.left.Watch(watchList).pressed.once { evCallCount++ }
        gamepadWrapped.bumper.right.Watch(watchList).pressed.once { evCallCount++ }
        gamepad.bumperLeft = true
        gamepad.bumperRight = true

        gamepad.stickLeftButton = false
        gamepad.stickRightButton = false
        gamepadWrapped.stick.left.button.Watch(watchList).pressed.once { evCallCount++ }
        gamepadWrapped.stick.right.button.Watch(watchList).pressed.once { evCallCount++ }
        gamepad.stickLeftButton = true
        gamepad.stickRightButton = true

        gamepad.dpadDown = false
        gamepad.dpadUp = false
        gamepad.dpadLeft = false
        gamepad.dpadRight = false
        gamepadWrapped.dpad.up.Watch(watchList).pressed.once { evCallCount++ }
        gamepadWrapped.dpad.down.Watch(watchList).pressed.once { evCallCount++ }
        gamepadWrapped.dpad.left.Watch(watchList).pressed.once { evCallCount++ }
        gamepadWrapped.dpad.right.Watch(watchList).pressed.once { evCallCount++ }
        gamepad.dpadDown = true
        gamepad.dpadUp = true
        gamepad.dpadLeft = true
        gamepad.dpadRight = true

        watchList.tick()

        assert(evCallCount == 12) { "some gamepad buttons don't work" }
    }
}