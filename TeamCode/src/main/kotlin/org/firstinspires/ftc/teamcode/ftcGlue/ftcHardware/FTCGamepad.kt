package org.firstinspires.ftc.teamcode.ftcGlue.ftcHardware

import com.qualcomm.robotcore.hardware.Gamepad
import org.firstinspires.ftc.teamcode.ftcGlue.IGamepad
import org.firstinspires.ftc.teamcode.util.Vec2
import org.firstinspires.ftc.teamcode.util.delegate
import org.firstinspires.ftc.teamcode.util.toDouble

class FTCGamepad(
        gamepad: Gamepad,
) : IGamepad {
    override val x by gamepad::x
    override val y by gamepad::y
    override val a by gamepad::a
    override val b by gamepad::b

    override val dpadDown by gamepad::dpad_down
    override val dpadUp by gamepad::dpad_up
    override val dpadLeft by gamepad::dpad_left
    override val dpadRight by gamepad::dpad_right

    override val stickLeftButton by gamepad::left_stick_button
    override val stickLeft get() = Vec2(stickLeftX.toDouble(), stickLeftY.toDouble())
    private val stickLeftX by gamepad::left_stick_x
    private val stickLeftY by gamepad::left_stick_y

    override val stickRightButton by gamepad::right_stick_button
    override val stickRight get() = Vec2(stickRightX.toDouble(), stickRightY.toDouble())
    private val stickRightX by gamepad::right_stick_x
    private val stickRightY by gamepad::right_stick_y

    override val bumperLeft by gamepad::left_bumper
    override val bumperRight by gamepad::right_bumper

    override val triggerLeft by gamepad::left_trigger.delegate().toDouble()
    override val triggerRight by gamepad::right_trigger.delegate().toDouble()
}