package org.firstinspires.ftc.teamcode.hardware.motion

import org.firstinspires.ftc.teamcode.util.Vec2Rot

interface IDrive {
    /** Desired motor power in robot space */
    var power: Vec2Rot
}