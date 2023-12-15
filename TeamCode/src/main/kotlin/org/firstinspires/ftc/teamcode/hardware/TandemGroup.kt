package org.firstinspires.ftc.teamcode.hardware

import org.firstinspires.ftc.teamcode.ftcGlue.IHardwareMap

interface TandemGroup {
    var pos: Double
    var power: Double

    class Motor(
            private val leaderRef: org.firstinspires.ftc.teamcode.hardware.Motor,
            private vararg val followerRefs: org.firstinspires.ftc.teamcode.hardware.Motor
    ) {
        inner class Impl(hardware: IHardwareMap) : TandemGroup {
            private val leader = leaderRef.Impl(hardware)
            private val followers = followerRefs.map { it.Impl(hardware) }

            override var pos by leader::pos
            override var power: Double
                get() = leader.power
                set(power) {
                    leader.power = power
                    for (motor in followers) {
                        motor.power = power
                    }
                }
            fun setTorque(torque: Double, currentVel: Double) {
                val dividedTorque = torque / (followers.size + 1)
                leader.setTorque(dividedTorque, currentVel)
                for (follower in followers) {
                    follower.setTorque(dividedTorque, currentVel)
                }
            }
        }
    }
    class Servo(
            private val leaderRef: org.firstinspires.ftc.teamcode.hardware.Servo,
            private vararg val followerRefs: org.firstinspires.ftc.teamcode.hardware.Servo
    ) {
        inner class Impl(hardware: IHardwareMap) : TandemGroup {
            private val leader = leaderRef.Impl(hardware)
            private val followers = followerRefs.map { it.Impl(hardware) }

            override var pos
                get() = leader.pos
                set(pos) {
                    leader.pos = pos
                    for (servo in followers) {
                        servo.pos = pos
                    }
                }
            override var power: Double
                get() = leader.power
                set(power) {
                    leader.power = power
                    for (motor in followers) {
                        motor.power = power
                    }
                }

        }
    }
}