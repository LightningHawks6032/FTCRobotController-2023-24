package org.firstinspires.ftc.teamcode.hardware.motion

import org.firstinspires.ftc.teamcode.ftcGlue.IHardwareMap
import org.firstinspires.ftc.teamcode.util.DeltaValue
import org.firstinspires.ftc.teamcode.util.Vec2Rot
import kotlin.math.cos

class MecanumWheelOdometry(
        val mecanumDrive: MecanumDrive,
        val wheelAngleOffPerp: Double,
) : IOdometry() {
    override fun tick(dt: Double) {
        val delta = currentImpl?.readDelta() ?: run {
            println("attempted to tick odometry without initializing hardware")
            return
        }

        val prevVel = vel
        val prevPos = pos
        val count = 50
        // rotation makes things weird, approximate the behavior of spinning and translating
        // at the same time by just adding a little bit at a time, calculus style.
        for (iteration in 0 until count) {
            val miniDelta = delta / count.toDouble()
            pos += robot2worldTransform().transformVelFwd(miniDelta)
        }
        vel = (pos - prevPos) / dt
        acc = (vel - prevVel) / dt
    }

    override fun assertPosition(newPos: Vec2Rot) {
        pos = newPos
        vel = Vec2Rot.zero
        acc = Vec2Rot.zero
    }

    override fun nudge(newPos: Vec2Rot) {
        val deltaHeading = newPos.r - pos.r
        pos = newPos
        vel = vel.transformP { it.rotate(deltaHeading) }
        acc = acc.transformP { it.rotate(deltaHeading) }
    }

    private var currentImpl: Impl? = null

    inner class Impl(hardwareMap: IHardwareMap) {
        private val flReader = mecanumDrive.flRef.Impl(hardwareMap)
        private val frReader = mecanumDrive.frRef.Impl(hardwareMap)
        private val blReader = mecanumDrive.blRef.Impl(hardwareMap)
        private val brReader = mecanumDrive.brRef.Impl(hardwareMap)

        private val deltaFL by DeltaValue.Double(flReader::pos)
        private val deltaFR by DeltaValue.Double(frReader::pos)
        private val deltaBL by DeltaValue.Double(blReader::pos)
        private val deltaBR by DeltaValue.Double(brReader::pos)

        init {
            currentImpl = this
        }

        /** Get the estimated change in position in robot local space. */
        fun readDelta(): Vec2Rot {
            val fl = deltaFL * mecanumDrive.wheelRadius
            val fr = deltaFR * mecanumDrive.wheelRadius
            val bl = deltaBL * mecanumDrive.wheelRadius
            val br = deltaBR * mecanumDrive.wheelRadius
            val k = cos(wheelAngleOffPerp) / mecanumDrive.wheelDisplacement

            // let R = r * k
            // fl = y + x - R
            // fr = y - x + R
            // bl = y - x - R
            // br = y + x + R

            // (fl + fr)/2 = (bl + br)/2 = y
            // (fl - bl)/2 = (br - fr)/2 = x
            // (br - fl)/2 = (fr - bl)/2 = R

            val deltaY = (fl+fr+bl+br)/4
            val deltaX = (fl + br - fr - bl)/4
            val deltaR = (fr + br - fl - bl)/4 * mecanumDrive.wheelDisplacement / cos(wheelAngleOffPerp)
            return mecanumDrive.assembly2robotTransform.transformVelFwd(
                    Vec2Rot(deltaX, deltaY, deltaR)
            )
        }
    }
}