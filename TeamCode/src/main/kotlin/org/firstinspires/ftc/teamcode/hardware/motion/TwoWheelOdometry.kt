package org.firstinspires.ftc.teamcode.hardware.motion

import org.firstinspires.ftc.teamcode.ftcGlue.IHardwareMap
import org.firstinspires.ftc.teamcode.hardware.IMU
import org.firstinspires.ftc.teamcode.hardware.Motor
import org.firstinspires.ftc.teamcode.util.DeltaValue
import org.firstinspires.ftc.teamcode.util.Vec2Rot

class TwoWheelOdometry(
        val center: Vec2Rot,
        val xReaderPos: Double = 1.0,
        val yReaderPos: Double = 1.0,
        spec: Motor.PhysicalSpec,
        reversalPattern: ReversalPattern,
        ids: Ids,
) : IOdometry() {
    val xReader = Motor(ids.x, spec, Motor.Config { useEncoder = true; reversed = reversalPattern.x })
    val yReader = Motor(ids.y, spec, Motor.Config { useEncoder = true; reversed = reversalPattern.y })
    val imu = IMU(ids.imu, IMU.SpinAxis.VERTICAL)


    data class ReversalPattern(
            val x: Boolean = false,
            val y: Boolean = false,
    )

    data class Ids(
            val x: String,
            val y: String,
            val imu: String,
    ) {
        companion object {
            val default = Ids("x", "y", "Control Hub")
        }
    }


    override fun tick(dt: Double) {
        val delta = currentImpl?.readDelta(dt)
        val transform = robotLocTransform()

        val prevVel = vel
        vel = transform.localToGlobalVel(delta?.vel ?: Vec2Rot.zero)
        pos += transform.localToGlobalVel(delta?.dPos ?: Vec2Rot.zero)
        acc = (vel - prevVel) / dt
    }

    override fun assertPosition(newPos: Vec2Rot) {
        pos = newPos
        vel = Vec2Rot.zero
        acc = Vec2Rot.zero
    }

    override fun nudge(newPos: Vec2Rot) {
        acc = acc.transformP { it.rotate(newPos.r - pos.r) }
        vel = vel.transformP { it.rotate(newPos.r - pos.r) }
        pos = newPos
    }

    private var currentImpl: Impl? = null

    inner class Impl(hardwareMap: IHardwareMap) {
        private val xReader = this@TwoWheelOdometry.xReader.Impl(hardwareMap)
        private val yReader = this@TwoWheelOdometry.yReader.Impl(hardwareMap)
        private val imu = this@TwoWheelOdometry.imu.Impl(hardwareMap)

        private val deltaX by DeltaValue.Double(xReader::pos)
        private val deltaY by DeltaValue.Double(yReader::pos)
        private val deltaR by DeltaValue.Double(imu::spinAngle)

        init {
            currentImpl = this
        }

        fun readDelta(dt: Double): Delta {
            val deltaX = deltaX / xReaderPos
            val deltaY = deltaY / yReaderPos
            val deltaR = deltaR
            val deltaPosRaw = Vec2Rot(deltaX, deltaY, deltaR)

            // `rotate(center.r)` handles assembly rotation
            val deltaPos = deltaPosRaw.transformP { it.rotate(center.r) }

            return Delta(deltaPos, dt)
        }
    }
}