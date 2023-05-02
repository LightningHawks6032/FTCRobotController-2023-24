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
) : Odometry() {
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


    override fun readDeltaFromImpl(dt: Double) = currentImpl?.readDelta(dt)

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