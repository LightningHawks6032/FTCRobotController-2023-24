package org.firstinspires.ftc.teamcode.hardware.motion

import org.firstinspires.ftc.teamcode.ftcGlue.IHardwareMap
import org.firstinspires.ftc.teamcode.hardware.Motor
import org.firstinspires.ftc.teamcode.util.DeltaValue
import org.firstinspires.ftc.teamcode.util.Transform2D
import org.firstinspires.ftc.teamcode.util.Vec2Rot

/*
Layout of the odometry setup:

        [x0]
     ---

       [center]  [y]
      +         |

        [x1]
     ---

y ^
  + >
    x
*/
class ThreeWheelOdometry(
        /** Offset of the center of the odometry setup in robot space. */
        val locationOnRobot: Vec2Rot,
        /** Offset in the +x direction, facing along the y axis (movement towards +y should be read as positive number) */
        val yReaderPos: Double = 1.0,
        /** Offset in the +y direction, facing along the x axis (movement towards +x should be read as positive number)  */
        val x0ReaderPos: Double = 1.0,
        /** Offset in the +y direction, facing along the x axis (movement towards +x should be read as positive number)  */
        val x1ReaderPos: Double = 1.0,
        spec: Motor.PhysicalSpec,
        val wheelRadiiInches: Double = 1.0,
        reversalPattern: ReversalPattern,
        ids: Ids,
) : IOdometry() {
    val yReader = Motor(ids.y, spec, Motor.Config { useEncoder = true; reversed = reversalPattern.y })
    val x0Reader = Motor(ids.x0, spec, Motor.Config { useEncoder = true; reversed = reversalPattern.x0 })
    val x1Reader = Motor(ids.x1, spec, Motor.Config { useEncoder = true; reversed = reversalPattern.x1 })

    val assembly2robotTransform = Transform2D.local2outerFromLocation(locationOnRobot)

    var x0nz = 0
    var x1nz = 0
    var ynz = 0

    override fun axesFunctional() = x0nz > 10 && x1nz > 10 && ynz > 10

    data class ReversalPattern(
            val y: Boolean = false,
            val x0: Boolean = false,
            val x1: Boolean = false,
    )

    data class Ids(
            val y: String,
            val x0: String,
            val x1: String,
    )


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
        private val yReader = this@ThreeWheelOdometry.yReader.Impl(hardwareMap)
        private val x0Reader = this@ThreeWheelOdometry.x0Reader.Impl(hardwareMap)
        private val x1Reader = this@ThreeWheelOdometry.x1Reader.Impl(hardwareMap)

        private val deltaY by DeltaValue.Double(yReader::pos)
        private val deltaX0 by DeltaValue.Double(x0Reader::pos)
        private val deltaX1 by DeltaValue.Double(x1Reader::pos)

        init {
            currentImpl = this
        }

        /** Get the estimated change in position in robot local space. */
        fun readDelta(): Vec2Rot {
            var deltaX0 = deltaX0 * wheelRadiiInches
            var deltaX1 = deltaX1 * wheelRadiiInches
            var deltaYR = deltaY * wheelRadiiInches

            println("DELTA x0:$deltaX0 x1:$deltaX1 y:$deltaYR")
            if (deltaX0 != 0.0) x0nz += 1
            if (deltaX1 != 0.0) x1nz += 1
            if (deltaYR != 0.0) ynz += 1

            // vx0 = -vr * x0p + vx ; vr = (vx - vx0) / x0p ; vx = vx0 + vr * x0p
            // vx1 = -vr * x1p + vx ; vr = (vx - vx1) / x1p ; vx = vx1 + vr * x1p
            // vy_ = vr * yp + vy   ; vr = (vy_ - vy) / yp  ; vy = vy_ - vr * yp

            //// vr = (vx - vx0) / x0p = (vx - vx1) / x1p
            // vx * (1 - x0p / x1p) = vx0 - vx1 * x0p / x1p
            // vx = (vx0 * x1p - vx1 * x0p) / (x1p - x0p)

            //// vx = vx0 + vr * x0p = vx1 + vr * x1p
            // vr * x0p - vr * x1p = vx1 - vx0
            // vr = (vx1 - vx0)/(x0p - x1p)
            //// vy = vy_ - vr * yp
            //

            val deltaR = (deltaX0 - deltaX1) / (x1ReaderPos - x0ReaderPos)
            deltaX0 += x0ReaderPos * deltaR
            deltaX1 += x1ReaderPos * deltaR
            deltaYR += -yReaderPos * deltaR

            val deltaX = (deltaX0 + deltaX1) / 2
            val deltaY = deltaYR
            return assembly2robotTransform.transformVelFwd(
                    Vec2Rot(deltaX, deltaY, deltaR)
            ) + Vec2Rot(locationOnRobot.v.rotate(-90.0) * deltaR,0.0) // account for moving reference frame
        }
    }
}