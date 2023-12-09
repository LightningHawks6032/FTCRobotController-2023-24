package org.firstinspires.ftc.teamcode.test.motion

import org.firstinspires.ftc.teamcode.hardware.Motor
import org.firstinspires.ftc.teamcode.hardware.motion.ThreeWheelOdometry
import org.firstinspires.ftc.teamcode.test.ftcGlue.CIDCMotor
import org.firstinspires.ftc.teamcode.test.ftcGlue.CIHardwareMap
import org.firstinspires.ftc.teamcode.util.NotForCompetition
import org.firstinspires.ftc.teamcode.util.Transform2D
import org.firstinspires.ftc.teamcode.util.Vec2Rot
import org.junit.Test
import kotlin.math.*

@OptIn(NotForCompetition::class)
class ThreeWheelOdometryTests {


    private class VirtualOdo(hm: CIHardwareMap = CIHardwareMap()) {
        private val cix0 = hm.dcMotors["x0"] as CIDCMotor
        private val cix1 = hm.dcMotors["x1"] as CIDCMotor
        private val ciy = hm.dcMotors["y"] as CIDCMotor
        private var x0Pos = 0.0
        private var x1Pos = 0.0
        private var yPos = 0.0

        private val yReaderPos = 1.234
        private val x0ReaderPos = 3.456
        private val x1ReaderPos = -6.789

        private val odo = ThreeWheelOdometry(
                Vec2Rot.zero,
                yReaderPos,
                x0ReaderPos,
                x1ReaderPos,
                Motor.PhysicalSpec.DEBUG,
                wheelRadiiInches = 1.0,
                ThreeWheelOdometry.ReversalPattern(),
                ThreeWheelOdometry.Ids("y", "x0", "x1")
        ).also { it.Impl(hm) }

        private var robot2world = Transform2D.local2outerFromLocation(Vec2Rot.zero)
        fun resetPos(pos: Vec2Rot) {
            robot2world = Transform2D(pos)
            odo.assertPosition(pos)
        }

        fun getRealPos() = robot2world.local2outerToLocation()
        fun getMeasuredPos() = odo.pos

        fun updateRealPos(destination: Vec2Rot, miniSteps: Int = 100) {
            val delta = destination - robot2world.local2outerToLocation()
            for (i in 0 until miniSteps) {
                applyGlobalDelta(delta / miniSteps.toDouble())
            }
            cix0.pos = (x0Pos * Motor.DEBUG_ENCODER_RESOLUTION / (PI * 2)).roundToInt()
            cix1.pos = (x1Pos * Motor.DEBUG_ENCODER_RESOLUTION / (PI * 2)).roundToInt()
            ciy.pos = (yPos * Motor.DEBUG_ENCODER_RESOLUTION / (PI * 2)).roundToInt()
            println("px $x0Pos, ${cix0.pos}, ${x0Pos * Motor.DEBUG_ENCODER_RESOLUTION}")

        }

        fun tick(dt: Double) {
            odo.tick(dt)
        }

        private fun applyGlobalDelta(delta: Vec2Rot) {
            robot2world = Transform2D.local2outerFromLocation(
                    robot2world.local2outerToLocation() + delta
            )
            val localDelta = robot2world.transformVelInv(delta)
            val deltaX = localDelta.v.x
            val deltaY = localDelta.v.y
            val deltaR = localDelta.r

            x0Pos += deltaX - deltaR * x0ReaderPos
            x1Pos += deltaX - deltaR * x1ReaderPos
            yPos += deltaY + deltaR * yReaderPos
        }

        init {
            resetPos(Vec2Rot.zero)
        }
    }

    private inline fun iterTime(totalTime: Double, dt: Double, callback: (t: Double, dt: Double) -> Unit) {
        for (t in 0 until (totalTime / dt).roundToInt()) {
            println("n $t")
            callback(t.toDouble() * dt, dt)
        }
    }

    private fun testGoStraight(direction: Vec2Rot) {
        val odo = VirtualOdo()

        iterTime(2.0, 0.05) { t, dt ->
            odo.updateRealPos(direction * t)
            odo.tick(dt)
            println("""
                    measured: ${odo.getMeasuredPos()}
                    real: ${odo.getRealPos()}
                """.trimIndent())

            assert((odo.getMeasuredPos() - odo.getRealPos()).let {
                it.v.magSq + it.r.pow(2)
            } < 1.0e-2) {
                """
                    odometry sim failed
                    measured: ${odo.getMeasuredPos()}
                    real: ${odo.getRealPos()}
                """.trimIndent()
            }
        }
    }


    private fun testGoCircle(r: Double, winding: Double) {
        val odo = VirtualOdo()

        iterTime(2.0 * PI, 0.05) { t, dt ->
            odo.updateRealPos(Vec2Rot((sin(t))*r, (1 - cos(t))*r, t*(winding+1)))
            odo.tick(dt)
            println("""
                    measured: ${odo.getMeasuredPos()}
                    real: ${odo.getRealPos()}
                """.trimIndent())

            assert((odo.getMeasuredPos() - odo.getRealPos()).let {
                it.v.magSq + it.r.pow(2)
            } < 1.0e-2) {
                """
                    odometry sim failed
                    measured: ${odo.getMeasuredPos()}
                    real: ${odo.getRealPos()}
                """.trimIndent()
            }
        }
    }

    @Test
    fun testGoStraightInManyDirections() {
        testGoStraight(Vec2Rot(1.0, 0.0, 0.0))
        testGoStraight(Vec2Rot(0.0, 1.0, 0.0))
        testGoStraight(Vec2Rot(0.0, 0.0, 1.0))
        testGoStraight(Vec2Rot(-2.0, 1.3, 0.0))
        testGoStraight(Vec2Rot(-2.0, 1.3, 2.2))
        testGoCircle(1.0, 0.0)
        testGoCircle(2.0, 1.0)
    }
}