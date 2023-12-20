package org.firstinspires.ftc.teamcode.test.motion

import org.firstinspires.ftc.teamcode.controlSystems.DriveController
import org.firstinspires.ftc.teamcode.controlSystems.PID1D
import org.firstinspires.ftc.teamcode.controlSystems.motionPath.buildPath
import org.firstinspires.ftc.teamcode.util.Vec2
import org.firstinspires.ftc.teamcode.util.Vec2Rot
import org.junit.Test
import kotlin.math.absoluteValue
import kotlin.math.pow
import kotlin.math.roundToInt

class DriveControllerTests {

    @Test
    fun pid() {
        val pid = PID1D(PID1D.Coefficients(0.5, 0.3, 0.9, 0.5))

        val data = mutableListOf<Double>()

        var x = 0.0
        var v = 0.0

        val tx = 1.0
        val tv = 0.0

        val dt = 0.05
        for (i in 0 until 1000) {
            v += pid.tick(x, v, tx, tv, dt) * dt
            x += v * dt

            data.add(x)

        }
        assert((x - tx).absoluteValue < 0.001)
        println("----------------------")
        println("x")
        for (x_ in data) {
            println("$x_")
        }
        println("----------------------")
    }

    @Test
    fun pathFollow() {
        val sim = VirtualSimulatedRobot(
                startPos = Vec2Rot(10.0, 5.0, 1.0),
                mass = 10.0,
                momentOfInertia = 5000.0,
                muKinetic = 0.03,
                muStatic = 0.04,
        )

        val drive = DriveController(
                sim, sim,
                PID1D.Coefficients(0.6, 0.2, 0.9, 0.5),
                PID1D.Coefficients(0.6, 0.2, 0.9, 0.5),
                Vec2Rot(20.0, 20.0, 25000.0),
        )

        val duration = 10.0
        val dt = 0.05

        val data = mutableListOf<Pair<Triple<Double, Double, Double>, Triple<Double, Double, Double>>>()

        val path = buildPath(sim.pos, Vec2Rot.zero) {
            bezierR(UntilAt(2.0.seconds), 0.0, 0.0)
            stationaryR(UntilAt(5.0.seconds))
            bezierR(UntilAt(7.0.seconds), -3.0, -1.0)
            bezierR(UntilAt(10.0.seconds), -2.0, 0.0)

            bezierXY(UntilAt(3.0.seconds), toPos = Vec2(0.0, 0.0), Vec2.zero)
            stationaryXY(UntilAt(5.0.seconds))
            bezierXY(UntilAt(6.0.seconds), toPos = Vec2(10.0, 0.0), Vec2(0.0, 10.0) * Math.PI / 2.0)
            bezierXY(UntilAt(7.0.seconds), toPos = Vec2(0.0, 10.0), Vec2(-10.0, 0.0) * Math.PI / 2.0)
            bezierXY(UntilAt(8.0.seconds), toPos = Vec2(-10.0, 0.0), Vec2(0.0, -10.0) * Math.PI / 2.0)
            bezierXY(UntilAt(9.0.seconds), toPos = Vec2(0.0, -10.0), Vec2(10.0, 0.0) * Math.PI / 2.0)
            bezierXY(UntilAt(10.0.seconds), toPos = Vec2(0.0, 0.0), Vec2.zero)
        }

        drive.path = path
        drive.t = 0.0

        for (t in 0 until (duration / dt).roundToInt()) {
            drive.tick(dt)

            data.add(Pair(
                    Triple(sim.pos.v.x, sim.pos.v.y, sim.pos.r),
                    Triple(drive.targetPos.v.x, drive.targetPos.v.y, drive.targetPos.r),
            ))

            assert((sim.pos - drive.targetPos).let { it.v.magSq + it.r.pow(2) } < 2.0.pow(2))
        }

        println("----------------------")
        println("x\ty\tr\ttx\tty\ttr\t")
        for ((p, tp) in data) {
            val (x, y, r) = p
            val (tx, ty, tr) = tp
            println("$x\t$y\t$r\t$tx\t$ty\t$tr\t")
        }
        println("----------------------")
    }
}