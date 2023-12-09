package org.firstinspires.ftc.teamcode.test.motion

import org.firstinspires.ftc.teamcode.controlSystems.DriveController
import org.firstinspires.ftc.teamcode.controlSystems.PID1D
import org.firstinspires.ftc.teamcode.util.Vec2Rot
import kotlin.math.roundToInt

class DriveControllerTests {

//    @Test TODO
    fun theRobot() {
        val sim = VirtualSimulatedRobot(mass = 10.0, momentOfInertia = 0.5, muKinetic = 0.03, muStatic = 0.04)

        val drive = DriveController(sim, sim, PID1D.Coefficients(0.3,0.1,0.9,0.0,0.0,0.0))

        val duration = 10.0
        val dt = 0.05

        val data = mutableListOf<Triple<Double,Double,Double>>()

        drive.targetPos = Vec2Rot(24.0, 0.01, -0.01)
        for (t in 0 until (duration / dt).roundToInt()) {
            drive.tick(dt)

            data.add(Triple(sim.pos.v.x, sim.pos.v.y, sim.pos.r))
        }

        println("----------------------")
        println("x\ty\tr\t")
        for ((x,y,r) in data) {
            println("$x\t$y\t$r\t")
        }
        println("----------------------")
    }
}