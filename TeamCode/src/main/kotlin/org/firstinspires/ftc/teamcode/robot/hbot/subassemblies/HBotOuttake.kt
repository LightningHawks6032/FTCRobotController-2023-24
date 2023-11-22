package org.firstinspires.ftc.teamcode.robot.hbot.subassemblies

import org.firstinspires.ftc.teamcode.controlSystems.ActuatorPositionController
import org.firstinspires.ftc.teamcode.controlSystems.PID1D
import org.firstinspires.ftc.teamcode.ftcGlue.IHardwareMap
import org.firstinspires.ftc.teamcode.hardware.Motor
import org.firstinspires.ftc.teamcode.hardware.Servo
import org.firstinspires.ftc.teamcode.hardware.TandemGroup
import org.firstinspires.ftc.teamcode.util.*

private const val IN_PER_RAD_OUTTAKE_SLIDES = 1.0 // TODO

class HBotOuttake(
        doReverse: Boolean = false,
        private val pidCoefficients: PID1D.Coefficients,
) {
    private val lifterRef = TandemGroup.Motor(
            Motor("l0", Motor.PhysicalSpec.GOBILDA_5202_0002_0003, Motor.Config { reversed = doReverse }),
            Motor("l1", Motor.PhysicalSpec.GOBILDA_5202_0002_0003, Motor.Config { reversed = doReverse.not() }),
    )
    private val outputServosRef = TandemGroup.Servo(
            Servo("or", continuousRotation = false, Servo.Config { reversed = false }), // TODO
            Servo("ol", continuousRotation = false, Servo.Config { reversed = true }),
    )

    private var outputServosOpen = false

    inner class Impl(hardwareMap: IHardwareMap) {
         val lifter = lifterRef.Impl(hardwareMap)
        private val outputServos = outputServosRef.Impl(hardwareMap)

        private fun onEditZeroPos(it: ()->Unit) {
            controller.posZeroPointEdit(it)
        }

        var pos by lifter::pos.delegate()
                .withWriteEffect { onEditZeroPos(it) }
                .times { IN_PER_RAD_OUTTAKE_SLIDES }
        private var power by lifter::power

        val controller = ActuatorPositionController(pidCoefficients, this::power, this::pos)
        fun tick(dt: Double) = controller.tick(dt)

        var outputServosOpen by this@HBotOuttake::outputServosOpen.delegate().withAfterWriteEffect { open ->
            outputServos.pos = if (open) 0.0 else 1.0
        }
        @NotForCompetition
        fun debugControlOutputServos() = outputServos

        init {
            // update servo positions
            outputServosOpen = this@HBotOuttake.outputServosOpen
        }
    }
}