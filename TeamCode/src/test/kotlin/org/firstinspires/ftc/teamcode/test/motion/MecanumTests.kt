package org.firstinspires.ftc.teamcode.test.motion

import org.firstinspires.ftc.teamcode.hardware.Motor
import org.firstinspires.ftc.teamcode.hardware.motion.MecanumDrive
import org.firstinspires.ftc.teamcode.test.ftcGlue.CIHardwareMap
import org.firstinspires.ftc.teamcode.util.Vec2
import org.firstinspires.ftc.teamcode.util.Vec2Rot
import org.junit.Test

class MecanumTests {
    @Test
    fun drivePowerTranslatedProperly() {
        val hardware = CIHardwareMap()
        val ids = MecanumDrive.Ids.default
        val spec = Motor.PhysicalSpec.GOBILDA_5202_0002_0005

        // get mecanum reference
        val mecanum = MecanumDrive(
                spec,
                MecanumDrive.ReversalPattern(),
                ids,
        ).Impl(hardware)

        // get motor references
        val fr = Motor(ids.fr, spec).Impl(hardware)
        val fl = Motor(ids.fl, spec).Impl(hardware)
        val br = Motor(ids.br, spec).Impl(hardware)
        val bl = Motor(ids.bl, spec).Impl(hardware)

        // brake
        mecanum.power = Vec2Rot.zero
        assert(fr.power == 0.0)
        assert(fl.power == 0.0)
        assert(br.power == 0.0)
        assert(bl.power == 0.0)

        // go forwards
        mecanum.power = Vec2Rot(Vec2(0.0, 1.0), 0.0)
        assert(fr.power == 1.0)
        assert(fl.power == 1.0)
        assert(br.power == 1.0)
        assert(bl.power == 1.0)

        // go backwards
        mecanum.power = Vec2Rot(Vec2(0.0, -1.0), 0.0)
        assert(fr.power == -1.0)
        assert(fl.power == -1.0)
        assert(br.power == -1.0)
        assert(bl.power == -1.0)

        // go right
        mecanum.power = Vec2Rot(Vec2(1.0, 0.0), 0.0)
        assert(fr.power == -1.0)
        assert(fl.power == 1.0)
        assert(br.power == 1.0)
        assert(bl.power == -1.0)

        // go clockwise
        mecanum.power = Vec2Rot(Vec2(0.0, 0.0), -1.0)
        assert(fr.power == -1.0)
        assert(fl.power == 1.0)
        assert(br.power == -1.0)
        assert(bl.power == 1.0)

        // go forwards + right
        mecanum.power = Vec2Rot(Vec2(0.5, 0.5), 0.0)
        assert(fr.power == 0.0)
        assert(fl.power == 1.0)
        assert(br.power == 1.0)
        assert(bl.power == 0.0)

        // go forwards + counter-clockwise
        mecanum.power = Vec2Rot(Vec2(0.0, 1.0), 1.0)
        assert(fr.power == 2.0)
        assert(fl.power == 0.0)
        assert(br.power == 2.0)
        assert(bl.power == 0.0)
    }

    @Test
    fun reversingWorksProperly() {
        reversingWorksProperly(
                MecanumDrive.ReversalPattern(), // no reversal
                frReversed = false,
                flReversed = false,
                brReversed = false,
                blReversed = false,
        )
        reversingWorksProperly(
                MecanumDrive.ReversalPattern(
                        all = true
                ),
                frReversed = true,
                flReversed = true,
                brReversed = true,
                blReversed = true,
        )
        reversingWorksProperly(
                MecanumDrive.ReversalPattern(
                        left = true
                ),
                frReversed = false,
                flReversed = true,
                brReversed = false,
                blReversed = true,
        )
        reversingWorksProperly(
                MecanumDrive.ReversalPattern(
                        right = true
                ),
                frReversed = true,
                flReversed = false,
                brReversed = true,
                blReversed = false,
        )
        reversingWorksProperly(
                MecanumDrive.ReversalPattern(
                        front = true
                ),
                frReversed = true,
                flReversed = true,
                brReversed = false,
                blReversed = false,
        )
        reversingWorksProperly(
                MecanumDrive.ReversalPattern(
                        back = true
                ),
                frReversed = false,
                flReversed = false,
                brReversed = true,
                blReversed = true,
        )
        reversingWorksProperly(
                MecanumDrive.ReversalPattern(
                        front = true,
                        right = true
                ),
                frReversed = false,
                flReversed = true,
                brReversed = true,
                blReversed = false,
        )
    }

    fun reversingWorksProperly(
            reversePattern: MecanumDrive.ReversalPattern,
            frReversed: Boolean,
            flReversed: Boolean,
            brReversed: Boolean,
            blReversed: Boolean
    ) {
        val hardware = CIHardwareMap()
        val ids = MecanumDrive.Ids.default
        val spec = Motor.PhysicalSpec.GOBILDA_5202_0002_0005

        // get mecanum reference
        val mecanum = MecanumDrive(
                spec,
                reversePattern,
                ids,
        ).Impl(hardware)

        // get motor references
        val fr = hardware.dcMotors[ids.fr]!!
        val fl = hardware.dcMotors[ids.fl]!!
        val br = hardware.dcMotors[ids.br]!!
        val bl = hardware.dcMotors[ids.bl]!!

        mecanum.power = Vec2Rot(Vec2(0.0, 1.0), 0.0)
        assert(fr.power == if (frReversed) -1.0 else 1.0) { "fr: ${fr.power} != ${if (frReversed) -1.0 else 1.0} " }
        assert(fl.power == if (flReversed) -1.0 else 1.0) { "fl: ${fl.power} != ${if (flReversed) -1.0 else 1.0} " }
        assert(br.power == if (brReversed) -1.0 else 1.0) { "br: ${br.power} != ${if (brReversed) -1.0 else 1.0} " }
        assert(bl.power == if (blReversed) -1.0 else 1.0) { "bl: ${bl.power} != ${if (blReversed) -1.0 else 1.0} " }
    }
}