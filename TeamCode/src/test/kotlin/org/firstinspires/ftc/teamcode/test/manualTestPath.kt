package org.firstinspires.ftc.teamcode.test

import org.firstinspires.ftc.teamcode.controlSystems.motionPath.buildPath
import org.firstinspires.ftc.teamcode.robot.arbot.FTCCenterStagePOIs
import org.firstinspires.ftc.teamcode.util.Alliance
import org.firstinspires.ftc.teamcode.util.Vec2Rot
import org.firstinspires.ftc.teamcode.vision.prop.TeamPropVisProcessor.Loc
import kotlin.math.roundToInt

private val POIs = FTCCenterStagePOIs

fun main() {

    val alliance = Alliance.Red
    val detectedLoc = Loc.Right

    val startingPos = POIs.startingPosition(alliance, 12.0, true)

    val (p,_) = buildPath(startingPos, Vec2Rot.zero) {

        // AUTO PATH TO TEST GOES HERE

    }

    println("t\tx\ty\tr")
    for (i in 0 until (p.validUntilT * 10).roundToInt()) {
        val t = i.toDouble() / 10
        val (v, r) = p.sampleClamped(t).pos
        val (x, y) = v
        println("$t\t$x\t$y\t$r")
    }
}