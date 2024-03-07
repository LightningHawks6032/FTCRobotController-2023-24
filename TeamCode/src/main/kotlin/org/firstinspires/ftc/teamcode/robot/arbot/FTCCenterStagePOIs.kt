package org.firstinspires.ftc.teamcode.robot.arbot

import org.firstinspires.ftc.teamcode.layout.POIAARect
import org.firstinspires.ftc.teamcode.layout.POILine
import org.firstinspires.ftc.teamcode.util.Alliance
import org.firstinspires.ftc.teamcode.util.Vec2
import org.firstinspires.ftc.teamcode.util.Vec2Rot
import kotlin.math.PI

private const val INCH = 1.0 // base unit
private const val FEET = 12.0 * INCH

object FTCCenterStagePOIs {
    val fieldBounds = POIAARect(-6.0 to 6.0, -6.0 to 6.0) * FEET

    private val trussXSafeBounds = (-2.0 * FEET - 9 * INCH) to (0.0 * FEET + 9 * INCH)
    private val outerTrussSafePaths = listOf(
            // ^  blue alliance
            POILine.xLine(trussXSafeBounds, 5 * FEET),
            POILine.xLine(trussXSafeBounds, 3 * FEET),
            POILine.xLine(trussXSafeBounds, -3 * FEET),
            POILine.xLine(trussXSafeBounds, -5 * FEET),
            // v  red alliance
    )

    fun nearestTrussSafePath(point: Vec2) = outerTrussSafePaths[
            if (point.y > 4 * FEET) {
                0
            } else if (point.y > 0) {
                1
            } else if (point.y > -4 * FEET) {
                2
            } else {
                3
            }
    ]

    private val backdropRed = Vec2(5.333 * FEET, -3 * FEET)
    private val backdropBlue = Vec2(5.333 * FEET, 3 * FEET)
    private const val backdropWidth = 1 * FEET

    fun facingBackdropFromDistance(alliance: Alliance, distance: Double, strafe: Double = 0.0): Vec2 {
        val center = when (alliance) {
            Alliance.Red -> backdropRed
            Alliance.Blue -> backdropBlue
        }
        return center + Vec2(-distance, strafe.coerceIn(-backdropWidth / 2, backdropWidth / 2))
    }


    enum class Facing {
        Opponent,
        Self,
        Red,
        Blue,

        /** Facing in the direction such that Red is on the right. */
        Forwards,

        /** Facing in the direction such that Red is on the left. */
        Backwards;

        fun angle(alliance: Alliance) = when (this) {
            Opponent -> when (alliance) {
                Alliance.Red -> PI / 2 // facing Blue
                Alliance.Blue -> -PI / 2 // facing Red
            }
            Self -> when (alliance) {
                Alliance.Red -> -PI / 2 // facing Red
                Alliance.Blue -> PI / 2 // facing Blue
            }
            Red -> -PI / 2
            Blue -> PI / 2
            Forwards -> 0.0
            Backwards -> PI
        }
    }

    fun startingPosition(alliance: Alliance, x: Double, faceInwards: Boolean, robotRadius: Double = 9.0): Vec2Rot =
            when (alliance) {
                Alliance.Red -> Vec2Rot(
                        x, -6.0 * FEET + robotRadius,
                        if (faceInwards) PI / 2 else 0.0,
                )
                Alliance.Blue -> Vec2Rot(
                        x, 6.0 * FEET - robotRadius,
                        if (faceInwards) -PI / 2 else 0.0,
                )
            }

    fun relativePos(alliance: Alliance, x: Double, distanceFromHomeWall: Double): Vec2 =
            when (alliance) {
                Alliance.Red -> Vec2(x, fieldBounds.y.first + distanceFromHomeWall)
                Alliance.Blue -> Vec2(x, fieldBounds.y.second - distanceFromHomeWall)
            }
    fun relativeVel(alliance: Alliance, x: Double, velocityFromHomeWall: Double): Vec2 =
            when (alliance) {
                Alliance.Red -> Vec2(x, + velocityFromHomeWall)
                Alliance.Blue -> Vec2(x, - velocityFromHomeWall)
            }

    fun coerceOutOfDangerZone(pos: Vec2Rot, vel: Vec2Rot): Vec2Rot {
        val xSoon = pos.v.x + vel.v.x * 0.5
        if (xSoon + 2.0 < trussXSafeBounds.first || xSoon - 2.0 > trussXSafeBounds.second) {
            return pos
        }

        val nearestSafeTrussPass = nearestTrussSafePath(pos.v)
        val safeR = pos.r - (pos.r + PI/4).mod(PI/2) + PI/4
        val safeXY = nearestSafeTrussPass.project(pos.v)
        return Vec2Rot(safeXY, safeR)
    }
}