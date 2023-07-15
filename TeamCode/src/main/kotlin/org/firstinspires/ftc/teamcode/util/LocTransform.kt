package org.firstinspires.ftc.teamcode.util

/**
 * Utility for converting positions between world space,
 * robot space, and assembly local space.
 */
object LocTransform {
    object Ground {
        /** A ground transform which has [center] (in global space) at it's local origin. */
        class Transform(private val center: Vec2Rot) {
            /** Global space to local space. */
            fun globalToLocalPos(worldPos: Vec2Rot) =
                    (worldPos - center).transformP { it.rotate(-center.r) }

            /** Local space to global space. */
            fun localToGlobalPos(localPos: Vec2Rot) =
                    localPos.transformP { it.rotate(center.r) } + center

            /** Global space to local space. */
            fun globalToLocalVel(worldVel: Vec2Rot) =
                    worldVel.transformP { it.rotate(-center.r) }

            /** Local space to global space. */
            fun localToGlobalVel(localVel: Vec2Rot) =
                    localVel.transformP { it.rotate(center.r) }
        }
    }

    object Space {
        // TODO
    }
}