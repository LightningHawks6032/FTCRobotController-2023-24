package org.firstinspires.ftc.teamcode.hardware.motion

import org.firstinspires.ftc.robotcore.external.navigation.Quaternion
import org.firstinspires.ftc.teamcode.util.Vec2
import org.firstinspires.ftc.teamcode.util.Vec2Rot
import org.firstinspires.ftc.teamcode.util.Vec3
import org.firstinspires.ftc.teamcode.util.toVec3
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection
import kotlin.math.absoluteValue
import kotlin.math.atan2
import kotlin.math.cos
import kotlin.math.sin

class AprilTagOdometry(
        vararg cameras: CameraPlacement,
) : IOdometry() {
    private val cameras = cameras.map { placement -> TrackingCamera(placement) }

    private var updatedAnyCamera = false
    private var dtAccumulated = 0.0

    override fun tick(dt: Double) {
        dtAccumulated += dt
        if (!updatedAnyCamera) return
        updatedAnyCamera = false

        val lastPos = pos

        val estimates = cameras.flatMap { it.reportPerTagPositionEstimates() }

        // Position should be the weighted average of all the
        // estimated positions reported by each camera.
        pos = estimates.map { it.robotWorldPosition * it.confidence }.reduce(Vec2Rot::plus) /
                estimates.map { it.confidence }.reduce(Double::plus)

        val lastVel = vel
        vel = (pos - lastPos) / dtAccumulated
        acc = (vel - lastVel) / dtAccumulated
    }

    fun tickTrackedTags(camNum: Int, detections: List<AprilTagDetection>) {
        updatedAnyCamera = true
        cameras[camNum].trackedTags = detections.filter { it.ftcPose != null }
    }

    override fun hasPositionInfo() = cameras.any { it.hasPosition }

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

    class CameraPlacement(
            val robotSpacePos: Vec3,
            val camZRotOffY: Double,
    )

    private class TrackingCamera(
            val placement: CameraPlacement
    ) {
        var trackedTags: List<AprilTagDetection> = emptyList()

        val hasPosition = trackedTags.isNotEmpty()

        /** dark magic that approximates positions from AprilTag poses and location metadata */
        fun reportPerTagPositionEstimates(): List<TagPositionEstimate> =
                trackedTags.map { detection ->
                    // this code is dense with math I suggest you sit down for this one

                    // TODO: units (inches or mm idk lol)

                    if (detection.rawPose == null || detection.metadata == null) {
                        return@map null
                    }

                    // game plan
                    // we know
                    // - camera position relative to robot
                    // - tag position relative to camera
                    // keep transforming until we have robot relative to world space.
                    // use math only so we don't make any silly mistakes with visualization
                    // get transform from (robot to world), this is our robot position info.

                    // Note 0: quaternions rotate things that's all you need to know

                    //////// Note 1: How to perspective transformations //
                    // let v: vec
                    ///// (rotation: a to world)
                    // let r: quaternion
                    ///// (translation: a to world)
                    // let t: vec
                    ///// (a to world).applyTo(v)
                    // let v_world = r.applyTo(v) + t
                    ///// inverse(a to world).applyTo(v)
                    // let v_a = r.inverse().applyTo(v_world - t)
                    //////////////////////////////////////////////////////

                    //////// Note 2: How to quaternions //////////////////
                    // let a: quaternion
                    // let b: quaternion
                    ///// Notice! the order of multiplication is backwards!
                    // let a_then_b = b*a
                    //////////////////////////////////////////////////////

                    // Transform from (tag to camera) perspective
                    val tagPosCamSpace = detection.rawPose.let { p -> Vec3(p.x, p.y, p.z) }
                    // TODO: don't know which way `tagRotCamSpace` goes, may need to be reversed
                    val tagRotCamSpace = Quaternion.fromMatrix(detection.rawPose.R, 0)
                            .let {
                                if (org.firstinspires.ftc.teamcode.util.DebugVars.bool["invertTagRot"] == true) {
                                    it.inverse()
                                } else {
                                    it
                                }
                            }

                    // Transform from (tag to world) perspective
                    val tagPosWorldSpace = detection.metadata.fieldPosition.toVec3()
                    val tagRotWorldSpace = detection.metadata.fieldOrientation

                    // Transform from (cam to robot) perspective
                    val camPosRobotSpace = placement.robotSpacePos
                    val camRotRobotSpace = Quaternion(
                            // makes a pure z-axis rotation
                            cos(placement.camZRotOffY).toFloat(),
                            0f, // no idea how but it works and it's in the right direction
                            0f,
                            sin(placement.camZRotOffY).toFloat(),
                            0,
                    )

                    // want (cam to world)
                    // (cam to world) = (cam to tag) then (tag to world)
                    //                = inverse(tag to cam) then (tag to world)
                    // !! Remember quaternion multiplication order !! //
                    val camRotWorldSpace = tagRotWorldSpace
                            .multiply(tagRotCamSpace.inverse(), 0)

                    // position will be harder because we'll also need to consider rotation
                    // when transforming back.

                    // have (cam to tag)
                    //   = (rotate: cam to tag).applyTo(it) + (translate: cam to tag)
                    // get (tag to cam)
                    //   inverse(rotate: cam to tag).applyTo(it - (translate: cam to tag))
                    // now apply it to zero vec (origin point of camera)
                    val camPosTagSpace = (/*zero*/-tagPosCamSpace)
                            .rotateByQuaternion(tagRotCamSpace.inverse())

                    // have (tag to world)
                    //   = (rotate: tag to world).applyTo(it) + (translate: tag to world)
                    // now apply it to the previous result
                    val camPosWorldSpace = camPosTagSpace
                            .rotateByQuaternion(tagRotWorldSpace) + tagPosWorldSpace

                    // want (robot to world)
                    //   = (robot to cam) then (cam to world)
                    //   = inverse(cam to robot) then (cam to world)
                    val robotRotWorldSpace = camRotWorldSpace
                            .multiply(camRotRobotSpace.inverse(), 0)

                    // inverse(cam to robot)
                    //   = (rotate: cam to robot).inverse().applyTo(it - (translate: cam to robot))
                    // once again go from the zero position, now the center of the robot
                    // this will give the robot's position relative to the camera
                    val robotPosCameraSpace = (/*zero*/-camPosRobotSpace)
                            .rotateByQuaternion(camRotRobotSpace.inverse())
                    // (cam to world)
                    //   = (rotate: cam to world).applyTo(it) + (translate: cam to world)
                    val robotPosWorldSpace = robotPosCameraSpace
                            .rotateByQuaternion(camRotWorldSpace) + camPosWorldSpace

                    // We did it! Now we need to convert back to Vec2Rot, because the robot
                    // does not know what a "3d" is.

                    // take (robot to world) and extract z-axis rotation.
                    // Also keep track of how much rotation was not in the z direction
                    // so we can tell if our predictions are bad.
                    val (robotZRot, rotErrorAmount) = Vec3(1.0, 0.0, 0.0)
                            .rotateByQuaternion(robotRotWorldSpace)
                            .let { dir ->
                                Pair(
                                        atan2(dir.y, dir.x),
                                        dir.z,
                                )
                            }
                    // Extract x and y coordinates, ignoring z (no floating robots, please)
                    // Also keep track of z position is stored to tell
                    // if our predictions are bad.
                    val robotXYPos = robotPosWorldSpace.let { v -> Vec2(v.x, v.y) }
                    val posErrorAmount = robotPosWorldSpace.z

                    // Combine
                    val robotPos = Vec2Rot(robotXYPos, robotZRot)

                    // Crude confidence approximation
                    val confidence = 1.0 / (posErrorAmount.absoluteValue * 0.2
                            + rotErrorAmount.absoluteValue * 1.0
                            + 0.5)

                    TagPositionEstimate(confidence, robotPos)
                }.filterNotNull()
    }

    private class TagPositionEstimate(
            val confidence: Double,
            val robotWorldPosition: Vec2Rot,
    )
}