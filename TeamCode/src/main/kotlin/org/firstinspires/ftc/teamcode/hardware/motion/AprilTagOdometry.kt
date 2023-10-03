package org.firstinspires.ftc.teamcode.hardware.motion

import org.firstinspires.ftc.robotcore.external.navigation.Quaternion
import org.firstinspires.ftc.teamcode.util.*
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection
import kotlin.math.absoluteValue
import kotlin.math.atan2

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


        val estimates = cameras.flatMap { it.reportPerTagPositionEstimates() }
        if (estimates.isEmpty()) return

        val lastPos = pos
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

        fun predictCamLocation(detection: AprilTagDetection): CamLocationEstimate {
            // Transform from (tag to camera) perspective
            val posePos = detection.rawPose.let { p -> Vec3(p.x, p.y, p.z) }
            val poseRot = Quaternion.fromMatrix(detection.rawPose.R, 0).inverse()
//            println("\n" + ("""
//                        ### CSV Regression thing Row ###
//                        pose_pos_x, pose_pos_y, pose_pos_z, pose_r_w, pose_r_x, pose_r_y, pose_r_z, gt_x, gt_y, gt_z, gt_r
//                        ${posePos.let { "${it.x}, ${it.y}, ${it.z}" }}, ${poseRot.let { "${it.w}, ${it.x}, ${it.y}, ${it.z}" }}, ####, ####, ####, ####
//                        ##################################
//                    """.trimIndent()))

            val rotated = poseRot.apply(posePos)
            val camPosTagSpace = rotated.let { Vec3(-it.z, it.x, -it.y) } * 0.85
            val camHdgTagSpace = poseRot.apply(Vec3(1.0, 0.0, 0.0)).let {
                atan2(it.z, it.x)
            }
            return CamLocationEstimate(camPosTagSpace, camHdgTagSpace)
        }

        /** dark magic that approximates positions from AprilTag poses and location metadata */
        fun reportPerTagPositionEstimates(): List<BotPositionEstimate> =
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

                    // get (cam to tag)
                    val camLoc = predictCamLocation(detection)

                    // Transform from (tag to world) perspective
                    val tagPosWorldSpace = detection.metadata.fieldPosition.toVec3()
                    val tagRotWorldSpace = detection.metadata.fieldOrientation
                    val tagHdgWorldSpace = Vec3(1.0,0.0,0.0).rotateByQuaternion(tagRotWorldSpace).let { atan2(it.y, it.x) }

                    // have (tag to world)
                    //   = (rotate: tag to world).applyTo(it) + (translate: tag to world)
                    // now apply it to the previous result
                    val camPosWorldSpace = tagRotWorldSpace.apply(camLoc.camPosTagSpace) + tagPosWorldSpace
                    // (world to tag) then (tag to cam)
                    val camHdgWorldSpace = camLoc.camHdgTagSpace + tagHdgWorldSpace

                    // inverse(robot to cam) = (cam to robot)
                    //   = (rotate: robot to cam).inverse().applyTo(it - (translate: robot to cam))
                    // go from the zero position, now the center of the robot
                    // this will give the robot's position relative to the camera
                    val robotPosCameraSpace = (/*zero*/-placement.robotSpacePos)
                            .rotateZ(-placement.camZRotOffY)
                    // (world to robot) = (world to cam) then (cam to robot)
                    val robotPosWorldSpace = robotPosCameraSpace
                            .rotateZ(camHdgWorldSpace - placement.camZRotOffY) +
                            camPosWorldSpace
                    val robotHdgWorldSpace = camHdgWorldSpace - placement.camZRotOffY

                    // We did it! Now we need to convert back to Vec2Rot, because the robot
                    // does not know what a "3d" is.

                    // Extract x and y coordinates, ignoring z (no floating robots, please)
                    val robotXYPos = robotPosWorldSpace.let { v -> Vec2(v.x, v.y) }

                    // Combine
                    val robotPos = Vec2Rot(robotXYPos, robotHdgWorldSpace)

                    // Crude confidence approximation, based on if the robot thinks it's
                    // either floating or embedded in the ground.
                    val confidence = 1.0 / (robotPosWorldSpace.z.absoluteValue * 0.2 + 0.5)

                    BotPositionEstimate(confidence, robotPos)
                }.filterNotNull()
    }

    private class BotPositionEstimate(
            val confidence: Double,
            val robotWorldPosition: Vec2Rot,
    )

    /** Represents the predicted location of a camera relative
     * to an AprilTag in inches. */
    private class CamLocationEstimate(
            val camPosTagSpace: Vec3,
            val camHdgTagSpace: Double,
    )
}