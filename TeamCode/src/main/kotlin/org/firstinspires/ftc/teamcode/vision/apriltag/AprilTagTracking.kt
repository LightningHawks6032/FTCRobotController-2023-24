package org.firstinspires.ftc.teamcode.vision.apriltag

import org.firstinspires.ftc.robotcore.external.navigation.Quaternion
import org.firstinspires.ftc.teamcode.hardware.motion.IOdometry
import org.firstinspires.ftc.teamcode.util.*
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection
import kotlin.math.absoluteValue
import kotlin.math.atan2
import kotlin.math.cos
import kotlin.math.sin

class AprilTagTracking(
        tagInfo: AprilTagInfoBuilder,
        vararg cameras: CameraPlacement,
) {
    private val tagUsages = tagInfo.buildUsages()

    private val cameras = cameras.map { placement -> TrackingCamera(placement) }


    private var updatedAnyCameraN = 0
    private fun markHasUpdatedSomeCamera() {
        updatedAnyCameraN += 1
    }

    inner class Odometry : IOdometry() {
        private var dtAccumulated = 0.0
        private var odoUpdatedN = 0

        override fun tick(dt: Double) {
            dtAccumulated += dt
            if (odoUpdatedN != updatedAnyCameraN) return
            odoUpdatedN = updatedAnyCameraN


            val estimates = cameras.flatMap { it.botPosEstimates }
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
    }


    fun updateEstimates(
            camNum: Int,
            detections: List<AprilTagDetection>,
            acceptedRobotPos: Vec2Rot,
    ) = cameras[camNum].updateEstimates(
            detections,
            acceptedRobotPos,
    )


    class CameraPlacement(
            val robotSpacePos: Vec3,
            val camZRotOffX: Double,
    )

    inner class TrackingCamera(
            val placement: CameraPlacement
    ) {
        internal var botPosEstimates: List<BotPositionEstimate> = emptyList()
            private set
        internal var tagPosEstimates: List<TagPositionEstimate> = emptyList()
            private set

        val hasPosition = botPosEstimates.isNotEmpty()

        private fun predictTFCamToTag(detection: AprilTagDetection): Transform3D {
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

            val camRotTagSpace = poseRot.let { Quaternion(it.w, it.z, -it.x, it.y, 0) }
            return Transform3D(camPosTagSpace, camRotTagSpace)
        }

        /** dark magic that approximates positions from AprilTag poses and location metadata */
        fun updateEstimates(detections: List<AprilTagDetection>, acceptedRobotPos: Vec2Rot) {
            // Predict the positions we care about by representing each relative position we know
            // (tag relative to camera, camera relative to robot, etc.), and represent them as
            // transformations to turn a position relative to one into a position relative
            // to the other.
            //
            // This representation is helpful, as it allows us to chain changes of perspective
            // without much sweat (see Transform3D for details).
            //
            // TODO: units (inches or mm idk lol, probably inches tho)
            //

            val camPosRobotSpace = placement.robotSpacePos
            val camRotRobotSpace = Quaternion(
                    // z-rotation in quaternion
                    cos(placement.camZRotOffX).toFloat(),
                    0.0f,
                    0.0f,
                    sin(placement.camZRotOffX).toFloat(),
                    0,
            )
            val cam2robot = Transform3D(camPosRobotSpace, camRotRobotSpace)

            val robot2worldPrev = Transform3D(
                    acceptedRobotPos.v.let { Vec3(it.x,it.y,0.0) },
                    Quaternion(
                            // z-rotation in quaternion
                            cos(acceptedRobotPos.r).toFloat(),
                            0.0f,
                            0.0f,
                            sin(acceptedRobotPos.r).toFloat(),
                            0,
                    )
            )

            val allEstimates = detections.mapNotNull { detection ->
                if (detection.rawPose == null || detection.metadata == null) {
                    return@mapNotNull null
                }

                val cam2tag = predictTFCamToTag(detection)

                when (tagUsages[detection.id]) {
                    null -> null
                    AprilTagUsage.RobotPosition -> {
                        val tagPosWorldSpace = detection.metadata.fieldPosition.toVec3()
                        val tagRotWorldSpace = detection.metadata.fieldOrientation
                        val tag2world = Transform3D(tagPosWorldSpace, tagRotWorldSpace)

                        // transformation chain: robot to cam to tag to world
                        val robot2world = cam2robot.inverse() then cam2tag then tag2world

                        // We did it! Now we need to convert back to Vec2Rot, because the robot
                        // does not know what a "3d" is.

                        // Extract coordinates from the resulting Transform3D and combine
                        val robotPosXY = robot2world.offset.let { Vec2(it.x, it.y) }
                        val robotPosZ = robot2world.offset.z
                        val (robotHdg, robotHdgZErr) = Vec3(1.0, 0.0, 0.0).rotateByQuaternion(robot2world.rotation).let {
                            Pair(atan2(it.y, it.x), it.z)
                        }
                        val robotPos = Vec2Rot(robotPosXY, robotHdg)

                        // Crude confidence approximation, based on if the robot thinks it's
                        // either floating or embedded in the ground.
                        val confidence = 1.0 / (robotPosZ.absoluteValue * 0.2 + robotHdgZErr.absoluteValue * 0.2 + 0.5)

                        BotPositionEstimate(confidence, robotPos)
                    }
                    AprilTagUsage.TagPosition -> {
                        // transformation chain: tag to camera to robot to world
                        val tag2world = cam2tag.inverse() then cam2robot then robot2worldPrev

                        // This is easy because we want to report these positions in 3d space anyway.
                        TagPositionEstimate(tag2world.offset, tag2world.rotation)
                    }
                }
            }

            botPosEstimates = allEstimates.filterIsInstance<BotPositionEstimate>()
            tagPosEstimates = allEstimates.filterIsInstance<TagPositionEstimate>()

            markHasUpdatedSomeCamera()
        }
    }

    internal class BotPositionEstimate(
            val confidence: Double,
            val robotWorldPosition: Vec2Rot,
    )

    class TagPositionEstimate(
            val pos: Vec3,
            val rot: Quaternion,
    )
}