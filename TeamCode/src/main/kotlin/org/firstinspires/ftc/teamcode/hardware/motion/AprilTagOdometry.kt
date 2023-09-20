package org.firstinspires.ftc.teamcode.hardware.motion

import org.firstinspires.ftc.robotcore.external.matrices.VectorF
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference
import org.firstinspires.ftc.robotcore.external.navigation.Orientation
import org.firstinspires.ftc.teamcode.util.Vec2
import org.firstinspires.ftc.teamcode.util.Vec2Rot
import org.firstinspires.ftc.teamcode.util.Vec3
import org.firstinspires.ftc.teamcode.util.intoVec3
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection

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

        fun reportPerTagPositionEstimates(): List<TagPositionEstimate> =
                // dark magic
                trackedTags.map {
                    if (it.rawPose == null || it.metadata == null) {
                        return@map null
                    }
                    val tagRot = it.rawPose.R
                    val tagOrientation = Orientation.getOrientation(tagRot, AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.RADIANS)
                    val tagWorldRot = it.metadata.fieldOrientation.toMatrix()
                    val tagWorldOrientation = Orientation.getOrientation(tagRot, AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.RADIANS)
                    val tagWorldPos = it.metadata.fieldPosition.intoVec3()

                    // (tag_world_rot * inv(tag_rot))(-v) + tag_world_pos
                    val estimated3DCamPos = tagWorldRot.transform(
                            tagRot.inverted().transform(
                                    VectorF(
                                            -it.rawPose.x.toFloat(),
                                            -it.rawPose.y.toFloat(),
                                            -it.rawPose.z.toFloat(),
                                    )
                            )
                    ).intoVec3().rotateZ(placement.camZRotOffY) + tagWorldPos

                    val estimatedRobotZRot = (tagWorldOrientation.thirdAngle
                            - tagOrientation.thirdAngle
                            - placement.camZRotOffY)

                    val cameraPlacementOffset = -(placement.robotSpacePos.rotateZ(estimatedRobotZRot))

                    val estimated3DRobotPos = estimated3DCamPos - cameraPlacementOffset

                    println("#${it.id} cp(${estimated3DCamPos.let { c-> "${c.x}, ${c.y}, ${c.z}" }}) rp(${estimated3DRobotPos.let { c-> "${c.x}, ${c.y}, ${c.z}" }}; r=${estimatedRobotZRot})")

                    TagPositionEstimate(
                            0.0,
                            Vec2Rot(Vec2(estimated3DRobotPos.x, estimated3DRobotPos.y), estimatedRobotZRot),
                    )
                }.filterNotNull()
    }

    private class TagPositionEstimate(
            val confidence: Double,
            val robotWorldPosition: Vec2Rot,
    )
}