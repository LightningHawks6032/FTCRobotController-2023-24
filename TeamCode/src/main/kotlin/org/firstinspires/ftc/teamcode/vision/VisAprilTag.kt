package org.firstinspires.ftc.teamcode.vision

import org.firstinspires.ftc.robotcore.external.matrices.VectorF
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit
import org.firstinspires.ftc.robotcore.external.navigation.Quaternion
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection
import org.firstinspires.ftc.vision.apriltag.AprilTagLibrary
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor.PoseSolver

class VisAprilTag(
        /** Information about sizing and positioning of tags in the real world. */
        val tagInfo: AprilTagLibrary,
        /** Whether or not to draw boxes, axes, and ids on the tags in the camera stream. */
        val drawAnnotations: Boolean = true,
        /** Lens information. Defaults set by FTC for most cameras, so you shouldn't have to use. */
        val manualLensIntrinsics: LensIntrinsics? = null,
) : VisProcessor<VisAprilTag.Instance>() {

    /**
     * Information about the lens (now in a data class). The library auto-fills these values.
     *
     * I have zero idea how you get these numbers, hopefully you wont need to used this.
     */
    data class LensIntrinsics(val fx: Double, val fy: Double, val cx: Double, val cy: Double)

    override fun new() = Instance()
    inner class Instance : VisProcessor.Instance() {
        override val ftcProcessor = with(org.firstinspires.ftc.vision.apriltag.AprilTagProcessor.Builder()) {
            manualLensIntrinsics?.let {
                setLensIntrinsics(it.fx, it.fy, it.cx, it.cy)
            }

            setDrawTagID(drawAnnotations)
            setDrawTagOutline(drawAnnotations)
            setDrawAxes(drawAnnotations)
            setDrawCubeProjection(drawAnnotations)

            setTagLibrary(tagInfo)

            build()
        }

        /**
         * FTCRobotController docs:
         *
         * > Get a list containing detections that were detected SINCE THE PREVIOUS CALL
         * to this method, or NULL if no new detections are available. This is useful
         * to avoid re-processing the same detections multiple times.
         */
        val freshDetections: List<AprilTagDetection>?
            get() = ftcProcessor.freshDetections?.toList()

        /**
         * FTCRobotController docs:
         *
         * > Get the average time in milliseconds the currently set pose
         * solver is taking to converge on a solution **per tag**. Some pose
         * solvers are much more expensive than others.
         */
        val perTagAvgPoseSolveTime: Int
            get() = ftcProcessor.perTagAvgPoseSolveTime

        var decimation: Double = 1.0
            set(value) {
                if (field == value) return
                field = value
                ftcProcessor.setDecimation(value.toFloat())
            }

        /**
         * FTCRobotController docs:
         *
         * > Specify the method used to calculate 6DOF pose from the tag
         * corner positions once found by the AprilTag algorithm.
         */
        fun setPoseSolver(solver: PoseSolver) {
            ftcProcessor.setPoseSolver(solver)
        }

        init {
            // Make sure we agree on initial value for decimation
            ftcProcessor.setDecimation(decimation.toFloat())

            setPoseSolver(solver = PoseSolver.OPENCV_ITERATIVE)
        }
    }

    companion object {
        fun specifyTagInfo(builder: TagInfoBuilder.()->Unit) = AprilTagLibrary.Builder().also {
            builder(TagInfoBuilder(it))
        }.build()
    }

    class TagInfoBuilder(private val ftc: AprilTagLibrary.Builder) {
        fun addTag(id: Int, name: String, sizeInches: Double) {
            ftc.addTag(id, name, sizeInches, DistanceUnit.INCH)
        }
        fun addTag(id: Int, name: String, sizeInches: Double, locationInches: VectorF, worldNegXToTagFwdRot: Quaternion) {
            ftc.addTag(id, name, sizeInches, locationInches, DistanceUnit.INCH, worldNegXToTagFwdRot)
        }
        fun addTag(tag: TagInfo) {
            ftc.addTag(tag.id, tag.name, tag.sizeInches, DistanceUnit.INCH)
        }
        fun addTags(lib: AprilTagLibrary) {
            ftc.addTags(lib)
        }
        fun addTagsCenterStage() {
            // Need to create them manually because the official FTCRobotController repo typo'd it.
            addTag(1, "BlueAllianceLeft",
                            2.0, VectorF(60.25f, 41.41f, 4f),
                            Quaternion(0.683f, -0.183f, 0.183f, 0.683f, 0))
            addTag(2, "BlueAllianceCenter",
                            2.0, VectorF(60.25f, 35.41f, 4f),
                            Quaternion(0.683f, -0.183f, 0.183f, 0.683f, 0))
            addTag(3, "BlueAllianceRight",
                            2.0, VectorF(60.25f, 29.41f, 4f),
                            Quaternion(0.683f, -0.183f, 0.183f, 0.683f, 0))
            addTag(4, "RedAllianceLeft",
                            2.0, VectorF(60.25f, -29.41f, 4f),
                            Quaternion(0.683f, -0.183f, 0.183f, 0.683f, 0))
            addTag(5, "RedAllianceCenter",
                            2.0, VectorF(60.25f, -35.41f, 4f),
                            Quaternion(0.683f, -0.183f, 0.183f, 0.683f, 0))
            addTag(6, "RedAllianceRight",
                            2.0, VectorF(60.25f, -41.41f, 4f),
                            Quaternion(0.683f, -0.183f, 0.183f, 0.683f, 0))
            addTag(7, "RedAudienceWallLarge",
                            5.0, VectorF(-70.25f, -40.625f, 5.5f),
                            Quaternion(0.7071f, 0f, 0f, -0.7071f, 0))
            addTag(8, "RedAudienceWallSmall",
                            2.0, VectorF(-70.25f, -35.125f, 4f),
                            Quaternion(0.7071f, 0f, 0f, -0.7071f, 0))
            addTag(9, "BlueAudienceWallSmall",
                            2.0, VectorF(-70.25f, 35.125f, 4f),
                            Quaternion(0.7071f, 0f, 0f, -0.7071f, 0))
            addTag(10, "BlueAudienceWallLarge",
                            5.0, VectorF(-70.25f, 40.625f, 5.5f),
                            Quaternion(0.7071f, 0f, 0f, -0.7071f, 0))
        }

        operator fun TagInfo.unaryPlus() {
            addTag(this)
        }
        operator fun AprilTagLibrary.unaryPlus() {
            addTags(this)
        }
    }
    data class TagInfo(val id: Int, val name: String, val sizeInches: Double)
}