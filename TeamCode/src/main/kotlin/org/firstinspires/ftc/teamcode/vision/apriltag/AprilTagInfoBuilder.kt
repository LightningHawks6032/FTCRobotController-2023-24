package org.firstinspires.ftc.teamcode.vision.apriltag

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit
import org.firstinspires.ftc.teamcode.util.Quaternion
import org.firstinspires.ftc.teamcode.util.Vec3
import org.firstinspires.ftc.teamcode.util.toFTC
import org.firstinspires.ftc.teamcode.util.toVectorF
import org.firstinspires.ftc.vision.apriltag.AprilTagLibrary

class AprilTagInfoBuilder(
        private val libBuilder: AprilTagLibrary.Builder,
        private val tagUsages: MutableMap<Int, AprilTagUsage>
) {
    fun buildTagLibrary() = libBuilder.build()
    fun buildUsages() = tagUsages.toMap()

    fun addForTagPos(id: Int, name: String, sizeInches: Double) {
        libBuilder.addTag(id, name, sizeInches, DistanceUnit.INCH)
        tagUsages[id] = AprilTagUsage.TagPosition
    }

    fun addForRobotPos(id: Int, name: String, sizeInches: Double, locationInches: Vec3, worldNegXToTagFwdRot: Quaternion) {
        libBuilder.addTag(id, name, sizeInches, locationInches.toVectorF(), DistanceUnit.INCH, worldNegXToTagFwdRot.toFTC())
        tagUsages[id] = AprilTagUsage.RobotPosition
    }

    fun addLibForRobotPos(lib: AprilTagLibrary) {
        this.libBuilder.addTags(lib)
        for (tagMeta in lib.allTags) {
            tagUsages[tagMeta.id] = AprilTagUsage.RobotPosition
        }
    }

    fun addTagsCenterStage() {
        // Need to create them manually because the official FTCRobotController repo typo'd it.
        addForRobotPos(1, "BlueAllianceLeft",
                2.0, Vec3(60.25, 41.41, 4.0),
                Quaternion(0.683, -0.183, 0.183, 0.683))
        addForRobotPos(2, "BlueAllianceCenter",
                2.0, Vec3(60.25, 35.41, 4.0),
                Quaternion(0.683, -0.183, 0.183, 0.683))
        addForRobotPos(3, "BlueAllianceRight",
                2.0, Vec3(60.25, 29.41, 4.0),
                Quaternion(0.683, -0.183, 0.183, 0.683))
        addForRobotPos(4, "RedAllianceLeft",
                2.0, Vec3(60.25, -29.41, 4.0),
                Quaternion(0.683, -0.183, 0.183, 0.683))
        addForRobotPos(5, "RedAllianceCenter",
                2.0, Vec3(60.25, -35.41, 4.0),
                Quaternion(0.683, -0.183, 0.183, 0.683))
        addForRobotPos(6, "RedAllianceRight",
                2.0, Vec3(60.25, -41.41, 4.0),
                Quaternion(0.683, -0.183, 0.183, 0.683))
        addForRobotPos(7, "RedAudienceWallLarge",
                5.0, Vec3(-70.25, -40.625, 5.5),
                Quaternion(0.7071, 0.0, 0.0, -0.7071))
        addForRobotPos(8, "RedAudienceWallSmall",
                2.0, Vec3(-70.25, -35.125, 4.0),
                Quaternion(0.7071, 0.0, 0.0, -0.7071))
        addForRobotPos(9, "BlueAudienceWallSmall",
                2.0, Vec3(-70.25, 35.125, 4.0),
                Quaternion(0.7071, 0.0, 0.0, -0.7071))
        addForRobotPos(10, "BlueAudienceWallLarge",
                5.0, Vec3(-70.25, 40.625, 5.5),
                Quaternion(0.7071, 0.0, 0.0, -0.7071))
    }

    fun build(): Pair<AprilTagLibrary, Map<Int, AprilTagUsage>> = Pair(libBuilder.build(), tagUsages)

    companion object {
        operator fun invoke(builderFn: AprilTagInfoBuilder.() -> Unit) =
                AprilTagInfoBuilder(AprilTagLibrary.Builder(), mutableMapOf()).also(builderFn)
    }
}