package org.firstinspires.ftc.teamcode.test.apriltag

import org.firstinspires.ftc.teamcode.util.Quaternion
import org.firstinspires.ftc.teamcode.util.Transform3D
import org.firstinspires.ftc.teamcode.util.Vec3
import org.firstinspires.ftc.teamcode.vision.apriltag.AprilTagInfoBuilder
import org.firstinspires.ftc.teamcode.vision.apriltag.AprilTagTracking
import org.junit.Test
import kotlin.reflect.full.declaredFunctions
import kotlin.reflect.jvm.isAccessible

class PredictCamOffsetTest {
    val cam = AprilTagTracking(AprilTagInfoBuilder {})
            .TrackingCamera(AprilTagTracking.CameraPlacement(Vec3(0.0,0.0,0.0), 0.0))
    fun predictTFCamToTagPure(posePos: Vec3, poseRot: Quaternion): Transform3D {
        val fn = AprilTagTracking.TrackingCamera::class.declaredFunctions.find { it.name == "predictTFCamToTagPure" }
                ?: throw Error("Couldn't find function predictTFCamToTagPure")
        fn.isAccessible = true
        return fn.call(cam, posePos, poseRot) as Transform3D
    }
    fun predictTFTagToCamPure(posePos: Vec3, poseRot: Quaternion): Transform3D {
        val fn = AprilTagTracking.TrackingCamera::class.declaredFunctions.find { it.name == "predictTFTagToCamPure" }
                ?: throw Error("Couldn't find function predictTFTagToCamPure")
        fn.isAccessible = true
        return fn.call(cam, posePos, poseRot) as Transform3D
    }

    @Test
    fun jofdjdslk() {
        /** ((measured_pos, measured_rot), (output_pos, output_rot)) */
        val data = arrayOf(
                Pair(
                        Pair(Vec3(0.24893522262573242, 0.7957226634025574, 13.965217590332031), Quaternion(0.99766254, -0.066111565, 0.010482159, 0.013740392)),
                        Pair(Vec3(-11.8575192 ,  -0.04076538,   0.89835603), Quaternion(0.99766256, 0.01374039,  0.06611157,  0.01048216)),
                ),
                Pair(
                        Pair(Vec3(0.7144445180892944, 0.14634402096271515, 14.119644165039062), Quaternion(0.94588923, -0.07440801, 0.31050897, -0.057803024)),
                        Pair(Vec3(-9.92948652,  -6.47996155,   1.95969373), Quaternion(0.94588923, -0.05780302,  0.07440801,  0.31050897)),
                ),
                Pair(
                        Pair(Vec3(-9.631043434143066, 3.504793405532837, 30.20060920715332), Quaternion(0.9365898, -0.044289347, -0.34753338, 0.007655327)),
                        Pair(Vec3(-24.93499328,  10.62039935,  -0.56615232), Quaternion(0.93658977, 0.00765533,  0.04428935, -0.34753337)),
                ),
        )

        for ((input, output) in data) {
            val (in_pos, in_rot) = input
            val (out_pos, out_rot) = output
            val (pos, rot) = predictTFCamToTagPure(in_pos, in_rot).local2outerToLocation()
            assert((out_pos - pos).mag < 0.1) { "Predictions match the python notebook predictions. $out_pos != $pos"}
            assert((out_rot + rot * -1.0).magnitude < 0.1) { "Predictions match the python notebook predictions. $out_rot != $rot"}
        }


    }
}