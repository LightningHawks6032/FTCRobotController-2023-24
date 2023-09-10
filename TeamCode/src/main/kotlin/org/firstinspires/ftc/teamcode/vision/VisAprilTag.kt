package org.firstinspires.ftc.teamcode.vision

import org.firstinspires.ftc.vision.apriltag.AprilTagDetection
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor.PoseSolver

class VisAprilTag(
        val manualLensIntrinsics: LensIntrinsics? = null,
) : VisProcessor<VisAprilTag.Instance>() {

    class LensIntrinsics(val fx: Double, val fy: Double, val cx: Double, val cy: Double)

    override fun new() = Instance()
    inner class Instance : VisProcessor.Instance() {
        override val ftcProcessor = with(org.firstinspires.ftc.vision.apriltag.AprilTagProcessor.Builder()) {
            manualLensIntrinsics?.let {
                setLensIntrinsics(it.fx, it.fy, it.cx, it.cy)
            }

            setDrawTagID(true)
            setDrawTagOutline(true)
            setDrawAxes(true)
            setDrawCubeProjection(true)

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
        fun setPoseSolver(solver: PoseSolver) =
                ftcProcessor.setPoseSolver(solver)

        init {
            // Make sure we agree on initial value for decimation
            ftcProcessor.setDecimation(decimation.toFloat())
        }
    }
}