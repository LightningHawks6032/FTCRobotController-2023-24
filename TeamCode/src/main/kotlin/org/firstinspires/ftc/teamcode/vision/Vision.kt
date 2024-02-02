package org.firstinspires.ftc.teamcode.vision

import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraName
import org.firstinspires.ftc.teamcode.LOpMode
import org.firstinspires.ftc.teamcode.ftcGlue.IHardwareMap
import org.firstinspires.ftc.vision.VisionPortal

class Vision(val webcamId: String) {


    inner class Impl(hardwareMap: IHardwareMap) {
        private val camera = hardwareMap.getRaw(webcamId, CameraName::class) ?: TODO("camera '$webcamId' not found")

        /**
         * Creates a VisionPortal using the builder.
         */
        fun create(vararg processors: VisProcessor.VisionImpl, build: VisionPortal.Builder.() -> Unit = {}): VisionPortal {
            val builder = VisionPortal.Builder()
            builder.setCamera(camera)
            build(builder)
            builder.addProcessors(*processors.map { it.ftcProcessor }.toTypedArray())
            return builder.build()
        }
    }
}

/**
 * Combined function for activating vision and calling [LOpMode.RunScope.createLoop]
 * to poll data. Automatically closes the camera stream when the loop terminates.
 */
suspend fun <T : Any> LOpMode<T>.RunScope.createVisionLoop(
        vision: Vision.Impl,
        processors: List<VisProcessor.VisionImpl>,
        build: VisionPortal.Builder.() -> Unit = {},
        /** Loop runs while condition is true. */
        condition: () -> Boolean = { duringRun },
        /** Code to be asynchronously looped. */
        block: LOpMode.LoopScope<T>.(visionPortal: VisionPortal) -> Unit,
) {
    val visionPortal = vision.create(*processors.toTypedArray()) { build.invoke(this) }
    createLoop(condition, blockDestructor = {
        visionPortal.close()
        processors.forEach { it.cleanup() }
    }) {
        block(this@createLoop, visionPortal)
    }
}