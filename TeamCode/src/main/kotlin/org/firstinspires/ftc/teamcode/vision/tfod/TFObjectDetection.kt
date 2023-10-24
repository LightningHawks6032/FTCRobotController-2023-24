package org.firstinspires.ftc.teamcode.vision.tfod

import org.firstinspires.ftc.robotcore.external.tfod.Recognition
import org.firstinspires.ftc.teamcode.vision.VisProcessor
import org.firstinspires.ftc.vision.tfod.TfodProcessor

class TFObjectDetection(
        val customModelFileName: String? = null,
) : VisProcessor<TFObjectDetection.VisionImpl>() {
    inner class VisionImpl : VisProcessor.VisionImpl() {
        override val ftcProcessor: TfodProcessor = with(TfodProcessor.Builder()) {
            if (customModelFileName != null) {
                setModelFileName(customModelFileName)
            }
            build()
        }

        override fun cleanup() {
            ftcProcessor.shutdown()
        }

        /** Gets a list containing recognitions that were detected
         * since the last call to this method, or null if no new
         * recognitions are available. This is useful to avoid
         * re-processing the same recognitions multiple times. */
        val freshRecognitions: List<Recognition>?
            get() = ftcProcessor.freshRecognitions
    }
}