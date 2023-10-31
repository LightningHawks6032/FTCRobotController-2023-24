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

        var minConfidence: Double = -1.0
            get() = field
            set(value) {
                ftcProcessor.setMinResultConfidence(value.toFloat())
                field = value
            }


        init {
            minConfidence = 0.75
        }


        /** Gets a list containing recognitions that were detected
         * since the last call to this method, or null if no new
         * recognitions are available. This is useful to avoid
         * re-processing the same recognitions multiple times. */
        val freshRecognitions: List<Recognition>?
            get() = ftcProcessor.recognitions
//        get() = ftcProcessor.freshRecognitions
    }
}