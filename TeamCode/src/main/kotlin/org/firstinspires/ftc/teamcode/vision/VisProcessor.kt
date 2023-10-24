package org.firstinspires.ftc.teamcode.vision

abstract class VisProcessor<T : VisProcessor.VisionImpl> {
    abstract class VisionImpl {
        internal abstract val ftcProcessor: org.firstinspires.ftc.vision.VisionProcessor
        abstract fun cleanup()
    }
}