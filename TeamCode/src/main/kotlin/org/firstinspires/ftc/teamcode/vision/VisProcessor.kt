package org.firstinspires.ftc.teamcode.vision

abstract class VisProcessor<T : VisProcessor.Instance> {
    abstract class Instance {
        internal abstract val ftcProcessor: org.firstinspires.ftc.vision.VisionProcessor
    }

    abstract fun new(): T
}