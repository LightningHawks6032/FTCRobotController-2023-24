package org.firstinspires.ftc.teamcode.vision.prop

import android.graphics.Canvas
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration
import org.firstinspires.ftc.teamcode.util.Alliance
import org.firstinspires.ftc.teamcode.vision.SimpleLinePaint
import org.firstinspires.ftc.teamcode.vision.VisProcessor
import org.firstinspires.ftc.vision.VisionProcessor
import org.opencv.core.*
import org.opencv.imgproc.Imgproc

class TeamPropVisProcessor(
        alliance: Alliance,
) : VisionProcessor, VisProcessor.VisionImpl() {
    override val ftcProcessor get() = this

    /** The range in which the color of a pixel must lie in order to be considered part of the Team Prop. */
    private val detectColorRange = when (alliance) {
        // Constants tuned by 5436 Aluminum Cobblers
        Alliance.Red -> DetectColorRange(
                centerHue = 10.0,
                hueThreshold = 20.0,
                minSaturation = 70.0,
                minValue = 70.0,
        )
        Alliance.Blue -> DetectColorRange(
                centerHue = 215.0,
                hueThreshold = 120.0,
                minSaturation = 30.0,
                minValue = 30.0,
        )
    }

    /** A list of labelled sections of the texture to look for the Team Prop in. */
    private val regions = arrayOf(
            // (0.0, 0.0) is top left, (1.0, 1.0) is bottom right
            Region(Loc.Left, x = 0.0 to 0.2, y = 0.4 to 0.6),
            Region(Loc.Center, x = 0.4 to 0.6, y = 0.4 to 0.6),
            Region(Loc.Right, x = 0.8 to 1.0, y = 0.4 to 0.6),
    )

    /** The location on the field the Team Prop is in, as measured by this processor. */
    var location = Loc.Center
        private set

    /** A secondary texture used during [processFrame] so we don't need to overwrite `input`. */
    private var filtered = Mat()


    override fun init(width: Int, height: Int, calibration: CameraCalibration?) {
        // Nothing to set up
    }

    override fun processFrame(input: Mat, captureTimeNanos: Long): Any {
        // Write to [filtered], making it white wherever the hue, saturation, and value are correct.
        Imgproc.cvtColor(input, filtered, Imgproc.COLOR_RGB2HSV)
        Core.inRange(filtered, detectColorRange.lower, detectColorRange.upper, filtered)

        // Pass this filtered image to the regions, selecting the one with the highest score
        location = regions.maxBy { region ->
            region.evalObjectProbability(filtered)
        }.loc

        return filtered
    }


    override fun onDrawFrame(canvas: Canvas, onscreenWidth: Int, onscreenHeight: Int, scaleBmpPxToCanvasPx: Float, scaleCanvasDensity: Float, userContext: Any?) {
        // draw annotations for regions
        for (region in regions) {
            region.drawAnnotation(
                    canvas,
                    onscreenWidth, onscreenHeight,
                    currentMatchedLoc = location,
            )
        }
    }

    override fun cleanup() {
        // Nothing to clean up
    }


    /** An enum representing where the Team Prop could be on the field. */
    enum class Loc {
        Left, Center, Right
    }

    /**
     * A helper class that stores the range in which the color of a pixel must
     * lie in order to be considered part of the Team Prop.
     */
    private class DetectColorRange(hueThreshold: Double, centerHue: Double, minSaturation: Double, minValue: Double) {
        val lower = Scalar(centerHue - hueThreshold, minSaturation, minValue)
        val upper = Scalar(centerHue + hueThreshold, 255.0, 255.0)
    }

    /** A labelled section of the texture to look for the Team Prop in. */
    private class Region(
            val loc: Loc,
            val x: Pair<Double, Double>,
            val y: Pair<Double, Double>,
    ) {
        private fun toRect(w: Int, h: Int) = Rect(
                Point(
                        (w * x.first).coerceAtLeast(0.0),
                        (h * y.first).coerceAtLeast(0.0),
                ),
                Point(
                        (w * x.second).coerceAtMost((w - 1).toDouble()),
                        (h * y.second).coerceAtMost((h - 1).toDouble()),
                ),
        )

        /** Returns a score proportional to how much of the input texture is white. */
        fun evalObjectProbability(filtered: Mat): Double {
            // select the part of the texture corresponding to this Region
            val regionMat = filtered.submat(
                    this.toRect(filtered.width(), filtered.height())
            )

            // mean brightness of the sub-texture.
            // (using red channel as measure of overall brightness, because they should all be the same)
            return Core.mean(regionMat).`val`[0]
        }

        /** Utility to draw some annotations on the camera viewport in [onDrawFrame]. */
        fun drawAnnotation(canvas: Canvas, w: Int, h: Int, currentMatchedLoc: Loc) {
            val (x0, x1) = x.let { (x0, x1) -> (x0 * w).toFloat() to (x1 * w).toFloat() }
            val (y0, y1) = y.let { (y0, y1) -> (y0 * h).toFloat() to (y1 * h).toFloat() }
            val paint = if (loc == currentMatchedLoc) SimpleLinePaint.green else SimpleLinePaint.white
            canvas.drawLine(x0, y0, x0, y1, paint) // left
            canvas.drawLine(x1, y0, x1, y1, paint) // right
            canvas.drawLine(x0, y0, x1, y0, paint) // top
            canvas.drawLine(x0, y1, x1, y1, paint) // bottom
        }
    }
}