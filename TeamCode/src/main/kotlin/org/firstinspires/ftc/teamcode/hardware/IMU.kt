package org.firstinspires.ftc.teamcode.hardware

import com.qualcomm.hardware.bosch.BNO055IMU
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference
import org.firstinspires.ftc.teamcode.ftcGlue.IHardwareMap
import kotlin.math.sqrt

class IMU(
        private val id: String,
        private val spinAxis: SpinAxis,
) {

    enum class SpinAxis {
        VERTICAL, SECOND, THIRD
    }


    inner class Impl(hardwareMap: IHardwareMap) {
        private val imu = hardwareMap.getRaw(id, BNO055IMU::class) ?: TODO("NOT FOUND")
        init {
            imu.initialize(BNO055IMU.Parameters().also {
                it.mode = BNO055IMU.SensorMode.IMU
                it.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC
                it.angleUnit = BNO055IMU.AngleUnit.RADIANS
            })
        }

        val spinAngle: Double
            get() {
                val orientation = imu.getAngularOrientation(
                        AxesReference.INTRINSIC,
                        AxesOrder.XYZ,
                        AngleUnit.RADIANS,
                )

                return when (spinAxis) {
                    SpinAxis.VERTICAL -> orientation.firstAngle
                    SpinAxis.SECOND -> orientation.secondAngle
                    SpinAxis.THIRD -> orientation.thirdAngle
                }.toDouble()
            }

        val gravityMagnitude
            get() = imu.gravity.let {
                sqrt(it.xAccel * it.xAccel +
                        it.yAccel * it.yAccel +
                        it.zAccel * it.zAccel
                )
            }.toDouble()
    }
}