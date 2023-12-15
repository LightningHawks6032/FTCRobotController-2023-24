package org.firstinspires.ftc.teamcode.robot.arbot.opmodes

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import io.ktor.server.application.*
import io.ktor.server.engine.*
import io.ktor.server.netty.*
import io.ktor.server.response.*
import io.ktor.server.routing.*
import kotlinx.coroutines.delay
import kotlinx.coroutines.yield
import org.firstinspires.ftc.teamcode.LOpMode
import org.firstinspires.ftc.teamcode.hardware.Motor
import org.firstinspires.ftc.teamcode.robot.arbot.ArBotRobot
import org.firstinspires.ftc.teamcode.util.NotForCompetition
import kotlin.math.absoluteValue

private class MotorExperimentData {
    class Column(val name: String) {
        val data = mutableListOf<Double>()
    }

    private val columns = mutableListOf<Column>()
    fun column(name: String) = Column(name).also { columns.add(it) }

    fun getCSV(): String {
        val rows = mutableListOf(columns.joinToString(",") { it.name })
        for (i in 0 until (columns.maxOfOrNull { it.data.size } ?: 0)) {
            rows.add(columns.joinToString(",") {
                it.data.getOrNull(i)?.toString() ?: ""
            })
        }
        return rows.joinToString("\n")
    }
}

//private const val PORT = 6032
//private const val FILENAME = "data.csv"

@OptIn(NotForCompetition::class)
@Autonomous
class MotorAccelerationExperiment : LOpMode<ArBotRobot.Impl>(ArBotRobot, {
    val data = MotorExperimentData()
//    val server = embeddedServer(Netty, port = PORT) {
//        routing {
//            get("/$FILENAME") {
//                call.respondText(data.getCSV())
//            }
//        }
//    }
//    server.start()
//    cleanupOnStop {
//        server.stop()
//    }


    val motorRaw = Motor("a", Motor.PhysicalSpec.GOBILDA_5202_0002_0005).Impl(rawHardware)

    val pos by motorRaw::pos
    var power by motorRaw::power
    var vel = 0.0
    var acc = 0.0
    var lastPos = 0.0
    var dta = 0.0
    createLoop({ duringInit || duringRun }) {
//        println("$pos, ${pos - lastPos}, $vel, $acc, $dt")
        dta += dt
        if (dta > 0.2 || dta > 0.025 && pos != lastPos) {
            val newVel = (pos - lastPos) / dta
            acc = (newVel - vel) / dta
            vel = newVel
            lastPos = pos
            dta = 0.0
        }
    }

    createLoop({ duringInit }) {
        withTelemetry {
            ln("motor experiment, hold motor steady")
            ln("WARNING: this will automatically spin one motor, do not be alarmed")
            ln("(using motor 'a' (Expansion Hub, Port 0), requires odometry cable)")
            ln("P: $pos, V: $vel, A: $acc")
        }
    }

    waitForStart()
    withTelemetry {
        ln("hold motor steady")
    }

    val colPow = data.column("power")
    val colPos = data.column("pos")
    val colVel = data.column("vel")
    val colAcc = data.column("acc")
    for (spinPower in doubleArrayOf(
            0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1.0, 1.1,
    )) {
        delay(500)

        withTelemetry {
            ln("running spin up test at power $spinPower")
        }
        power = spinPower
        for (i in 0 until 50) {
            delay(50)
            colPow.data.add(spinPower)
            colPos.data.add(pos)
            colVel.data.add(vel)
            colAcc.data.add(acc)
        }

        // spin down
        withTelemetry {
            ln("spinning down...")
        }
        power = 0.0
        while (vel.absoluteValue > 0.1) {
            yield()
        }
    }
    for (spinBackPow in doubleArrayOf(
            -1.0, -0.8, -0.6, -0.4, -0.2, 0.0, 0.2, 0.4, 0.6, 0.8,
    )) {
        delay(200)

        // spin up
        withTelemetry {
            ln("spinning up...")
        }
        power = 1.0
        delay(4000)

        withTelemetry {
            ln("running spin back test at power $spinBackPow")
        }
        power = spinBackPow
        for (i in 0 until 50) {
            delay(50)
            colPow.data.add(spinBackPow)
            colPos.data.add(pos)
            colVel.data.add(vel)
            colAcc.data.add(acc)
        }
    }
    // spin down
    power = 0.0
    while (vel.absoluteValue > 0.1) {
        yield()
    }

    // done
    withTelemetry {
//        ln("done, connect to robot wifi, output is at http://192.168.1.1:$PORT/$FILENAME")

        ln("done, output is in logcat")
        print(":::::::::::::::\n".repeat(10) + "\n\n" + data.getCSV() + "\n\n:::::::::::::::")
    }
}, FinishedBehaviour.KEEP_ALIVE_INDEFINITELY)