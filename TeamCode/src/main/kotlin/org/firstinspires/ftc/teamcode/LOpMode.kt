package org.firstinspires.ftc.teamcode

import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.util.RobotLog
import kotlinx.coroutines.*
import org.firstinspires.ftc.teamcode.event.WatchList
import org.firstinspires.ftc.teamcode.ftcGlue.IRobot
import org.firstinspires.ftc.teamcode.ftcGlue.WithTelemetry
import org.firstinspires.ftc.teamcode.ftcGlue.liveIntegration.LIGamepad
import org.firstinspires.ftc.teamcode.ftcGlue.liveIntegration.LIHardwareMap
import org.firstinspires.ftc.teamcode.hardware.Gamepad
import org.firstinspires.ftc.teamcode.util.Timer
import org.firstinspires.ftc.teamcode.util.TriggerLock

open class LOpMode<T: Any>(
        private val robotSpec: IRobot<T>,
        val runBlock: suspend LOpMode<T>.RunScope.() -> Unit
) : OpMode(), OpBaseScope<T> {

    val scope = CoroutineScope(Dispatchers.Default)
    private var coroutineJob: Job? = null
    private var toThrow: Throwable? = null

    final override var duringInit = false; private set
    final override var duringRun = false; private set
    final override var isStarted = false; private set
    final override var wasStopRequested = false; private set

    private val timerSinceStart = Timer()
    private val timerSinceInit = Timer()
    override val timeSinceStart by timerSinceStart::now
    override val timeSinceInit by timerSinceInit::now

    private val stopHandles = mutableSetOf<CloseScope.() -> Unit>()

    override val gamepadA = Gamepad(LIGamepad(gamepad1))
    override val gamepadB = Gamepad(LIGamepad(gamepad2))
    override val withTelemetry = WithTelemetry(telemetry)

    final override lateinit var robot: T
        private set

    /**
     * This method will be called once when the INIT button is pressed.
     */
    override fun init() {
        robot = robotSpec.impl(LIHardwareMap(hardwareMap))
        timerSinceInit.isTiming = true
        coroutineJob = scope.launch {
            runBlock(RunScope())
        }.also {
            it.invokeOnCompletion { e ->
                when (e) {
                    null -> {
                        // normal end
                        requestOpModeStop()
                    }
                    is CancellationException,
                    is InterruptedException -> {
                        // Interruption / Cancellation, shutting down the op mode
                        RobotLog.d("${this::class.simpleName} received an interrupt/cancellation; shutting down")
                        requestOpModeStop()
                    }
                    else -> {
                        // An error was thrown. don't stop, forward it to the loop
                        toThrow = e
                    }
                }
            }
        }
    }

    /**
     * This method will be called once when the PLAY button is first pressed.
     * This method is optional. By default this method takes not action.
     * Example usage: Starting another thread.
     */
    override fun start() {
        timerSinceStart.isTiming = true
        startTrigger.unlock()
    }

    private val startTrigger = TriggerLock()

    /**
     * This method will be called repeatedly when the INIT button is pressed.
     * This method is optional. By default this method takes no action.
     */
    override fun init_loop() = handleLoop()

    /**
     * This method will be called repeatedly in a loop while this op mode is running
     */
    override fun loop() = handleLoop()

    private fun handleLoop() {
        toThrow?.let { throw it }
    }

    /**
     * This method will be called when this op mode is first disabled
     */
    override fun stop() {


        // prevent getting stopped twice (because that's bad)
        if (wasStopRequested) return

        // Handle edge case of stop() before init()
        if (coroutineJob == null) return

        /*
         * Is it ending because it simply... ended (e.g. end of auto), or
         * because the user failed to monitor for the start condition?
         *
         * We must check userMethodReturned, because if it didn't return,
         * but also !userMonitoredForStart, that means the opmode was aborted
         * during init. We don't want to show a warning in that case.
         */
//        if (!userMonitoredForStart && helper.userMethodReturned) {
//            RobotLog.addGlobalWarningMessage("The OpMode which was just initialized ended prematurely as a result of not monitoring for the start condition. Did you forget to call waitForStart()?")
//        }

        // make isStopRequested() return true (and opModeIsActive() return false)
        wasStopRequested = true

        timerSinceStart.isTiming = false
        timerSinceInit.isTiming = false
        runBlocking {
            coroutineJob!!.cancelAndJoin()
        }
        CloseScope().let {
            stopHandles.forEach { handle -> handle(it) }
        }
    }


    inner class RunScope :
            OpBaseScope<T> by this,
            CoroutineScope by scope {
        suspend fun waitForStart() = startTrigger.wait()
        fun cleanupOnStop(block: CloseScope.() -> Unit) = stopHandles.add(block)

        fun createLoop(condition: () -> Boolean = { duringRun }, block: LoopScope<T>.() -> Unit) {
            launch {
                val loopScope = LoopScopeImpl()
                while (condition()) {
                    try {
                        block(loopScope)
                    } catch (_: LoopScope.Continue) {
                        // let the loop continue to next iteration
                    } catch (_: LoopScope.Break) {
                        break // break out of outer loop
                    }
                    loopScope.updateAtEndOfLoop()
                }
            }
        }
    }

    interface LoopScope<T> : OpBaseScope<T> {
        fun <K: WatchList.Watchable> watches(gen: (WatchList)->K, block: (K) -> Unit)
        object Break : Throwable()
        object Continue : Throwable()

        val dt: Double
        val BREAK: Nothing
        val CONTINUE: Nothing
    }

    inner class LoopScopeImpl : LoopScope<T>, OpBaseScope<T> by this {
        private val watchList = WatchList()
        override fun <K: WatchList.Watchable> watches(gen: (WatchList)->K, block: (K) -> Unit) {
            if (isFirstLoop)
                block(gen(watchList))
        }

        private var tLast = timeSinceInit
        private var isFirstLoop = true
        fun updateAtEndOfLoop() {
            dt = timeSinceInit - tLast
            tLast = timeSinceInit
            isFirstLoop = false
            watchList.tick()
        }

        override var dt = 0.0
            private set

        override val BREAK get() = throw LoopScope.Break
        override val CONTINUE get() = throw LoopScope.Continue
    }

    inner class CloseScope : OpBaseScope<T> by this {

    }
}

interface OpBaseScope<T> {
    val duringRun: Boolean
    val duringInit: Boolean
    val isStarted: Boolean
    val wasStopRequested: Boolean
    val timeSinceStart: Double
    val timeSinceInit: Double

    val robot: T
    val gamepadA: Gamepad
    val gamepadB: Gamepad
    val withTelemetry: WithTelemetry
}