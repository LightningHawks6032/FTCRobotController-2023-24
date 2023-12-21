package org.firstinspires.ftc.teamcode.controlSystems

class ActionSequence(
        actions: List<Pair<Double, () -> Unit>>,
        private val setup: (() -> Unit)?,
        private val teardown: (() -> Unit)?,
) {
    private val actions = actions.sortedBy { it.first }
    inner class Instance {
        val setup by this@ActionSequence::setup
        val teardown by this@ActionSequence::teardown
        var t = 0.0

        fun tick(dt: Double) {
            val tNext = t + dt
            val actionsNow = actions.filter { (evTime,_) -> evTime >= t && evTime < tNext }
            for ((_,action) in actionsNow) {
                action()
            }
            t += dt
        }
    }
    class Executor {
        fun start(sequence: ActionSequence) {
            runningSequence = sequence.Instance()
        }
        var runningSequence: ActionSequence.Instance? = null
            set(value) {
                field?.teardown?.invoke()
                field = value
                value?.setup?.invoke()
            }
        fun tick(dt: Double) {
            runningSequence?.tick(dt)
        }
    }
}