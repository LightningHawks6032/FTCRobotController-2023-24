package org.firstinspires.ftc.teamcode.event

import kotlin.random.Random

interface EventSender<T> {
    fun bind(block: (T) -> Unit): EventSymbol<T>
    fun once(block: (T) -> Unit): EventSymbol<T>
    fun unbind(ev: EventSymbol<T>): Boolean

    class EventSymbol<T>(private val sender: EventSender<T>) {
        private val key = Random.nextInt()
        override fun equals(other: Any?) =
                other is EventSymbol<*> &&
                        other.sender == sender &&
                        other.key == key

        override fun hashCode() = key

        fun unbind() = sender.unbind(this)
    }

    class Default<T> : EventSender<T> {
        private val evMap = mutableMapOf<EventSymbol<T>, (T) -> Unit>()
        private val evMapOnce = mutableMapOf<EventSymbol<T>, (T) -> Unit>()
        override fun once(block: (T) -> Unit) =
                EventSymbol(this).also { evMapOnce[it] = block }
        override fun bind(block: (T) -> Unit) =
                EventSymbol(this).also { evMap[it] = block }
        override fun unbind(ev: EventSymbol<T>) =
                (evMap.remove(ev) ?: evMapOnce.remove(ev)) != null

        fun send(v: T) {
            (evMap.values + evMapOnce.values).forEach { it(v) }
            evMapOnce.clear()
        }
    }
}

fun EventSender<Unit>.bind(block: ()->Unit) = bind { block() }
fun EventSender<Unit>.once(block: ()->Unit) = once { block() }
fun EventSender.Default<Unit>.send() = send(Unit)