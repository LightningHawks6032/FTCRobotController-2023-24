package org.firstinspires.ftc.teamcode.event

class WatchList(
        list:MutableSet<Watchable> = mutableSetOf()
) : MutableSet<WatchList.Watchable> by list {

    fun tick() {
    println("OTick ${this.size}")
    forEach { it.tick() }
    }

    interface Watchable {
        fun tick()
    }
}