package org.firstinspires.ftc.teamcode.util

import kotlinx.coroutines.sync.Mutex

class TriggerLock : TriggerWaiter {


    private val mutex = Mutex(locked = true)

    fun unlock() = synchronized(mutex) {
        if (mutex.isLocked) mutex.unlock()
    }

    override suspend fun wait() =
            mutex.lock().also {
                unlock()
            }
}
interface TriggerWaiter {
    suspend fun wait()
}