package org.firstinspires.ftc.teamcode.util

import kotlinx.coroutines.sync.Mutex
import kotlinx.coroutines.sync.withLock

class TriggerLock : LockWaiter {
    private val mutex = Mutex(locked = true)

    fun unlock() = synchronized(mutex) {
        if (mutex.isLocked) mutex.unlock()
    }

    override suspend fun wait() =
            mutex.lock().also {
                unlock()
            }

    override val locked get() = mutex.isLocked
}

class BoolLock(locked: Boolean) : LockWaiter {
    override var locked = locked
        set(value) {
            // only update on change (mutex will throw otherwise ¯\_(ツ)_/¯)
            if (value != field) {
                // thread synchronization just in case
                // keeps mutex lock state matching `locked` field
                synchronized(mutex) {
                    if (value)
                        mutex.tryLock()
                    else
                        mutex.unlock()
                    field = value
                }
            }
        }
    private val mutex = Mutex(locked)

    override suspend fun wait() {
        // only put in effort if we're actually locked
        if (mutex.isLocked) {
            // try to acquire lock (requires waiting, only one at a time)
            mutex.lock()
            // thread synchronization just in case
            // keeps mutex lock state matching `locked` field
            synchronized(mutex) {
                if (!this.locked)
                    mutex.unlock()
            }
        }
    }
}


class ValueLock<T>(
        private var value: T,
        private val lockCondition: (T) -> Boolean
) : LockWaiter {
    private val valueMutex = Mutex()
    suspend fun mutate(block:(T)->T) {
        valueMutex.withLock {
            value = block(value)
            lock.locked = lockCondition(value)
        }
    }

    private val lock = BoolLock(lockCondition(value))
    override suspend fun wait() = lock.wait()
    override val locked by lock::locked
}

interface LockWaiter {
    suspend fun wait()
    val locked: Boolean
}

suspend fun waitAll(vararg locks: LockWaiter) {
    while (locks.any { it.locked })
        for (lock in locks)
            lock.wait()
}