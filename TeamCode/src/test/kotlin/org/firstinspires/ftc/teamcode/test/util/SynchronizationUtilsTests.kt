package org.firstinspires.ftc.teamcode.test.util

import kotlinx.coroutines.runBlocking
import org.firstinspires.ftc.teamcode.util.ValueLock
import org.junit.Test

class SynchronizationUtilsTests {
    @Test
    fun valueLock() {
        runBlocking {
            val lock = ValueLock(0) { it != 0 }

            assert(!lock.locked)

            lock.mutate { it + 1 } // nonzero, locked
            assert(lock.locked)

            lock.mutate { -69420 } // nonzero, locked
            assert(lock.locked)

            lock.mutate { 0 } // zero unlocked
            assert(!lock.locked)
        }
    }
}