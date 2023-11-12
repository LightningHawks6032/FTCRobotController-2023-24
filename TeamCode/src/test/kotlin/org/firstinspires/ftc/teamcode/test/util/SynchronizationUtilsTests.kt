package org.firstinspires.ftc.teamcode.test.util

import kotlinx.coroutines.delay
import kotlinx.coroutines.launch
import kotlinx.coroutines.runBlocking
import org.firstinspires.ftc.teamcode.util.TriggerLock
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

    @Test
    fun triggerLockRelocking() {
        runBlocking {
            val triggerLock = TriggerLock(relock = true)
            var i = -1
            var finished = false
            launch {
                for (_x in 0 until 5) {
                    delay(10)
                    i += 1
                    delay(10)
                    triggerLock.unlock()
                }
            }
            launch {
                for (j in 0 until 5) {
                    triggerLock.wait()
                    assert(i == j)
                }
                finished = true
            }
            launch {
                delay(10*2*5+250)
                if (!finished) {
                    throw Error("Time limit exceeded, `triggerLock.wait()` hung.")
                }
            }
        }
    }
}