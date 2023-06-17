package org.firstinspires.ftc.teamcode.test.util

import org.slf4j.ILoggerFactory
import org.slf4j.Logger
import org.slf4j.Marker
import org.slf4j.impl.StaticLoggerBinder
import java.text.SimpleDateFormat
import java.util.*

/**
 * Because things like Ktor require an SLF4J logger implementation
 * that is normally provided by android, but unavailable in this
 * context, this function is here to override the default behavior
 * with a test-environment-friendly version.
 *
 * Call before any `Logger`s get created in any libraries of interest.
 */
fun bindTestEnvSLF4JLogger() {
    // Reflect into the system that creates `Logger`s
    // then replace the LoggerFactory with our own.
    val binder = StaticLoggerBinder.getSingleton()
    binder::class.java.getDeclaredField("loggerFactory")
            .also { it.isAccessible = true }
            .set(binder, testEnvSLF4JLoggerFactory)
}

val nowString get() = SimpleDateFormat("yyyy-MM-dd HH:mm:ss").format(Date()) as String

private val testEnvSLF4JLoggerFactory = ILoggerFactory { name -> TestEnvSLF4JLogger(name) }
private class TestEnvSLF4JLogger(private val name: String) : Logger {
    override fun getName() = name

    override fun isTraceEnabled() = false

    override fun isTraceEnabled(marker: Marker?) = false

    override fun trace(msg: String?) {}

    override fun trace(format: String?, arg: Any?) {}

    override fun trace(format: String?, arg1: Any?, arg2: Any?) {}

    override fun trace(format: String?, vararg arguments: Any?) {}

    override fun trace(msg: String?, t: Throwable?) {}

    override fun trace(marker: Marker?, msg: String?) {}

    override fun trace(marker: Marker?, format: String?, arg: Any?) {}

    override fun trace(marker: Marker?, format: String?, arg1: Any?, arg2: Any?) {}

    override fun trace(marker: Marker?, format: String?, vararg argArray: Any?) {}

    override fun trace(marker: Marker?, msg: String?, t: Throwable?) {}

    override fun isDebugEnabled() = false

    override fun isDebugEnabled(marker: Marker?) = false

    override fun debug(msg: String?) {}

    override fun debug(format: String?, arg: Any?) {}

    override fun debug(format: String?, arg1: Any?, arg2: Any?) {}

    override fun debug(format: String?, vararg arguments: Any?) {}

    override fun debug(msg: String?, t: Throwable?) {}

    override fun debug(marker: Marker?, msg: String?) {}

    override fun debug(marker: Marker?, format: String?, arg: Any?) {}

    override fun debug(marker: Marker?, format: String?, arg1: Any?, arg2: Any?) {}

    override fun debug(marker: Marker?, format: String?, vararg arguments: Any?) {}

    override fun debug(marker: Marker?, msg: String?, t: Throwable?) {}

    override fun isInfoEnabled() = true

    override fun isInfoEnabled(marker: Marker?) = true

    override fun info(msg: String?) {
        println("\u001B[34m[INFO:$name $nowString]\u001B[0m $msg")
    }

    override fun info(format: String?, arg: Any?) {
        info("$format :: $arg")
    }

    override fun info(format: String?, arg1: Any?, arg2: Any?) {
        info("$format :: $arg1, $arg2")
    }

    override fun info(format: String?, vararg arguments: Any?) {
        info("$format :: $arguments")
    }

    override fun info(msg: String?, t: Throwable?) {
        info("$msg :: $t")
    }

    override fun info(marker: Marker?, msg: String?) {
        info(msg)
    }

    override fun info(marker: Marker?, format: String?, arg: Any?) {
        info(format, arg)
    }

    override fun info(marker: Marker?, format: String?, arg1: Any?, arg2: Any?) {
        info(format, arg1, arg2)
    }

    override fun info(marker: Marker?, format: String?, vararg arguments: Any?) {
        info(format, *arguments)
    }

    override fun info(marker: Marker?, msg: String?, t: Throwable?) {
        info(msg, t)
    }

    override fun isWarnEnabled() = true

    override fun isWarnEnabled(marker: Marker?) = false

    override fun warn(msg: String?) {
        println("\u001B[33m[WARN:$name $nowString] $msg\u001B[0m")
    }

    override fun warn(format: String?, arg: Any?) {
        warn("$format :: $arg")
    }

    override fun warn(format: String?, vararg arguments: Any?) {
        warn("$format :: $arguments")
    }

    override fun warn(format: String?, arg1: Any?, arg2: Any?) {
        warn("$format :: $arg1, $arg2")
    }

    override fun warn(msg: String?, t: Throwable?) {
        warn("$msg :: $t")
    }

    override fun warn(marker: Marker?, msg: String?) {
        warn(msg)
    }

    override fun warn(marker: Marker?, format: String?, arg: Any?) {
        warn(format, arg)
    }

    override fun warn(marker: Marker?, format: String?, arg1: Any?, arg2: Any?) {
        warn(format, arg1, arg2)
    }

    override fun warn(marker: Marker?, format: String?, vararg arguments: Any?) {
        warn(format, *arguments)
    }

    override fun warn(marker: Marker?, msg: String?, t: Throwable?) {
        warn(msg, t)
    }

    override fun isErrorEnabled() = true

    override fun isErrorEnabled(marker: Marker?) = false

    override fun error(msg: String?) {
        println("\u001B[31m[ERROR:$name $nowString] $msg\u001B[0m")
    }

    override fun error(format: String?, arg: Any?) {
        error("$format :: $arg")
    }

    override fun error(format: String?, arg1: Any?, arg2: Any?) {
        error("$format :: $arg1, $arg2")
    }

    override fun error(format: String?, vararg arguments: Any?) {
        error("$format :: $arguments")
    }

    override fun error(msg: String?, t: Throwable?) {
        error("$msg :: $t")
    }

    override fun error(marker: Marker?, msg: String?) {
        error(msg)
    }

    override fun error(marker: Marker?, format: String?, arg: Any?) {
        error(format, arg)
    }

    override fun error(marker: Marker?, format: String?, arg1: Any?, arg2: Any?) {
        error(format, arg1, arg2)
    }

    override fun error(marker: Marker?, format: String?, vararg arguments: Any?) {
        error(format, *arguments)
    }

    override fun error(marker: Marker?, msg: String?, t: Throwable?) {
        error(msg, t)
    }

}