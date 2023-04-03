package org.firstinspires.ftc.teamcode.hardware

import org.firstinspires.ftc.teamcode.ftcGlue.IHardwareMap

interface Assembly {
    val subassemblies: Array<Assembly>?
}

fun Assembly.collectLinkable(): Set<LinkableAssembly> =
        if (this is LinkableAssembly)
            setOf(this)
        else
            (subassemblies ?: emptyArray())
                    .map { it.collectLinkable() }
                    .reduce { a, b -> a + b }

interface LinkableAssembly : Assembly {
    fun link(hardware: IHardwareMap)
}

abstract class RobotAssembly : Assembly {
    fun link(hardware: IHardwareMap) = collectLinkable().forEach { it.link(hardware) }
}

