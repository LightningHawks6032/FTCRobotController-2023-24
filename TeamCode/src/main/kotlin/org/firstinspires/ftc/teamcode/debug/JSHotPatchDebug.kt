package org.firstinspires.ftc.teamcode.debug

import io.ktor.http.*
import io.ktor.server.application.*
import io.ktor.server.engine.*
import io.ktor.server.netty.*
import io.ktor.server.request.*
import io.ktor.server.response.*
import io.ktor.server.routing.*
import kotlinx.serialization.encodeToString
import kotlinx.serialization.json.Json
import org.graalvm.polyglot.Context
import org.graalvm.polyglot.HostAccess
import org.graalvm.polyglot.PolyglotException

private interface PatchScript {
    fun init()
}

@kotlinx.serialization.Serializable
@Suppress("UNUSED")
class PatchCreateRes(
        val ok: Boolean,
        val fail: PatchCreateError? = null,
        val message: String? = null,
        val stackTrace: Array<out String>? = null,
)

@kotlinx.serialization.Serializable
enum class PatchCreateError {
    HTTP_BODY_INVALID,
    PARSE_PATCH,
    CAST_PATCH,
    UNKNOWN,
}

class JSHotPatchDebug {
    private var script: PatchScript? = null

    private val jsContext: Context = Context
            .newBuilder("js")
            .allowHostAccess(HostAccess.ALL) //allows access to all Java classes
            .allowHostClassLookup { true }.build()

    private val server = embeddedServer(Netty, 6032) {
        routing {
            get("/") {
                call.respondText("JSHotPatch ready")
            }
            post("/script") {
                val patchCreateRes = try {
                    val scriptSrc: String = call.receive()

                    val script = jsContext
                            .eval("js", scriptSrc)
                            .`as`(PatchScript::class.java)
                    script.init()

                    this@JSHotPatchDebug.script = script
                    PatchCreateRes(true)
                } catch (e: ContentTransformationException) {
                    // failed at body parse
                    PatchCreateRes(false, PatchCreateError.HTTP_BODY_INVALID)
                } catch (e: PolyglotException) {
                    // failed to run patch code
                    PatchCreateRes(false, PatchCreateError.PARSE_PATCH, e.message, e.stackTrace.map { it.toString() }.toTypedArray())
                } catch (e: ClassCastException) {
                    // failed to convert patch result to DebugScript
                    PatchCreateRes(false, PatchCreateError.CAST_PATCH, e.message)
                } catch (e: Throwable) {
                    // anything else
                    PatchCreateRes(false, PatchCreateError.UNKNOWN, e.message, e.stackTrace.map { it.toString() }.toTypedArray())
                }

                call.respondText(Json.encodeToString(patchCreateRes), ContentType.Application.Json, HttpStatusCode.OK)
            }
            post("/kill") {
                call.respondText("", ContentType.Any, HttpStatusCode.OK)
                destroy()
            }
        }
    }

    init {
        server.start(wait = true)
    }

    fun destroy() {
        server.stop()
    }
}