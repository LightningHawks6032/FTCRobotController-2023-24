package org.firstinspires.ftc.teamcode.test

import org.firstinspires.ftc.teamcode.util.Quaternion
import org.firstinspires.ftc.teamcode.util.Transform3D
import org.firstinspires.ftc.teamcode.util.Vec3
import org.firstinspires.ftc.teamcode.util.tf
import org.junit.Test

class Transform3DTest {
    @Test
    fun r() {

        val tfAToB = Transform3D(Vec3(1.234, 5.678, 9.876), Quaternion(0.5, 0.5, 0.5, 0.5))
        val tfBToC = Transform3D(Vec3(4.321, 1.234, -2.532), Quaternion(0.0, 0.707, 0.707, 0.0))

        val v = arrayOf(
                Vec3(1.0, 0.0, 0.0),
                Vec3(0.0, 1.0, 0.0),
                Vec3(0.0, 0.0, 1.0),
                Vec3(1.3, 5.3, 2.3),
                Vec3(54.3, -3.1, 1.3),
        )
        val q = arrayOf(
                Quaternion(1.0, 0.0, 0.0, 0.0),
                Quaternion(0.0, 1.0, 0.0, 0.0),
                Quaternion(0.0, 0.0, 1.0, 0.0),
                Quaternion(0.0, 0.0, 0.0, 1.0),
                Quaternion(1.3, 5.3, 2.3, 4.2).normalized,
                Quaternion(54.3, -3.1, 1.3, 4.2).normalized,
        )

        assertUnchangedIsh("pos transform forward and inverse failed", *v, ) {
            tfAToB.transformInv(tfAToB.transformFwd(it))
        }
        assertUnchangedIsh("(A -> B) then (B -> A) was not identity", *v, ) {
            it.tf(tfAToB).tf(tfAToB.inverse())
        }
        assertUnchangedIsh("rot transform forward and inverse failed", *q, ) {
            tfAToB.transformInv(tfAToB.transformFwd(it))
        }
        assertUnchangedIsh("(A -> B) then (B -> A) was not identity", *q, ) {
            it.tf(tfAToB).tf(tfAToB.inverse())
        }

        val tfAtoC = tfAToB then tfBToC
        val tfCtoA = tfBToC.inverse() then tfAToB.inverse()

        assertUnchangedIsh("(A -> C) then (C -> B) then (B -> A) was not identity", *v) {
            it.tf(tfAtoC).tf(tfBToC.inverse()).tf(tfAToB.inverse())
        }
        assertUnchangedIsh("(A -> B) then (B -> C) then (C -> A) was not identity", *v) {
            it.tf(tfAToB).tf(tfBToC).tf(tfCtoA)
        }
    }

    private fun assertUnchangedIsh(msg: String, vararg vectors: Vec3, hopefullyIdentity: (Vec3) -> Vec3) {
        for (vec in vectors) {
            val out = hopefullyIdentity(vec)
            // It's pretty large but probably fine
            assert((vec - out).mag < 0.5) { "[ACTING ON $vec, GOT $out]\n$msg" }
        }
    }
    private fun assertUnchangedIsh(msg: String, vararg vectors: Quaternion, hopefullyIdentity: (Quaternion) -> Quaternion) {
        for (rot in vectors) {
            val out = hopefullyIdentity(rot)
            assert(rot.x - out.x < 1e-5 && rot.y - out.y < 1e-5 && rot.z - out.z < 1e-5 && rot.w - out.w < 1e-5) {
                "[ACTING ON $rot, GOT $out]\n$msg"
            }
        }
    }
}