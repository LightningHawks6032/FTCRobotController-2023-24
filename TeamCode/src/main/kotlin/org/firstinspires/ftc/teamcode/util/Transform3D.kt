package org.firstinspires.ftc.teamcode.util

import org.firstinspires.ftc.robotcore.external.navigation.Quaternion

data class Transform3D(
        val offset: Vec3,
        val rotation: Quaternion,
) {
    // Note 0: quaternions rotate things that's all you need to know

    //////// Note 1: How to transformations //
    // let v: vec
    ///// (a to world)
    // let r: quaternion
    // let t: vec
    ///// (a to world).applyTo(v)
    // let v_world = r.applyTo(v) + t
    ///// inverse(a to world).applyTo(v)
    // let v_a = r.inverse().applyTo(v_world - t)
    //////////////////////////////////////////////////////

    //////// Note 2: How to quaternions //////////////////
    // let a: quaternion
    // let b: quaternion
    ///// Notice! the order of multiplication is backwards!
    // let a_then_b = b*a
    //////////////////////////////////////////////////////


    fun transformFwd(vec: Vec3) =  // q v q' + t
            rotation.apply(vec) + offset
    fun transformFwd(rot: Quaternion): Quaternion = // q r
            rotation * rot
    fun transformInv(vec: Vec3) = // q' (v - t) q
            rotation.inverse().apply(vec - offset)
    fun transformInv(rot: Quaternion): Quaternion = // q' r
            rotation.inverse() * rot

    fun inverse() = // T = q' -t q ; Q = q'
            Transform3D(rotation.inverse().apply(-offset), rotation.inverse())

    // transformFwd(transformInv( (v, r) ))
    //   -> (q q' (v - t) q q' + t ,  q q' r) = (1 (v-t) 1 + t ,  1 r) = (v, r)  // correct

    // transformFwd1(transformFwd0( (v, r) )) = transform2( (v, r) )
    //   -> (q1 (q0 v q0' + t0) q1' + t1 ,  q1 q0 r)
    //    = (q1 q0 v q0' q1' + q1 t0 q1' + t1 ,  q1 q0 r)
    // > q2 = q1 q0
    //    = (q2 v q2' + q1 t0 q1' + t1,  q2 r)
    // > t2 = q1 t0 q1' + t1
    //    = (q2 v q2' + t2,  q2 r)

    infix fun then(next: Transform3D) =
            Transform3D(
                    next.rotation.apply(offset) + next.offset, // t2 = q1 t0 q1' + t1
                    next.rotation * rotation, // q2 = q1 q0
            )
}

fun Vec3.tf(transform3D: Transform3D) = transform3D.transformFwd(this)
fun Quaternion.tf(transform3D: Transform3D) = transform3D.transformFwd(this)