using BulletSharp;
using BulletSharp.Math;
using FluentAssertions.Extensions;
using Unity.Mathematics;
using UnityEngine;
using quaternion = Unity.Mathematics.quaternion;
using Vector3 = BulletSharp.Math.Vector3;

namespace VisualPinball.Engine.Unity.BulletPhysics
{
    public struct btTransform
    {
        public float3 pos;
        public quaternion rot;

        public btTransform(in MotionStateNativeView ms)
        {
            Matrix matrix = ms.WorldTransform;

            UnityEngine.Vector3 forward;
            forward.x = matrix.M31;
            forward.y = matrix.M32;
            forward.z = matrix.M33;

            UnityEngine.Vector3 upwards;
            upwards.x = matrix.M21;
            upwards.y = matrix.M22;
            upwards.z = matrix.M23;

            rot = quaternion.LookRotation(forward, upwards);
            pos = new float3(matrix.M41, matrix.M42, matrix.M43);
        }
    }

    public static class BulletPhysicsExt
    {
        public static btTransform ToBtTransform(this MotionStateNativeView motionStateView) { return new btTransform(motionStateView); }
        public static MotionStateNativeView ToView(this MotionState motionState, in Vector3 offset) { return new MotionStateNativeView(motionState, offset); }        
        public static RigidBodyNativeView ToView(this RigidBody rigidBody) { return new RigidBodyNativeView(rigidBody); }

        public static BulletSharp.Math.Quaternion ToBullet(this UnityEngine.Quaternion v)
        {
            return new BulletSharp.Math.Quaternion(v.x, v.y, v.z, v.w);
        }

        public static UnityEngine.Quaternion ToUnity(this BulletSharp.Math.Quaternion v)
        {
            return new UnityEngine.Quaternion(v.X, v.Y, v.Z, v.W);
        }

        public static BulletSharp.Math.Vector3 ToBullet(this UnityEngine.Vector3 v)
        {
            return new BulletSharp.Math.Vector3(v.x, v.y, v.z);
        }

        public static UnityEngine.Vector3 ToUnity(this BulletSharp.Math.Vector3 v)
        {
            return new UnityEngine.Vector3(v.X, v.Y, v.Z);
        }

        public static UnityEngine.Matrix4x4 ToUnity(this BulletSharp.Math.Matrix bm)
        {
            Matrix4x4 um = new Matrix4x4();
            um[0, 0] = bm[0, 0];
            um[0, 1] = bm[1, 0];
            um[0, 2] = bm[2, 0];
            um[0, 3] = bm[3, 0];

            um[1, 0] = bm[0, 1];
            um[1, 1] = bm[1, 1];
            um[1, 2] = bm[2, 1];
            um[1, 3] = bm[3, 1];

            um[2, 0] = bm[0, 2];
            um[2, 1] = bm[1, 2];
            um[2, 2] = bm[2, 2];
            um[2, 3] = bm[3, 2];

            um[3, 0] = bm[0, 3];
            um[3, 1] = bm[1, 3];
            um[3, 2] = bm[2, 3];
            um[3, 3] = bm[3, 3];
            return um;
        }

        public static BulletSharp.Math.Matrix ToBullet(this UnityEngine.Matrix4x4 um)
        {
            BulletSharp.Math.Matrix bm = new BulletSharp.Math.Matrix();
            um.ToBullet(ref bm);
            return bm;
        }

        public static void ToBullet(this UnityEngine.Matrix4x4 um, ref BulletSharp.Math.Matrix bm)
        {
            bm[0, 0] = um[0, 0];
            bm[0, 1] = um[1, 0];
            bm[0, 2] = um[2, 0];
            bm[0, 3] = um[3, 0];

            bm[1, 0] = um[0, 1];
            bm[1, 1] = um[1, 1];
            bm[1, 2] = um[2, 1];
            bm[1, 3] = um[3, 1];

            bm[2, 0] = um[0, 2];
            bm[2, 1] = um[1, 2];
            bm[2, 2] = um[2, 2];
            bm[2, 3] = um[3, 2];

            bm[3, 0] = um[0, 3];
            bm[3, 1] = um[1, 3];
            bm[3, 2] = um[2, 3];
            bm[3, 3] = um[3, 3];
        }

        public static float3 ExtractTranslationFromMatrix(ref BulletSharp.Math.Matrix matrix)
        {
            float3 translate;
            translate.x = matrix.M41;
            translate.y = matrix.M42;
            translate.z = matrix.M43;
            return translate;
        }

        public static quaternion ExtractRotationFromMatrix(ref BulletSharp.Math.Matrix matrix)
        {
            UnityEngine.Vector3 forward;
            forward.x = matrix.M31;
            forward.y = matrix.M32;
            forward.z = matrix.M33;

            UnityEngine.Vector3 upwards;
            upwards.x = matrix.M21;
            upwards.y = matrix.M22;
            upwards.z = matrix.M23;

            return quaternion.LookRotation(forward, upwards);
        }

        public static float ToRad(this float v) => v * math.PI / 180.0f;
        public static float ToDeg(this float v) => v * 180.0f / math.PI;
    }
}
