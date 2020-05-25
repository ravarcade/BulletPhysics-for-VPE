using System;
using System.Security;
using System.Runtime.InteropServices;
using UnityEngine;
using BulletSharp;
using BulletSharp.Math;

namespace VisualPinball.Engine.Unity.BulletPhysics
{
    public class BulletPhysicsMotionState : DefaultMotionState
    {
        public IntPtr GetNativePtr() { return _native; }
        public static void GetWorldTransformNotSafe(IntPtr obj, out Matrix worldTrans)
        {
            btMotionState_getWorldTransform(obj, out worldTrans);
        }

        [DllImport(Native.Dll, CallingConvention = Native.Conv), SuppressUnmanagedCodeSecurity]
        static extern void btMotionState_getWorldTransform(IntPtr obj, [Out] out Matrix worldTrans);
    }
}
