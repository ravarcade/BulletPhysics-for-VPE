using System;
using System.Security;
using System.Runtime.InteropServices;
using BulletSharp;
using BulletSharp.Math;

namespace VisualPinball.Engine.Unity.BulletPhysics
{
    /// <summary>
    /// Used with Unity ECS to get data from bullet physics native btMotionState.
    /// Note: 
    ///   MotionStateNativeView will not control life of btMotionState object.
    ///   If original object will be destroy, you may end with app crash.
    /// </summary>
    public struct MotionStateNativeView
    {
        private IntPtr _native;
        public MotionStateNativeView(MotionState motionState) { _native = motionState._native; }


        public Matrix WorldTransform
        {
            get
            {
                Matrix m;
                btMotionState_getWorldTransform(_native, out m);
                return m;
            }
        }


        [DllImport(Native.Dll, CallingConvention = Native.Conv), SuppressUnmanagedCodeSecurity]
        static extern void btMotionState_getWorldTransform(IntPtr obj, [Out] out Matrix worldTrans);
    }
}
