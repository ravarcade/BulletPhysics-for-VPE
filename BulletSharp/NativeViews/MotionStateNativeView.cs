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
        private Matrix _localOffset;

        public MotionStateNativeView(MotionState motionState, in Vector3 offset) 
        { 
            _native = motionState._native; 
            _localOffset = Matrix.Translation(offset); 
        }


        public Matrix WorldTransform
        {
            get
            {
                Matrix m;
                btMotionState_getWorldTransform(_native, out m);
                m = _localOffset * m;
                return m;
            }
        }


        [DllImport(Native.Dll, CallingConvention = Native.Conv), SuppressUnmanagedCodeSecurity]
        static extern void btMotionState_getWorldTransform(IntPtr obj, [Out] out Matrix worldTrans);
    }
}
