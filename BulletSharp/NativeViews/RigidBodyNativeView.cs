using System;
using System.Security;
using System.Runtime.InteropServices;
using BulletSharp;
using BulletSharp.Math;

namespace VisualPinball.Engine.Unity.BulletPhysics
{
    /// <summary>
    /// Used with Unity ECS to get data from bullet physics btRigidBody.
    /// Note: 
    ///   RigidBodyNativeView will not control life of btRigidBody object.
    ///   If original object will be destroy, you may end with app crash.
    /// </summary>
    public struct RigidBodyNativeView
    {
        private IntPtr _native;
        public RigidBodyNativeView(RigidBody rigidBody) { _native = rigidBody._native; }


        public float Friction { get { return btCollisionObject_getFriction(_native); } }
        public float Restitution { get { return btCollisionObject_getRestitution(_native); } }
        public float RollingFriction { get { return btCollisionObject_getRollingFriction(_native); } }

        public Vector3 AngularVelocity
        {
            get
            {
                Vector3 v;
                btRigidBody_getAngularVelocity(_native, out v);
                return v;
            }
        }

        public Vector3 LinearVelocity
        {
            get
            {
                Vector3 v;
                btRigidBody_getLinearVelocity(_native, out v);
                return v;
            }
        }

        [DllImport(Native.Dll, CallingConvention = Native.Conv), SuppressUnmanagedCodeSecurity]
        static extern IntPtr btCollisionObject_new();
        [DllImport(Native.Dll, CallingConvention = Native.Conv), SuppressUnmanagedCodeSecurity]
        static extern void btCollisionObject_activate(IntPtr obj);
        [DllImport(Native.Dll, CallingConvention = Native.Conv), SuppressUnmanagedCodeSecurity]
        static extern void btCollisionObject_activate2(IntPtr obj, bool forceActivation);
        [DllImport(Native.Dll, CallingConvention = Native.Conv), SuppressUnmanagedCodeSecurity]
        static extern int btCollisionObject_calculateSerializeBufferSize(IntPtr obj);
        [DllImport(Native.Dll, CallingConvention = Native.Conv), SuppressUnmanagedCodeSecurity]
        [return: MarshalAs(UnmanagedType.I1)]
        static extern bool btCollisionObject_checkCollideWith(IntPtr obj, IntPtr co);
        [DllImport(Native.Dll, CallingConvention = Native.Conv), SuppressUnmanagedCodeSecurity]
        [return: MarshalAs(UnmanagedType.I1)]
        static extern bool btCollisionObject_checkCollideWithOverride(IntPtr obj, IntPtr co);
        [DllImport(Native.Dll, CallingConvention = Native.Conv), SuppressUnmanagedCodeSecurity]
        static extern void btCollisionObject_forceActivationState(IntPtr obj, ActivationState newState);
        [DllImport(Native.Dll, CallingConvention = Native.Conv), SuppressUnmanagedCodeSecurity]
        static extern ActivationState btCollisionObject_getActivationState(IntPtr obj);
        [DllImport(Native.Dll, CallingConvention = Native.Conv), SuppressUnmanagedCodeSecurity]
        static extern void btCollisionObject_getAnisotropicFriction(IntPtr obj, [Out] out Vector3 value);
        [DllImport(Native.Dll, CallingConvention = Native.Conv), SuppressUnmanagedCodeSecurity]
        static extern IntPtr btCollisionObject_getBroadphaseHandle(IntPtr obj);
        [DllImport(Native.Dll, CallingConvention = Native.Conv), SuppressUnmanagedCodeSecurity]
        static extern float btCollisionObject_getCcdMotionThreshold(IntPtr obj);
        [DllImport(Native.Dll, CallingConvention = Native.Conv), SuppressUnmanagedCodeSecurity]
        static extern float btCollisionObject_getCcdSquareMotionThreshold(IntPtr obj);
        [DllImport(Native.Dll, CallingConvention = Native.Conv), SuppressUnmanagedCodeSecurity]
        static extern float btCollisionObject_getCcdSweptSphereRadius(IntPtr obj);
        [DllImport(Native.Dll, CallingConvention = Native.Conv), SuppressUnmanagedCodeSecurity]
        static extern CollisionFlags btCollisionObject_getCollisionFlags(IntPtr obj);
        //[DllImport(Native.Dll, CallingConvention = Native.Conv), SuppressUnmanagedCodeSecurity]
        //static extern IntPtr btCollisionObject_getCollisionShape(IntPtr obj);
        [DllImport(Native.Dll, CallingConvention = Native.Conv), SuppressUnmanagedCodeSecurity]
        static extern int btCollisionObject_getCompanionId(IntPtr obj);
        [DllImport(Native.Dll, CallingConvention = Native.Conv), SuppressUnmanagedCodeSecurity]
        static extern float btCollisionObject_getContactProcessingThreshold(IntPtr obj);
        [DllImport(Native.Dll, CallingConvention = Native.Conv), SuppressUnmanagedCodeSecurity]
        static extern float btCollisionObject_getDeactivationTime(IntPtr obj);
        [DllImport(Native.Dll, CallingConvention = Native.Conv), SuppressUnmanagedCodeSecurity]
        static extern float btCollisionObject_getFriction(IntPtr obj);
        [DllImport(Native.Dll, CallingConvention = Native.Conv), SuppressUnmanagedCodeSecurity]
        static extern float btCollisionObject_getHitFraction(IntPtr obj);
        [DllImport(Native.Dll, CallingConvention = Native.Conv), SuppressUnmanagedCodeSecurity]
        static extern CollisionObjectTypes btCollisionObject_getInternalType(IntPtr obj);
        [DllImport(Native.Dll, CallingConvention = Native.Conv), SuppressUnmanagedCodeSecurity]
        static extern void btCollisionObject_getInterpolationAngularVelocity(IntPtr obj, [Out] out Vector3 angvel);
        [DllImport(Native.Dll, CallingConvention = Native.Conv), SuppressUnmanagedCodeSecurity]
        static extern void btCollisionObject_getInterpolationLinearVelocity(IntPtr obj, [Out] out Vector3 linvel);
        [DllImport(Native.Dll, CallingConvention = Native.Conv), SuppressUnmanagedCodeSecurity]
        static extern void btCollisionObject_getInterpolationWorldTransform(IntPtr obj, [Out] out Matrix trans);
        [DllImport(Native.Dll, CallingConvention = Native.Conv), SuppressUnmanagedCodeSecurity]
        static extern int btCollisionObject_getIslandTag(IntPtr obj);
        [DllImport(Native.Dll, CallingConvention = Native.Conv), SuppressUnmanagedCodeSecurity]
        static extern float btCollisionObject_getRestitution(IntPtr obj);
        [DllImport(Native.Dll, CallingConvention = Native.Conv), SuppressUnmanagedCodeSecurity]
        static extern float btCollisionObject_getRollingFriction(IntPtr obj);
        [DllImport(Native.Dll, CallingConvention = Native.Conv), SuppressUnmanagedCodeSecurity]
        static extern int btCollisionObject_getUserIndex(IntPtr obj);
        [DllImport(Native.Dll, CallingConvention = Native.Conv), SuppressUnmanagedCodeSecurity]
        static extern IntPtr btCollisionObject_getUserPointer(IntPtr obj);
        [DllImport(Native.Dll, CallingConvention = Native.Conv), SuppressUnmanagedCodeSecurity]
        static extern void btCollisionObject_getWorldTransform(IntPtr obj, [Out] out Matrix worldTrans);
        [DllImport(Native.Dll, CallingConvention = Native.Conv), SuppressUnmanagedCodeSecurity]
        [return: MarshalAs(UnmanagedType.I1)]
        static extern bool btCollisionObject_hasAnisotropicFriction(IntPtr obj);
        [DllImport(Native.Dll, CallingConvention = Native.Conv), SuppressUnmanagedCodeSecurity]
        [return: MarshalAs(UnmanagedType.I1)]
        static extern bool btCollisionObject_hasAnisotropicFriction2(IntPtr obj, AnisotropicFrictionFlags frictionMode);
        [DllImport(Native.Dll, CallingConvention = Native.Conv), SuppressUnmanagedCodeSecurity]
        [return: MarshalAs(UnmanagedType.I1)]
        static extern bool btCollisionObject_hasContactResponse(IntPtr obj);
        [DllImport(Native.Dll, CallingConvention = Native.Conv), SuppressUnmanagedCodeSecurity]
        static extern IntPtr btCollisionObject_internalGetExtensionPointer(IntPtr obj);
        [DllImport(Native.Dll, CallingConvention = Native.Conv), SuppressUnmanagedCodeSecurity]
        static extern void btCollisionObject_internalSetExtensionPointer(IntPtr obj, IntPtr pointer);
        [DllImport(Native.Dll, CallingConvention = Native.Conv), SuppressUnmanagedCodeSecurity]
        [return: MarshalAs(UnmanagedType.I1)]
        static extern bool btCollisionObject_isActive(IntPtr obj);
        [DllImport(Native.Dll, CallingConvention = Native.Conv), SuppressUnmanagedCodeSecurity]
        [return: MarshalAs(UnmanagedType.I1)]
        static extern bool btCollisionObject_isKinematicObject(IntPtr obj);
        [DllImport(Native.Dll, CallingConvention = Native.Conv), SuppressUnmanagedCodeSecurity]
        [return: MarshalAs(UnmanagedType.I1)]
        static extern bool btCollisionObject_isStaticObject(IntPtr obj);
        [DllImport(Native.Dll, CallingConvention = Native.Conv), SuppressUnmanagedCodeSecurity]
        [return: MarshalAs(UnmanagedType.I1)]
        static extern bool btCollisionObject_isStaticOrKinematicObject(IntPtr obj);
        [DllImport(Native.Dll, CallingConvention = Native.Conv), SuppressUnmanagedCodeSecurity]
        [return: MarshalAs(UnmanagedType.I1)]
        static extern bool btCollisionObject_mergesSimulationIslands(IntPtr obj);
        [DllImport(Native.Dll, CallingConvention = Native.Conv), SuppressUnmanagedCodeSecurity]
        static extern IntPtr btCollisionObject_serialize(IntPtr obj, IntPtr dataBuffer, IntPtr serializer);
        [DllImport(Native.Dll, CallingConvention = Native.Conv), SuppressUnmanagedCodeSecurity]
        static extern void btCollisionObject_serializeSingleObject(IntPtr obj, IntPtr serializer);
        [DllImport(Native.Dll, CallingConvention = Native.Conv), SuppressUnmanagedCodeSecurity]
        static extern void btCollisionObject_setActivationState(IntPtr obj, ActivationState newState);
        [DllImport(Native.Dll, CallingConvention = Native.Conv), SuppressUnmanagedCodeSecurity]
        static extern void btCollisionObject_setAnisotropicFriction(IntPtr obj, [In] ref Vector3 anisotropicFriction);
        [DllImport(Native.Dll, CallingConvention = Native.Conv), SuppressUnmanagedCodeSecurity]
        static extern void btCollisionObject_setAnisotropicFriction2(IntPtr obj, [In] ref Vector3 anisotropicFriction, AnisotropicFrictionFlags frictionMode);
        [DllImport(Native.Dll, CallingConvention = Native.Conv), SuppressUnmanagedCodeSecurity]
        static extern void btCollisionObject_setBroadphaseHandle(IntPtr obj, IntPtr handle);
        [DllImport(Native.Dll, CallingConvention = Native.Conv), SuppressUnmanagedCodeSecurity]
        static extern void btCollisionObject_setCcdMotionThreshold(IntPtr obj, float ccdMotionThreshold);
        [DllImport(Native.Dll, CallingConvention = Native.Conv), SuppressUnmanagedCodeSecurity]
        static extern void btCollisionObject_setCcdSweptSphereRadius(IntPtr obj, float radius);
        [DllImport(Native.Dll, CallingConvention = Native.Conv), SuppressUnmanagedCodeSecurity]
        static extern void btCollisionObject_setCollisionFlags(IntPtr obj, CollisionFlags flags);
        [DllImport(Native.Dll, CallingConvention = Native.Conv), SuppressUnmanagedCodeSecurity]
        static extern void btCollisionObject_setCollisionShape(IntPtr obj, IntPtr collisionShape);
        [DllImport(Native.Dll, CallingConvention = Native.Conv), SuppressUnmanagedCodeSecurity]
        static extern void btCollisionObject_setCompanionId(IntPtr obj, int id);
        [DllImport(Native.Dll, CallingConvention = Native.Conv), SuppressUnmanagedCodeSecurity]
        static extern void btCollisionObject_setContactProcessingThreshold(IntPtr obj, float contactProcessingThreshold);
        [DllImport(Native.Dll, CallingConvention = Native.Conv), SuppressUnmanagedCodeSecurity]
        static extern void btCollisionObject_setDeactivationTime(IntPtr obj, float time);
        [DllImport(Native.Dll, CallingConvention = Native.Conv), SuppressUnmanagedCodeSecurity]
        static extern void btCollisionObject_setFriction(IntPtr obj, float frict);
        [DllImport(Native.Dll, CallingConvention = Native.Conv), SuppressUnmanagedCodeSecurity]
        static extern void btCollisionObject_setHitFraction(IntPtr obj, float hitFraction);
        [DllImport(Native.Dll, CallingConvention = Native.Conv), SuppressUnmanagedCodeSecurity]
        static extern void btCollisionObject_setIgnoreCollisionCheck(IntPtr obj, IntPtr co, bool ignoreCollisionCheck);
        [DllImport(Native.Dll, CallingConvention = Native.Conv), SuppressUnmanagedCodeSecurity]
        static extern void btCollisionObject_setInterpolationAngularVelocity(IntPtr obj, [In] ref Vector3 angvel);
        [DllImport(Native.Dll, CallingConvention = Native.Conv), SuppressUnmanagedCodeSecurity]
        static extern void btCollisionObject_setInterpolationLinearVelocity(IntPtr obj, [In] ref Vector3 linvel);
        [DllImport(Native.Dll, CallingConvention = Native.Conv), SuppressUnmanagedCodeSecurity]
        static extern void btCollisionObject_setInterpolationWorldTransform(IntPtr obj, [In] ref Matrix trans);
        [DllImport(Native.Dll, CallingConvention = Native.Conv), SuppressUnmanagedCodeSecurity]
        static extern void btCollisionObject_setIslandTag(IntPtr obj, int tag);
        [DllImport(Native.Dll, CallingConvention = Native.Conv), SuppressUnmanagedCodeSecurity]
        static extern void btCollisionObject_setRestitution(IntPtr obj, float rest);
        [DllImport(Native.Dll, CallingConvention = Native.Conv), SuppressUnmanagedCodeSecurity]
        static extern void btCollisionObject_setRollingFriction(IntPtr obj, float frict);
        [DllImport(Native.Dll, CallingConvention = Native.Conv), SuppressUnmanagedCodeSecurity]
        static extern void btCollisionObject_setUserIndex(IntPtr obj, int index);
        [DllImport(Native.Dll, CallingConvention = Native.Conv), SuppressUnmanagedCodeSecurity]
        static extern void btCollisionObject_setUserPointer(IntPtr obj, IntPtr userPointer);
        [DllImport(Native.Dll, CallingConvention = Native.Conv), SuppressUnmanagedCodeSecurity]
        static extern void btCollisionObject_setWorldTransform(IntPtr obj, [In] ref Matrix worldTrans);
        [DllImport(Native.Dll, CallingConvention = Native.Conv), SuppressUnmanagedCodeSecurity]
        static extern void btCollisionObject_delete(IntPtr obj);

        [DllImport(Native.Dll, CallingConvention = Native.Conv), SuppressUnmanagedCodeSecurity]
        static extern void btRigidBody_getAngularVelocity(IntPtr obj, [Out] out Vector3 ang_vel);
        [DllImport(Native.Dll, CallingConvention = Native.Conv), SuppressUnmanagedCodeSecurity]
        static extern void btRigidBody_getLinearVelocity(IntPtr obj, [Out] out Vector3 lin_vel);
    }
}
