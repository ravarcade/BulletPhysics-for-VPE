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

        public float LinearDamping
        {
            get
            {
                return btRigidBody_getLinearDamping(_native);
            }
        }
        public float AngularDamping
        {
            get
            {
                return btRigidBody_getAngularDamping(_native);
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

        // ===================

        [DllImport(Native.Dll, CallingConvention = Native.Conv), SuppressUnmanagedCodeSecurity]
        static extern IntPtr btRigidBody_new(IntPtr constructionInfo);
        [DllImport(Native.Dll, CallingConvention = Native.Conv), SuppressUnmanagedCodeSecurity]
        static extern void btRigidBody_addConstraintRef(IntPtr obj, IntPtr c);
        [DllImport(Native.Dll, CallingConvention = Native.Conv), SuppressUnmanagedCodeSecurity]
        static extern void btRigidBody_applyCentralForce(IntPtr obj, [In] ref Vector3 force);
        [DllImport(Native.Dll, CallingConvention = Native.Conv), SuppressUnmanagedCodeSecurity]
        static extern void btRigidBody_applyCentralImpulse(IntPtr obj, [In] ref Vector3 impulse);
        [DllImport(Native.Dll, CallingConvention = Native.Conv), SuppressUnmanagedCodeSecurity]
        static extern void btRigidBody_applyDamping(IntPtr obj, float timeStep);
        [DllImport(Native.Dll, CallingConvention = Native.Conv), SuppressUnmanagedCodeSecurity]
        static extern void btRigidBody_applyForce(IntPtr obj, [In] ref Vector3 force, [In] ref Vector3 rel_pos);
        [DllImport(Native.Dll, CallingConvention = Native.Conv), SuppressUnmanagedCodeSecurity]
        static extern void btRigidBody_applyGravity(IntPtr obj);
        [DllImport(Native.Dll, CallingConvention = Native.Conv), SuppressUnmanagedCodeSecurity]
        static extern void btRigidBody_applyImpulse(IntPtr obj, [In] ref Vector3 impulse, [In] ref Vector3 rel_pos);
        [DllImport(Native.Dll, CallingConvention = Native.Conv), SuppressUnmanagedCodeSecurity]
        static extern void btRigidBody_applyTorque(IntPtr obj, [In] ref Vector3 torque);
        [DllImport(Native.Dll, CallingConvention = Native.Conv), SuppressUnmanagedCodeSecurity]
        static extern void btRigidBody_applyTorqueImpulse(IntPtr obj, [In] ref Vector3 torque);
        [DllImport(Native.Dll, CallingConvention = Native.Conv), SuppressUnmanagedCodeSecurity]
        static extern void btRigidBody_clearForces(IntPtr obj);
        [DllImport(Native.Dll, CallingConvention = Native.Conv), SuppressUnmanagedCodeSecurity]
        static extern float btRigidBody_computeAngularImpulseDenominator(IntPtr obj, [In] ref Vector3 axis);
        [DllImport(Native.Dll, CallingConvention = Native.Conv), SuppressUnmanagedCodeSecurity]
        static extern void btRigidBody_computeGyroscopicForceExplicit(IntPtr obj, float maxGyroscopicForce, [Out] out Vector3 value);
        [DllImport(Native.Dll, CallingConvention = Native.Conv), SuppressUnmanagedCodeSecurity]
        static extern void btRigidBody_computeGyroscopicImpulseImplicit_Body(IntPtr obj, float step, [Out] out Vector3 value);
        [DllImport(Native.Dll, CallingConvention = Native.Conv), SuppressUnmanagedCodeSecurity]
        static extern void btRigidBody_computeGyroscopicImpulseImplicit_World(IntPtr obj, float dt, [Out] out Vector3 value);
        [DllImport(Native.Dll, CallingConvention = Native.Conv), SuppressUnmanagedCodeSecurity]
        static extern float btRigidBody_computeImpulseDenominator(IntPtr obj, [In] ref Vector3 pos, [In] ref Vector3 normal);
        [DllImport(Native.Dll, CallingConvention = Native.Conv), SuppressUnmanagedCodeSecurity]
        static extern void btRigidBody_getAabb(IntPtr obj, [Out] out Vector3 aabbMin, [Out] out Vector3 aabbMax);
        [DllImport(Native.Dll, CallingConvention = Native.Conv), SuppressUnmanagedCodeSecurity]
        static extern float btRigidBody_getAngularDamping(IntPtr obj);
        [DllImport(Native.Dll, CallingConvention = Native.Conv), SuppressUnmanagedCodeSecurity]
        static extern void btRigidBody_getAngularFactor(IntPtr obj, [Out] out Vector3 angFac);
        [DllImport(Native.Dll, CallingConvention = Native.Conv), SuppressUnmanagedCodeSecurity]
        static extern float btRigidBody_getAngularSleepingThreshold(IntPtr obj);
        [DllImport(Native.Dll, CallingConvention = Native.Conv), SuppressUnmanagedCodeSecurity]
        static extern void btRigidBody_getAngularVelocity(IntPtr obj, [Out] out Vector3 ang_vel);
        [DllImport(Native.Dll, CallingConvention = Native.Conv), SuppressUnmanagedCodeSecurity]
        static extern IntPtr btRigidBody_getBroadphaseProxy(IntPtr obj);
        [DllImport(Native.Dll, CallingConvention = Native.Conv), SuppressUnmanagedCodeSecurity]
        static extern void btRigidBody_getCenterOfMassPosition(IntPtr obj, [Out] out Vector3 value);
        [DllImport(Native.Dll, CallingConvention = Native.Conv), SuppressUnmanagedCodeSecurity]
        static extern void btRigidBody_getCenterOfMassTransform(IntPtr obj, [Out] out Matrix xform);
        //[DllImport(Native.Dll, CallingConvention = Native.Conv), SuppressUnmanagedCodeSecurity]
        //static extern IntPtr btRigidBody_getConstraintRef(IntPtr obj, int index);
        [DllImport(Native.Dll, CallingConvention = Native.Conv), SuppressUnmanagedCodeSecurity]
        static extern int btRigidBody_getContactSolverType(IntPtr obj);
        [DllImport(Native.Dll, CallingConvention = Native.Conv), SuppressUnmanagedCodeSecurity]
        static extern RigidBodyFlags btRigidBody_getFlags(IntPtr obj);
        [DllImport(Native.Dll, CallingConvention = Native.Conv), SuppressUnmanagedCodeSecurity]
        static extern int btRigidBody_getFrictionSolverType(IntPtr obj);
        [DllImport(Native.Dll, CallingConvention = Native.Conv), SuppressUnmanagedCodeSecurity]
        static extern void btRigidBody_getGravity(IntPtr obj, [Out] out Vector3 acceleration);
        [DllImport(Native.Dll, CallingConvention = Native.Conv), SuppressUnmanagedCodeSecurity]
        static extern void btRigidBody_getInvInertiaDiagLocal(IntPtr obj, [Out] out Vector3 diagInvInertia);
        [DllImport(Native.Dll, CallingConvention = Native.Conv), SuppressUnmanagedCodeSecurity]
        static extern void btRigidBody_getInvInertiaTensorWorld(IntPtr obj, [Out] out Matrix value);
        [DllImport(Native.Dll, CallingConvention = Native.Conv), SuppressUnmanagedCodeSecurity]
        static extern float btRigidBody_getInvMass(IntPtr obj);
        [DllImport(Native.Dll, CallingConvention = Native.Conv), SuppressUnmanagedCodeSecurity]
        static extern float btRigidBody_getLinearDamping(IntPtr obj);
        [DllImport(Native.Dll, CallingConvention = Native.Conv), SuppressUnmanagedCodeSecurity]
        static extern void btRigidBody_getLinearFactor(IntPtr obj, [Out] out Vector3 linearFactor);
        [DllImport(Native.Dll, CallingConvention = Native.Conv), SuppressUnmanagedCodeSecurity]
        static extern float btRigidBody_getLinearSleepingThreshold(IntPtr obj);
        [DllImport(Native.Dll, CallingConvention = Native.Conv), SuppressUnmanagedCodeSecurity]
        static extern void btRigidBody_getLinearVelocity(IntPtr obj, [Out] out Vector3 lin_vel);
        [DllImport(Native.Dll, CallingConvention = Native.Conv), SuppressUnmanagedCodeSecurity]
        static extern void btRigidBody_getLocalInertia(IntPtr obj, [Out] out Vector3 value);
        [DllImport(Native.Dll, CallingConvention = Native.Conv), SuppressUnmanagedCodeSecurity]
        static extern IntPtr btRigidBody_getMotionState(IntPtr obj);
        //[DllImport(Native.Dll, CallingConvention = Native.Conv), SuppressUnmanagedCodeSecurity]
        //static extern int btRigidBody_getNumConstraintRefs(IntPtr obj);
        [DllImport(Native.Dll, CallingConvention = Native.Conv), SuppressUnmanagedCodeSecurity]
        static extern void btRigidBody_getOrientation(IntPtr obj, [Out] out Quaternion value);
        [DllImport(Native.Dll, CallingConvention = Native.Conv), SuppressUnmanagedCodeSecurity]
        static extern void btRigidBody_getTotalForce(IntPtr obj, [Out] out Vector3 value);
        [DllImport(Native.Dll, CallingConvention = Native.Conv), SuppressUnmanagedCodeSecurity]
        static extern void btRigidBody_getTotalTorque(IntPtr obj, [Out] out Vector3 value);
        [DllImport(Native.Dll, CallingConvention = Native.Conv), SuppressUnmanagedCodeSecurity]
        static extern void btRigidBody_getVelocityInLocalPoint(IntPtr obj, [In] ref Vector3 rel_pos, [Out] out Vector3 value);
        [DllImport(Native.Dll, CallingConvention = Native.Conv), SuppressUnmanagedCodeSecurity]
        static extern void btRigidBody_integrateVelocities(IntPtr obj, float step);
        [DllImport(Native.Dll, CallingConvention = Native.Conv), SuppressUnmanagedCodeSecurity]
        [return: MarshalAs(UnmanagedType.I1)]
        static extern bool btRigidBody_isInWorld(IntPtr obj);
        [DllImport(Native.Dll, CallingConvention = Native.Conv), SuppressUnmanagedCodeSecurity]
        static extern void btRigidBody_predictIntegratedTransform(IntPtr obj, float step, [Out] out Matrix predictedTransform);
        [DllImport(Native.Dll, CallingConvention = Native.Conv), SuppressUnmanagedCodeSecurity]
        static extern void btRigidBody_proceedToTransform(IntPtr obj, [In] ref Matrix newTrans);
        [DllImport(Native.Dll, CallingConvention = Native.Conv), SuppressUnmanagedCodeSecurity]
        static extern void btRigidBody_removeConstraintRef(IntPtr obj, IntPtr c);
        [DllImport(Native.Dll, CallingConvention = Native.Conv), SuppressUnmanagedCodeSecurity]
        static extern void btRigidBody_saveKinematicState(IntPtr obj, float step);
        [DllImport(Native.Dll, CallingConvention = Native.Conv), SuppressUnmanagedCodeSecurity]
        static extern void btRigidBody_setAngularFactor(IntPtr obj, [In] ref Vector3 angFac);
        [DllImport(Native.Dll, CallingConvention = Native.Conv), SuppressUnmanagedCodeSecurity]
        static extern void btRigidBody_setAngularFactor2(IntPtr obj, float angFac);
        [DllImport(Native.Dll, CallingConvention = Native.Conv), SuppressUnmanagedCodeSecurity]
        static extern void btRigidBody_setAngularVelocity(IntPtr obj, [In] ref Vector3 ang_vel);
        [DllImport(Native.Dll, CallingConvention = Native.Conv), SuppressUnmanagedCodeSecurity]
        static extern void btRigidBody_setCenterOfMassTransform(IntPtr obj, [In] ref Matrix xform);
        [DllImport(Native.Dll, CallingConvention = Native.Conv), SuppressUnmanagedCodeSecurity]
        static extern void btRigidBody_setContactSolverType(IntPtr obj, int value);
        [DllImport(Native.Dll, CallingConvention = Native.Conv), SuppressUnmanagedCodeSecurity]
        static extern void btRigidBody_setDamping(IntPtr obj, float lin_damping, float ang_damping);
        [DllImport(Native.Dll, CallingConvention = Native.Conv), SuppressUnmanagedCodeSecurity]
        static extern void btRigidBody_setFlags(IntPtr obj, RigidBodyFlags flags);
        [DllImport(Native.Dll, CallingConvention = Native.Conv), SuppressUnmanagedCodeSecurity]
        static extern void btRigidBody_setFrictionSolverType(IntPtr obj, int value);
        [DllImport(Native.Dll, CallingConvention = Native.Conv), SuppressUnmanagedCodeSecurity]
        static extern void btRigidBody_setGravity(IntPtr obj, [In] ref Vector3 acceleration);
        [DllImport(Native.Dll, CallingConvention = Native.Conv), SuppressUnmanagedCodeSecurity]
        static extern void btRigidBody_setInvInertiaDiagLocal(IntPtr obj, [In] ref Vector3 diagInvInertia);
        [DllImport(Native.Dll, CallingConvention = Native.Conv), SuppressUnmanagedCodeSecurity]
        static extern void btRigidBody_setLinearFactor(IntPtr obj, [In] ref Vector3 linearFactor);
        [DllImport(Native.Dll, CallingConvention = Native.Conv), SuppressUnmanagedCodeSecurity]
        static extern void btRigidBody_setLinearVelocity(IntPtr obj, [In] ref Vector3 lin_vel);
        [DllImport(Native.Dll, CallingConvention = Native.Conv), SuppressUnmanagedCodeSecurity]
        static extern void btRigidBody_setMassProps(IntPtr obj, float mass, [In] ref Vector3 inertia);
        [DllImport(Native.Dll, CallingConvention = Native.Conv), SuppressUnmanagedCodeSecurity]
        static extern void btRigidBody_setMotionState(IntPtr obj, IntPtr motionState);
        [DllImport(Native.Dll, CallingConvention = Native.Conv), SuppressUnmanagedCodeSecurity]
        static extern void btRigidBody_setNewBroadphaseProxy(IntPtr obj, IntPtr broadphaseProxy);
        [DllImport(Native.Dll, CallingConvention = Native.Conv), SuppressUnmanagedCodeSecurity]
        static extern void btRigidBody_setSleepingThresholds(IntPtr obj, float linear, float angular);
        [DllImport(Native.Dll, CallingConvention = Native.Conv), SuppressUnmanagedCodeSecurity]
        static extern void btRigidBody_translate(IntPtr obj, [In] ref Vector3 v);
        [DllImport(Native.Dll, CallingConvention = Native.Conv), SuppressUnmanagedCodeSecurity]
        static extern IntPtr btRigidBody_upcast(IntPtr colObj);
        [DllImport(Native.Dll, CallingConvention = Native.Conv), SuppressUnmanagedCodeSecurity]
        static extern void btRigidBody_updateDeactivation(IntPtr obj, float timeStep);
        [DllImport(Native.Dll, CallingConvention = Native.Conv), SuppressUnmanagedCodeSecurity]
        static extern void btRigidBody_updateInertiaTensor(IntPtr obj);


    }
}
