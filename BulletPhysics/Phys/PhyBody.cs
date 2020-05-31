using System;
using UnityEngine;
using VisualPinball.Engine.Unity.BulletPhysics;
using VisualPinball.Unity.Game;
using BulletSharp;
using BulletSharp.Math;
using Vector3 = BulletSharp.Math.Vector3;
using System.Collections.Generic;
using JetBrains.Annotations;

namespace VisualPinball.Engine.Unity.BulletPhysics
{
    public enum PhyType
    {
        Playfield = 0,
        Ball = 1,
        Static = 2,
        Flipper = 3,
        Everything = 0x7fff
    };

    public class PhyBody
    {
        public PhyBody(PhyType pType = PhyType.Static)
        {
            _phyType = pType;
        }

        public object userObject { get; set; }

        private CollisionObject _collisionObject;
        protected CollisionObject collisionObject
        {
            get => _collisionObject;
            set { _collisionObject = value; _AddToList(); }
        }

        protected Vector3 inertia = Vector3.Zero;

        private PhyType _phyType;
        public short phyType { get { return (short)_phyType; } }

        public RigidBody body { get { return (RigidBody)collisionObject; } }
        public virtual TypedConstraint Constraint { get { return null; } }

        public void SetProperties(float mass, float friction, float restitution)
        {
            if (collisionObject != null)
            {
                collisionObject.Friction = friction;
                collisionObject.Restitution = restitution;

                body.Friction = friction;
                body.Restitution = restitution;

                if (mass > 0)
                {
                    inertia = body.CollisionShape.CalculateLocalInertia(mass);
                    body.SetMassProps(mass, inertia);
                }
            }
        }

        public static MotionState CreateMotionState() { return new BulletPhysicsMotionState(); }

        public void SetWorldTransform(Matrix m)
        {
            ((BulletPhysicsMotionState)body.MotionState).SetWorldTransform(ref m);
            body.WorldTransform = m;
        }

        public IntPtr GetMotionStateNativePtr() { return ((BulletPhysicsMotionState)body.MotionState).GetNativePtr(); }

        // ============================== debug and monitoring ===

        static int PhyBodyCounter = 0;
        static readonly Dictionary<int, CollisionObject> _collisionObjects = new Dictionary<int, CollisionObject>();
        public static RigidBody GetCollisionObject(int idx) { return (RigidBody)_collisionObjects?[idx]; }

        private void _AddToList()
        {
            RigidBodyIdx = PhyBodyCounter;
            _collisionObjects[RigidBodyIdx] = collisionObject;
            ++PhyBodyCounter;
        }

        public int RigidBodyIdx = -1;

    }
}
