using System.Collections.Generic;
using UnityEngine;
using Unity.Entities;
using Unity.Mathematics;
using VisualPinball.Unity.VPT.Flipper;
using VisualPinball.Unity.Extensions;
using BulletSharp;

using Vector3 = BulletSharp.Math.Vector3;
using Matrix = BulletSharp.Math.Matrix;


namespace VisualPinball.Engine.Unity.BulletPhysics
{
    public class PhyFlipper : PhyBody
    {
        public PhyFlipper(FlipperBehavior flipper) : base(PhyType.Flipper)
        {
            Mass = flipper.data.Mass * 3.0f;
            RotationDirection = flipper.data.StartAngle > flipper.data.EndAngle ? -1 : 1;
            _height = flipper.data.Height;
            _startAngle = flipper.data.StartAngle * Mathf.PI / 180.0f;
            _endAngle = flipper.data.EndAngle * Mathf.PI / 180.0f;

            SetupRigidBody(flipper.data.Mass, _AddFlipperCylinders(flipper));
        }

        // Adding hinge should be done after RigidBody is added to world
        public override TypedConstraint Constraint
        {
            get
            {
                if (_constraint == null)
                {
                    _constraint = _AddFlipperHinge();
                }
                return _constraint;
            }
        }

        TypedConstraint _AddFlipperHinge()
        {
            float hh = _height * 0.5f;

            var hinge = new HingeConstraint(
                body,
                new Vector3(0, 0, hh),
                Vector3.UnitZ,
                false);

            body.ActivationState = ActivationState.DisableDeactivation;
            body.SetSleepingThresholds(float.MaxValue, 0.0f); // no sleep for flippers

            if (RotationDirection == 1)
            {
                hinge.SetLimit(_startAngle, _endAngle, 0.0f);
            }
            else
            {
                hinge.SetLimit(_endAngle, _startAngle, 0.0f);
            }

            return hinge;
        }

        CollisionShape _AddFlipperCylinders(FlipperBehavior flipper)
        {
            float r1 = flipper.data.BaseRadius;
            float r2 = flipper.data.EndRadius;
            float h = flipper.data.Height;
            float l = flipper.data.FlipperRadius;

            var hh = h * 0.5f; // half height
            var cs = new BulletSharp.CompoundShape();

            cs.AddChildShape(
                Matrix.Translation(0, 0, hh),
                new CylinderShapeZ(r1, r1, hh));

            cs.AddChildShape(
                Matrix.Translation(0, -l, hh),
                new CylinderShapeZ(r2, r2, hh));

            // we can't add Triangle Mesh Shape to Compound Shape. Add one or two boxes
            float hbl = new Vector2(l, r1 - r2).magnitude * 0.5f;
            Vector3 n = new Vector3(l, r1 - r2, 0); n.Normalize();
            Vector3 beg = new Vector3(0, 0, hh) + n * (r1 - r2);
            Vector3 beg2 = new Vector3(-beg.X, beg.Y, beg.Z);
            Vector3 end = new Vector3(0, -l, hh);
            float angle = math.atan2(n.Y, n.X);

            bool onlyFront = true;
            bool rev = (flipper.data.StartAngle < 0 | flipper.data.StartAngle > 180);

            if (!onlyFront || rev)
                cs.AddChildShape(
                    Matrix.RotationZ(-angle) *
                    Matrix.Translation((beg + end) * 0.5f),
                    new BoxShape(Mathf.Min(r1, r2), hbl, hh));

            if (!onlyFront || !rev)
                cs.AddChildShape(
                    Matrix.RotationZ(angle) *
                    Matrix.Translation((beg2 + end) * 0.5f),
                    new BoxShape(Mathf.Min(r1, r2), hbl, hh));

            return cs;
        }

        /// <summary>
        /// Flipper rotation.
        /// Executed every physics simulation step.
        /// </summary>
        void _FlipperUpdate(ref BulletPhysicsComponent bpc)
        {
            _UpdateFlipperMass(ref bpc);
            float M = (float)(_usedFlipperMass * 1e7);
            float angle = _GetAngle();
            float maxAngle = math.abs(_startAngle - _endAngle) * (180.0f / math.PI);
            float flipperOnForce = bpc.flipperAcceleration;
            float flipperOffForce = flipperOnForce * bpc.flipperSolenoidOffAccelerationScale;
            if (angle > (maxAngle - bpc.flipperNumberOfDegreeNearEnd) && SolenoidState == 1)
            {
                flipperOnForce *= bpc.flipperOnNearEndAccelerationScale;
            }

            if (angle < bpc.flipperNumberOfDegreeNearEnd && SolenoidState == -1)
            {
                flipperOffForce *= bpc.flipperOnNearEndAccelerationScale;
            }

            switch (SolenoidState)
            {
                case 1:
                    body.ApplyTorque(new Vector3(0, 0, RotationDirection) * flipperOnForce * M);
                    break;
                case -1:
                    body.ApplyTorque(new Vector3(0, 0, -RotationDirection) * flipperOffForce * M);
                    break;
            }
        }

        float _GetAngle()
        {
            Matrix m = body.WorldTransform;
            float3 rot = BulletPhysicsExt.ExtractRotationFromMatrix(ref m).ToEuler();
            return math.abs(rot.z - _startAngle) * (180.0f / math.PI);
        }

        void _UpdateFlipperMass(ref BulletPhysicsComponent bpc)
        {
            // Update params from ImGui menu
            float newMass = math.pow(10.0f, bpc.flipperMassMultiplierLog);
            if (newMass != _prevFlipperMassMultiplierLog)
            {
                _prevFlipperMassMultiplierLog = newMass;
                _usedFlipperMass = Mass * _prevFlipperMassMultiplierLog;
                Vector3 inertia = body.CollisionShape.CalculateLocalInertia(_usedFlipperMass);
                body.SetMassProps(_usedFlipperMass, inertia);
            }
        }


        // ========================================================================== Data ===

        TypedConstraint _constraint = null;

        public float Mass;
        public int SolenoidState = 0;
        public int RotationDirection = 1;  /// left flipper =-1, right flipper =+1

        float _height;
        float _startAngle;
        float _endAngle;

        // flipper params
        private float _usedFlipperMass = 1.0f;
        private float _prevFlipperMassMultiplierLog = float.MaxValue;

        // ========================================================================== Static Methods & Data ===

        private readonly static Dictionary<Entity, PhyFlipper> _flippers = new Dictionary<Entity, PhyFlipper>();

        public static void OnRotateToEnd(Entity entity) { _flippers[entity].SolenoidState = 1; }
        public static void OnRotateToStart(Entity entity) { _flippers[entity].SolenoidState = -1; }
        public static bool GetFlipperState(Entity entity, out VisualPinball.Unity.DebugAndPhysicsComunicationProxy.FlipperState flipperState)
        {
            bool ret = _flippers.ContainsKey(entity);
            float angle = 0;
            bool solenoid = false;
            if (ret)
            {
                var phyFlipper = _flippers[entity];
                angle = phyFlipper._GetAngle();
                solenoid = phyFlipper.SolenoidState == 1;
            }

            flipperState = new VisualPinball.Unity.DebugAndPhysicsComunicationProxy.FlipperState(angle, solenoid);
            return ret;
        }

        public static void OnRegiesterFlipper(Entity entity, PhyFlipper phyBody)
        {
            phyBody.SolenoidState = -1; // down
            _flippers[entity] = phyBody;
        }

        public static void Update(ref BulletPhysicsComponent bpc)
        {
            foreach (var entry in _flippers)
            {
                entry.Value._FlipperUpdate(ref bpc);
            }
        }

    }
}
