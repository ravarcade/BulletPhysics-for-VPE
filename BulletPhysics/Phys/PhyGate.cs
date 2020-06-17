using System.Collections.Generic;
using UnityEngine;
using Unity.Entities;
using Unity.Mathematics;
using VisualPinball.Unity.VPT.Flipper;
using VisualPinball.Unity.Extensions;
using BulletSharp;
using VisualPinball.Unity.Physics.DebugUI;
using Vector3 = BulletSharp.Math.Vector3;
using Matrix = BulletSharp.Math.Matrix;
using VisualPinball.Unity.VPT.Gate;
using SixLabors.Primitives;
using System.Net.NetworkInformation;

namespace VisualPinball.Engine.Unity.BulletPhysics
{
    public class PhyGate : PhyBody
    {
        public float Mass;

        TypedConstraint _constraint = null;
        public int SolenoidState = 0;
        public int RotationDirection = 1;  /// left flipper =-1, right flipper =+1

        float _height;
        float _length;
        float _thickness;
        float _startAngle;
        float _endAngle;

        public PhyGate(GateBehavior gate, GateWireBehavior wire) : base(PhyType.Gate)
        {
            Mass = 1.0f; // why gates don't have mass?


            //RotationDirection = flipper.data.StartAngle > flipper.data.EndAngle ? -1 : 1;
            //_height = flipper.data.Height;
            //_startAngle = flipper.data.StartAngle * Mathf.PI / 180.0f;
            //_endAngle = flipper.data.EndAngle * Mathf.PI / 180.0f;

            SetupRigidBody(Mass, _AddGateWire(gate, wire));
            SetProperties(
                Mass,
                gate.data.Friction,
                gate.data.Elasticity * 100.0f);
            body.SetDamping(0, gate.data.Damping);
            base.name = gate.name;
            base.entity = Entity.Null;
        }

        public override TypedConstraint Constraint
        {
            get
            {
                if (_constraint == null)
                {
                    _constraint = _AddGateHinge();
                }
                return _constraint;
            }
        }

        TypedConstraint _AddGateHinge()
        {
            var hinge = new HingeConstraint(
                body,
                new Vector3(0, 0, 0),
                new Vector3(1, 0, 0),
                false);
            

            //body.ActivationState = ActivationState.DisableDeactivation;
            //body.SetSleepingThresholds(float.MaxValue, 0.0f); // no sleep for flippers

            //if (RotationDirection == 1)
            //{
            //    hinge.SetLimit(_startAngle, _endAngle, 0.0f);
            //}
            //else
            //{
            //    hinge.SetLimit(_endAngle, _startAngle, 0.0f);
            //}

            return hinge;
        }

        CollisionShape _AddGateWire(GateBehavior gate, GateWireBehavior wire)
        {
            var gt = gate.gameObject.transform;
            var wt = wire.gameObject.transform;            
            _height = gt.localPosition.z;
            _length = gt.localScale.z;
            _thickness = 0;
//            _rotationQ = gt.localRotation;
            var wireGo = wire.gameObject;
            var wireMeshes = wireGo.GetComponentsInChildren<MeshFilter>(true);

            foreach (var m in wireMeshes)
            {
                var meshSize = m.mesh.bounds.extents * gt.localScale.z;
                
                if (_thickness < meshSize.y)
                    _thickness = meshSize.y;
            }

            var cs = new CompoundShape();
            cs.AddChildShape(
                Matrix.Translation(0,0,-_height * 0.5f),
                new BoxShape(_length, _thickness, _height)
                );
            return cs;
            //return new BoxShape(_length, _thickness, _height);
        }
    } // PhyGate

} //  VisualPinball.Engine.Unity.BulletPhysics