using System.Collections.Generic;
using UnityEngine;
using Unity.Entities;
using BulletSharp;
using VisualPinball.Unity.Physics.DebugUI;
using Vector3 = BulletSharp.Math.Vector3;
using VisualPinball.Unity.VPT.Gate;
using VisualPinball.Unity.VPT.Table;
using VisualPinball.Engine.Common;
using FluentAssertions;
using Unity.Transforms;
using VisualPinball.Unity.Extensions;
using VisualPinball.Engine.VPT.Spinner;

namespace VisualPinball.Engine.Unity.BulletPhysics
{
    public class PhyGate : PhyBody
    {
        public float Mass;

        HingeConstraint _constraint = null;
        public int SolenoidState = 0;
        public int RotationDirection = 1;  /// left flipper =-1, right flipper =+1

        //float _height;
        float _length;
        float _thickness;
        float _barHeight;
        float _startAngle;
        float _endAngle;

        // yea... i want to use gate as spinner...
        enum GateType
        {
            Spinner = 0,
            OneWay = 1,
            TwoWay = 2            
        }
        GateType _type = GateType.Spinner;

        public PhyGate(GateBehavior gate, GateWireBehavior wire) : base(PhyType.Gate)
        {
            Mass = 0.4f; // why gates don't have mass?

            _startAngle = gate.data.AngleMin - 10.0f.ToRad();
            _endAngle = gate.data.AngleMax + 10.0f.ToRad();
            _type = gate.data.TwoWay ? GateType.TwoWay : GateType.OneWay;
            if (_type == GateType.TwoWay)
                _startAngle = -_endAngle;

            //RotationDirection = flipper.data.StartAngle > flipper.data.EndAngle ? -1 : 1;
            //_height = flipper.data.Height;
            //_startAngle = flipper.data.StartAngle * Mathf.PI / 180.0f;
            //_endAngle = flipper.data.EndAngle * Mathf.PI / 180.0f;

            SetupRigidBody(Mass, _AddGateWire(gate, wire));

            SetProperties(
                Mass,
                gate.data.Friction,
                gate.data.Elasticity * 100.0f);
            body.SetDamping(gate.data.Damping, gate.data.Damping);

            // calc position of wire like from gate
            Matrix4x4 m = Matrix4x4.TRS(
                gate.gameObject.transform.localPosition,
                gate.gameObject.transform.localRotation,
                UnityEngine.Vector3.one
                );

            // wire is drawed as child of gate, so position & rotation is relative to gate
            // we need also calc transformation from physics coords system to "gate relative" coords
            var m2 = m;
            m *= Matrix4x4.Translate(-offset.ToUnity());

            base.matrix = m.ToBullet();            
            base.localToWorld = m2.inverse;
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

        HingeConstraint _AddGateHinge()
        {
            var hinge = new HingeConstraint(
                body,
                Vector3.Zero + base.offset,
                Vector3.UnitX,
                false);

            if (_type!= GateType.Spinner)
            {
                hinge.SetLimit(-_endAngle, -_startAngle, 0.0f); // revers angles to have same as in VPX behavior
            }
            
            //body.ActivationState = ActivationState.DisableDeactivation;
            //body.SetSleepingThresholds(float.MaxValue, 0.0f); // no sleep for flippers

            return hinge;
        }

        CollisionShape _AddGateWire(GateBehavior gate, GateWireBehavior wire)
        {
            var gt = gate.gameObject.transform;
            var wireMeshe = wire.gameObject.GetComponentsInChildren<MeshFilter>(true)[0];

            var meshSize = wireMeshe.mesh.bounds.size * gt.localScale.z;

            _thickness = meshSize.y;
            _length = meshSize.x;
            _barHeight = meshSize.z;

            base.offset = new Vector3(0, 0, _barHeight * 0.70f);
            return new BoxShape(_length * 0.5f, _thickness * 0.5f, _barHeight * 0.25f); // bar = 30% of height
        }

        public static Entity lastGate = Entity.Null;
        public static PhyGate phyGate = null;

        public override void Register(Entity entity)
        {
            lastGate = entity;
            phyGate = this;
            base.Register(entity);
        }

        public static void dbg(EntityManager entityManager)
        {
            var dbg = EngineProvider<IDebugUI>.Get();
            
            if (phyGate != null)
            {
                var sa = phyGate._startAngle.ToDeg();
                var ea = phyGate._endAngle.ToDeg();
                bool isChanged = false;
                isChanged |= dbg.QuickPropertySync("start angle", ref sa);
                isChanged |= dbg.QuickPropertySync("end angle", ref ea);
                if (isChanged)
                {
                    phyGate._startAngle = sa.ToRad();
                    phyGate._endAngle = ea.ToRad();
                    phyGate._constraint.SetLimit(-ea.ToRad(), -sa.ToRad(), 0.0f);
                }
            }

            if (lastGate != Entity.Null && entityManager.HasComponent<BulletPhysicsTransformData>(lastGate))
            {
                var td = entityManager.GetComponentData<BulletPhysicsTransformData>(lastGate);
                if (dbg.QuickPropertySync("diag", ref td.localToWorld))
                {
                    entityManager.SetComponentData(lastGate, td);
                }

                float damp = phyGate.body.AngularDamping;
                if (dbg.QuickPropertySync("damping", ref damp))
                {
                    phyGate.body.SetDamping(damp, damp);
                }

            }
        }
    } // PhyGate

} //  VisualPinball.Engine.Unity.BulletPhysics