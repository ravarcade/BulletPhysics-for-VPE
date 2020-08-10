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
using BulletSharp.Math;

namespace VisualPinball.Engine.Unity.BulletPhysics
{
    public class PhyGate : PhyBody
    {        
        enum GateType
        {
            Spinner = 0, // yea... i want to use gate as spinner...
            OneWay = 1,
            TwoWay = 2
        }

        public float Mass;

        HingeConstraint _constraint = null;
        public int SolenoidState = 0;
        public int RotationDirection = 1;  /// left flipper =-1, right flipper =+1

        //float _height;
        float _length;
        float _thickness;
        float _barHeight;
        float _rotation;
        float _startAngle;
        float _endAngle;

        GateType _type = GateType.Spinner;

        static List<PhyGate> _gates = new List<PhyGate>();

        public PhyGate(GateBehavior gate, GateWireBehavior wire) : base(PhyType.Gate)
        {
            Mass = 0.4f; // why gates don't have mass?

            _rotation = gate.data.Rotation.ToRad();
            _startAngle = gate.data.AngleMin - 10.0f.ToRad();
            _endAngle = gate.data.AngleMax + 10.0f.ToRad();
            _type = gate.data.TwoWay ? GateType.TwoWay : GateType.OneWay;
            //s_type = GateType.TwoWays;
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
            Matrix AFrame = Matrix.RotationX(0) * Matrix.RotationZ(90.0f.ToRad()) * Matrix.RotationY(90.0f.ToRad()) * Matrix.Translation(offset); // rotatione required to make X axis pivot
            Matrix BFrame = AFrame * matrix;

            //var hinge = new HingeConstraint(
            //    body,
            //    Vector3.Zero + base.offset,
            //    Vector3.UnitX,
            //    true);

            //var ma = hinge.AFrame;
            //var mb = hinge.BFrame;
            //var oa = hinge.FrameOffsetA;
            //var ob = hinge.FrameOffsetB;
            //var im = matrix;
            //im.Invert();
            //var mb2 = mb * im;

            //hinge.Dispose();
            var hinge = new HingeConstraint(
                body,
                TypedConstraint.GetFixedBody(),
                AFrame,
                BFrame,
                true) ;

            if (_type!= GateType.Spinner)
            {
                hinge.SetLimit(_startAngle, _endAngle, 0.0f); // revers angles to have same as in VPX behavior
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

        public override void Register(Entity entity)
        {
            base.Register(entity);
            _gates.Add(this);
            AddGetToDebugWindow();
        }

        // ===================================================================== Part used only with ImGUI debug window ===

        static int dbgGate = -1;
        int dbgSa = -1;
        int dbgEa = -1;
        int dbgDamping = -1;
        int dbgHa = -1;

        private void AddGetToDebugWindow()
        { 
            var dbg = EngineProvider<IDebugUI>.Get();
            if (dbg != null)
            {
                if (dbgGate == -1)
                    dbgGate = dbg.AddProperty(-1, "Gates", this);

                int me = dbg.AddProperty(dbgGate, base.name, this);
                dbgSa = dbg.AddProperty(me, "start angle", _startAngle.ToDeg());
                dbgEa = dbg.AddProperty(me, "end angle", _endAngle.ToDeg());
                dbgDamping = dbg.AddProperty(me, "damping", body.AngularDamping);
                dbgHa = dbg.AddProperty(me, "angle", 0.0f);
            }

        }

        public static void dbg(EntityManager entityManager)
        {
            var dbg = EngineProvider<IDebugUI>.Get();
            if (dbg == null)
                return;

            foreach (var gate in _gates)
            {
                bool isLimitChanged = false;

                float sa = 0, ea = 0, damping = 0;

                isLimitChanged |= dbg.GetProperty(gate.dbgSa, ref sa);
                isLimitChanged |= dbg.GetProperty(gate.dbgEa, ref ea);
                if (isLimitChanged)
                    gate._constraint.SetLimit(sa.ToRad(), ea.ToRad(), 0);

                if (dbg.GetProperty(gate.dbgDamping, ref damping))
                    gate.body.SetDamping(damping, damping);
                float ha = gate._constraint.HingeAngle.ToDeg();
                dbg.SetProperty(gate.dbgHa, ha); // we don't read it... only display
            }            
        }
    } // PhyGate

} //  VisualPinball.Engine.Unity.BulletPhysics