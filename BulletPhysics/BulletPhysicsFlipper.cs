using System;
using UnityEngine;
using VisualPinball.Engine.Unity.BulletPhysics;
using VisualPinball.Unity.Physics;
using VisualPinball.Unity.Game;
using BulletSharp;
using BulletSharp.Math;
using Vector3 = BulletSharp.Math.Vector3;
using Unity.Entities;
using VisualPinball.Unity.VPT.Ball;
using Unity.Entities.UniversalDelegates;

// Added to remove VPX Physics
using ECS_World = Unity.Entities.World;
using VisualPinball.Unity.VPT.Flipper;
using System.Collections.Generic;
using Unity.Mathematics;
using VisualPinball.Unity.Extensions;
using VisualPinball.Engine.VPT.Flipper;

namespace VisualPinball.Engine.Unity.BulletPhysics
{
	public class PhyFlipper : PhyBody
	{
		public PhyFlipper(FlipperBehavior flipper) : base(PhyType.Flipper)
		{
			_Dbg_Init(flipper.gameObject.name);
			_flippersGameObjectToPhyBody[flipper.gameObject] = this;

			Mass = flipper.data.Mass * 3.0f;
			RotationDirection = flipper.data.StartAngle > flipper.data.EndAngle ? -1 : 1;
			_height = flipper.data.Height;
			_startAngle = flipper.data.StartAngle * Mathf.PI / 180.0f;
			_endAngle = flipper.data.EndAngle * Mathf.PI / 180.0f;

			var shape = _AddFlipperCylinders(flipper);
			Vector3 localInertia = Vector3.Zero;
			if (Mass > 1e10)
				shape.CalculateLocalInertia(Mass);

			shape.Margin = 0.04f;
			var constructionInfo = new RigidBodyConstructionInfo(
				flipper.data.Mass,
				CreateMotionState(),
				shape,
				localInertia
			);
			collisionObject = new RigidBody(constructionInfo);
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

			var hinge = new HingeConstraint(body,
				new Vector3(0, 0, hh),
				Vector3.UnitZ,
				false);

			body.ActivationState = ActivationState.DisableDeactivation;
			body.SetSleepingThresholds(float.MaxValue, 0.0f);

			if (true)
			{
				
				if (_startAngle < 0)
				{
					hinge.SetLimit(_startAngle, _endAngle, 0.0f);
				}
				else
				{
					hinge.SetLimit(_endAngle, _startAngle, 0.0f);
				}
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
		/// Execuded every physics simulation step.
		/// </summary>
		void _FlipperUpdate(ref BulletPhysicsComponent bpc)
		{
			_Dbg_OnFlipperUpdate(ref bpc);			
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
		private float prevFlipperMassMultiplierLog = float.MaxValue;

		// ========================================================================== Static Methods  & Data ===

		private readonly static Dictionary<int, PhyFlipper> _flippers = new Dictionary<int, PhyFlipper>();
		private readonly static Dictionary<GameObject, PhyFlipper> _flippersGameObjectToPhyBody = new Dictionary<GameObject, PhyFlipper>();

		public static void OnRotateToEnd(int flipperId) { _flippers[flipperId].SolenoidState = 1; }
		public static void OnRotateToStart(int flipperId) { _flippers[flipperId].SolenoidState = -1; }

		public static void OnRegiesterFlipper(GameObject go, int flipperId)
		{
			_flippers[flipperId] = _flippersGameObjectToPhyBody[go];
			_flippers[flipperId].SolenoidState = 0;
		}

		public static void Update(ref BulletPhysicsComponent bpc)
		{
			// right now for every frame, later for every simulation step
			foreach (var entry in _flippers)
			{
				entry.Value._FlipperUpdate(ref bpc);
			}
		}

		// ========================================================================== Debug & Monitoring ===

		public static void OnDebugDraw(Action<DebugFlipperData> PushUI_DebugFlipperData)
		{
			foreach (var entry in _flippers)
			{
				PushUI_DebugFlipperData(entry.Value._dbg_data);
			}
		}

		void _Dbg_Init(string name)
		{
			_dbg_data.Name = name;
			_dbg_data.SolenoidOnAngles = new List<float>();
			_dbg_data.SolenoidOffAngles = new List<float>();
			_dbg_data.SolenoidSate = 0;
		}

		void _Dbg_UpdateFlipperMass(ref BulletPhysicsComponent bpc)
		{
			// Update params from ImGui menu
			float newMass = math.pow(10.0f, bpc.flipperMassMultiplierLog);
			if (newMass != prevFlipperMassMultiplierLog)
			{
				prevFlipperMassMultiplierLog = newMass;
				_usedFlipperMass = Mass * prevFlipperMassMultiplierLog;
				Vector3 inertia = body.CollisionShape.CalculateLocalInertia(_usedFlipperMass);
				body.SetMassProps(_usedFlipperMass, inertia);
			}
		}

		void _Dbg_OnFlipperUpdate(ref BulletPhysicsComponent bpc)
		{
			_Dbg_UpdateFlipperMass(ref bpc);
			float angle = _GetAngle();

			if (SolenoidState == 1)
			{
				if (SolenoidState != _dbg_data.SolenoidSate)
					_dbg_data.SolenoidOnAngles.Clear();

				if (_dbg_data.SolenoidOnAngles.Count < _MaxSolenoidAnglesDataLength)
					_dbg_data.SolenoidOnAngles.Add(angle);
			}

			if (SolenoidState == -1)
			{
				if (SolenoidState != _dbg_data.SolenoidSate)
					_dbg_data.SolenoidOffAngles.Clear();

				if (_dbg_data.SolenoidOffAngles.Count < _MaxSolenoidAnglesDataLength)
					_dbg_data.SolenoidOffAngles.Add(angle);
			}
			_dbg_data.SolenoidSate = SolenoidState;
		}

		DebugFlipperData _dbg_data = new DebugFlipperData();
		const int _MaxSolenoidAnglesDataLength = 500;
	}
}
