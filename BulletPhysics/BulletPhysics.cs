using Unity.Entities;
using Unity.Mathematics;
using UnityEngine;
using VisualPinball.Unity.Physics.DebugUI;
using VisualPinball.Unity.Physics.Engine;
using VisualPinball.Unity.VPT.Table;

namespace VisualPinball.Engine.Unity.BulletPhysics
{
	public class BulletPhysics : IPhysicsEngine
	{
		public string Name => "Bullet Physics";

		private readonly DebugFlipperSlider[] _flipperDebugSliders;
		private BulletPhysicsComponent _component;

		public BulletPhysics()
		{
			_flipperDebugSliders = new[] {
				new DebugFlipperSlider("Acceleration", DebugFlipperSliderParam.Acc, 0.1f, 3.0f),
				new DebugFlipperSlider("Mass (log10)", DebugFlipperSliderParam.Mass, -1.0f, 8.0f),
				new DebugFlipperSlider("Off Scale", DebugFlipperSliderParam.OffScale, 0.01f, 1.0f),
				new DebugFlipperSlider("On Near End Scale", DebugFlipperSliderParam.OnNearEndScale, 0.01f, 1.0f),
				new DebugFlipperSlider("Num of degree near end", DebugFlipperSliderParam.NumOfDegreeNearEnd, 0.1f, 10.0f)
			};
		}

		public void Init(TableBehavior tableBehavior)
		{
			// add component if not already added in editor
			_component = tableBehavior.gameObject.GetComponent<BulletPhysicsComponent>();
			if (_component == null) {
				_component = tableBehavior.gameObject.AddComponent<BulletPhysicsComponent>();
			}
			_component.Initialize();
			_component.PrepareTable();
		}

		public Entity BallCreate(Mesh mesh, Material material, in float3 worldPos, in float3 localPos, in float3 localVel,
			in float scale, in float mass, in float radius)
		{
			return _component.BallCreate(mesh, material, in worldPos, localPos, in localVel, in scale, in mass, in radius);
		}

		public void BallManualRoll(in Entity ballEntity, in float3 targetWorldPosition)
		{
			_component.ManualBallRoller(ballEntity, targetWorldPosition);
		}

		public void FlipperRotateToEnd(in Entity entity)
		{
			_component.OnRotateToEnd(entity);
		}

		public void FlipperRotateToStart(in Entity entity)
		{
			_component.OnRotateToStart(entity);
		}

		public DebugFlipperState[] FlipperGetDebugStates()
		{
			return PhyFlipper.GetFlipperStates();
		}

		public DebugFlipperSlider[] FlipperGetDebugSliders()
		{
			return _flipperDebugSliders;
		}

		public void SetFlipperDebugValue(DebugFlipperSliderParam param, float v)
		{
			_component.SetFloat(param, v);
		}

		public float GetFlipperDebugValue(DebugFlipperSliderParam param)
		{
			return _component.GetFloat(param);
		}
	}
}