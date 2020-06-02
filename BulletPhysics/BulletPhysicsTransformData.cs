using System;
using Unity.Entities;
using Unity.Mathematics;
using VisualPinball.Engine.Unity.BulletPhysics;
using VisualPinball.Unity.Extensions;

public struct BulletPhysicsTransformData : IComponentData
{
    public MotionStateNativeView motionStateView;

#if UNITY_EDITOR
	// all below is used for debug
	public RigidBodyNativeView rigidBodyView;
	public float3 phyPos;
	public float3 phyRot;
	public float friction;
	public float restitution;
	public float rollingFriction;
	public float3 torq;
	public float3 velocity;

	public void UpdateDebug()
	{
		var worldTransform = motionStateView.ToBtTransform();
		phyPos = worldTransform.pos;
		phyRot = worldTransform.rot.ToEuler() * 180.0f / math.PI;
 
		friction = rigidBodyView.Friction;
		restitution = rigidBodyView.Restitution;
		rollingFriction = rigidBodyView.RollingFriction;
		torq = rigidBodyView.AngularVelocity.ToUnity();
		velocity = rigidBodyView.LinearVelocity.ToUnity();
	}
#endif
}
