using System;
using Unity.Entities;
using Unity.Mathematics;

public struct BulletPhysicsTransformData : IComponentData
{
	public IntPtr motionStatePtr;

#if UNITY_EDITOR
	// all below is used for debug
	public int rigidBodyIdx;
	public float3 phyPos;
	public float3 phyRot;
	public float friction;
	public float restitution;
	public float rollingFriction;
	public float3 torq;
	public float3 velocity;
#endif
}
