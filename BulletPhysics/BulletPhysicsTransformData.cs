using Unity.Mathematics;
using Unity.Entities;
using System;

using BulletSharp.Math;

[GenerateAuthoringComponent]
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
