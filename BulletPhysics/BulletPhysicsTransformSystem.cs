using Unity.Entities;
using Unity.Mathematics;
using Unity.Transforms;
using Unity.Jobs;
using VisualPinball.Engine.Unity.BulletPhysics;
using VisualPinball.Unity.Extensions;
using BulletSharp.Math;

[AlwaysSynchronizeSystem]
internal class BulletPhysicsTransformSystem : JobComponentSystem
{
    protected override JobHandle OnUpdate(JobHandle inputDeps)
    {
        Matrix ltw = BulletPhysicsHub.LocalToWorld;
        var ltwRot = BulletPhysicsHub.LocalToWorldRotation;

        /**
		 * Note: 
		 * (m * ltw).rotation = no rotation <- problem with math precision
		 * we neet to do m.rotation * ltw.rotation to get correct object rotation
		 */
#if UNITY_EDITOR
		Entities.ForEach((ref Translation translation, ref Rotation rototation, ref BulletPhysicsTransformData transformData) =>
#else
        Entities.ForEach((ref Translation translation, ref Rotation rototation, in BulletPhysicsTransformData transformData) =>
#endif
        {
            Matrix m;
            BulletPhysicsMotionState.GetWorldTransformNotSafe(transformData.motionStatePtr, out m);
            rototation.Value = ltwRot * BulletPhysicsExt.ExtractRotationFromMatrix(ref m);
            m *= ltw;
            translation.Value = BulletPhysicsExt.ExtractTranslationFromMatrix(ref m);

#if UNITY_EDITOR
			// debug only in unity editor
			BulletPhysicsMotionState.GetWorldTransformNotSafe(transformData.motionStatePtr, out m);
			transformData.phyPos = BulletPhysicsExt.ExtractTranslationFromMatrix(ref m);
			transformData.phyRot = BulletPhysicsExt.ExtractRotationFromMatrix(ref m).ToEuler()*180.0f / math.PI;
			var rb = PhyBody.GetCollisionObject(transformData.rigidBodyIdx);
			if (rb != null)
			{
				transformData.friction = rb.Friction;
				transformData.restitution = rb.Restitution;
				transformData.rollingFriction = rb.RollingFriction;
				transformData.torq = rb.AngularVelocity.ToUnity();
				transformData.velocity = rb.LinearVelocity.ToUnity();
			}
#endif
        }).Run();

        return default;
    }
}
