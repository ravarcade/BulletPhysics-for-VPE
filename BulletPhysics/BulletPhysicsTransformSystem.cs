using Unity.Entities;
using Unity.Transforms;
using Unity.Jobs;
using VisualPinball.Engine.Unity.BulletPhysics;

[AlwaysSynchronizeSystem]
internal class BulletPhysicsTransformSystem : JobComponentSystem
{
    protected override JobHandle OnUpdate(JobHandle inputDeps)
    {
        //var ltw = BulletPhysicsHub.LocalToWorld;

        /**
		 * Note:
		 * (m * ltw).rotation = no rotation <- problem with math precision
		 * we need to do m.rotation * ltw.rotation to get correct object rotation
		 */
#if UNITY_EDITOR
		Entities.ForEach((ref Translation translation, ref Rotation rototation, ref BulletPhysicsTransformData transformData) =>
#else
        Entities.ForEach((ref Translation translation, ref Rotation rototation, in BulletPhysicsTransformData transformData) =>
#endif
        {
            var ms = transformData.motionStateView.ToBtTransform();
            var ltw = transformData.localToWorld;
            rototation.Value = ltw.rotation * ms.rot;
            translation.Value = ltw.MultiplyPoint(ms.pos);

#if UNITY_EDITOR
            // debug only in unity editor
            transformData.UpdateDebug();
#endif
        }).Run();

        return default;
    }
}
