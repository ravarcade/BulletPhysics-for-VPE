using BulletSharp;
using BulletSharp.Math;
using Unity.Entities.UniversalDelegates;

namespace VisualPinball.Engine.Unity.BulletPhysics
{
    internal class PhyBall : PhyBody
    {
        public PhyBall(float radius, float mass) : base(PhyType.Ball)
        {
            SetupRigidBody(mass, new SphereShape(radius));
        }
    }
}