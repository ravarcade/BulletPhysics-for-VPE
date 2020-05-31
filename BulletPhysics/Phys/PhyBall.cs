using BulletSharp;
using BulletSharp.Math;

namespace VisualPinball.Engine.Unity.BulletPhysics
{
    internal class PhyBall : PhyBody
		{
			public PhyBall(float radius, float mass) : base(PhyType.Ball)
			{
				var shape = new SphereShape(radius);
				shape.Margin = 0.04f;

				var constructionInfo = new RigidBodyConstructionInfo(
					mass,
					CreateMotionState(),
					shape);

				collisionObject = new RigidBody(constructionInfo);								
			}
		}
}