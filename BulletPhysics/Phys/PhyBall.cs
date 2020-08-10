using System.Collections.Generic;
using Unity.Entities;
using BulletSharp;

namespace VisualPinball.Engine.Unity.BulletPhysics
{
    internal class PhyBall : PhyBody
    {
        private readonly static Dictionary<Entity, PhyBall> _balls = new Dictionary<Entity, PhyBall>();

        public PhyBall(Entity entity, float radius, float mass) : base(PhyType.Ball, false)
        {
            SetupRigidBody(mass, new SphereShape(radius));
            SetProperties(
                mass,
                0.3f,
                0.012f);
            _balls[entity] = this;
            base.name = "Ball" + Count;
            base.entity = entity;
        }


        public static int Count { get => _balls.Count; }

        public static PhyBall Get(Entity entity)
        {
            return _balls.ContainsKey(entity) ? _balls[entity] : null;
        }
    }
}