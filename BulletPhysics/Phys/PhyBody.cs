using BulletSharp;
using BulletSharp.Math;
using Unity.Entities;
using Vector3 = BulletSharp.Math.Vector3;

namespace VisualPinball.Engine.Unity.BulletPhysics
{
    public enum PhyType
    {
        Playfield = 0,
        Ball = 1,
        Static = 2,
        Flipper = 3,
        Gate = 4,
        Everything = 0x7fff
    };

    public class PhyBody
    {
        private PhyType _phyType;
        private CollisionObject _collisionObject;
        private string _name = "";
        private Entity _entity = Entity.Null;

        public object userObject { get; set; }
        public string name { get => _name; protected set { _name = value; } }
        public Entity entity { get => _entity; protected set { _entity = value; } }

        public PhyBody(PhyType pType = PhyType.Static)
        {
            _phyType = pType;
        }

        public void SetupRigidBody(float mass, CollisionShape shape, float margin = 0.04f)
        {
            shape.Margin = margin;

            var constructionInfo = new RigidBodyConstructionInfo(
                    mass,
                    CreateMotionState(),
                    shape);

            _collisionObject = new RigidBody(constructionInfo);
        }

        public short phyType { get { return (short)_phyType; } }

        public RigidBody body { get { return (RigidBody)_collisionObject; } }

        public virtual TypedConstraint Constraint { get { return null; } }

        public virtual void Register(Entity entity) { }

        public void SetProperties(float mass, float friction, float restitution)
        {
            if (_collisionObject != null)
            {
                _collisionObject.Friction = friction;
                _collisionObject.Restitution = restitution;

                body.Friction = friction;
                body.Restitution = restitution;

                if (mass > 0)
                {
                    Vector3 inertia = body.CollisionShape.CalculateLocalInertia(mass);
                    body.SetMassProps(mass, inertia);
                }
            }
        }

        public static MotionState CreateMotionState() { return new DefaultMotionState(); }

        public void SetWorldTransform(Matrix m)
        {
            body.MotionState.SetWorldTransform(ref m);
            body.WorldTransform = m;
        }
    }
}
