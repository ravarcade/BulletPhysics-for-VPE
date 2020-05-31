using BulletSharp;

namespace VisualPinball.Engine.Unity.BulletPhysics
{
    internal class PhyPlayfield : PhyBody
    {
        const float playfieldTickness = 5.0f; // 5mm

        public PhyPlayfield(float w, float h) : base(PhyType.Playfield)
        {
            var constructionInfo = new RigidBodyConstructionInfo(
                0f,                                                     // static object: mass = 0
                CreateMotionState(),
                new BoxShape(w, h, playfieldTickness));
            constructionInfo.CollisionShape.Margin = 0.04f;
            constructionInfo.Friction = 0.8f;
            constructionInfo.Restitution = 0.8f;

            collisionObject = new RigidBody(constructionInfo);
        }
    }
}