using Unity.Entities;
using BulletSharp;

namespace VisualPinball.Engine.Unity.BulletPhysics
{
    internal class PhyPlayfield : PhyBody
    {
        const float playfieldTickness = 5.0f; // 5mm

        public PhyPlayfield(float w, float h) : base(PhyType.Playfield)
        {
            SetupRigidBody(0, new BoxShape(w, h, playfieldTickness));
            base.name = "[Playfield]";
            base.entity = Entity.Null;
        }
    }
}