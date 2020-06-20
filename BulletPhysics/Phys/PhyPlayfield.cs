using Unity.Entities;
using BulletSharp;
using VisualPinball.Unity.VPT.Table;
using BulletSharp.Math;

namespace VisualPinball.Engine.Unity.BulletPhysics
{
    internal class PhyPlayfield : PhyBody
    {
        const float playfieldTickness = 5.0f; // 5mm

        public PhyPlayfield(TableBehavior table) : base(PhyType.Playfield)
        {
            float w = table.Table.Width * 0.5f;
            float h = table.Table.Height * 0.5f;

            SetupRigidBody(0, new BoxShape(w, h, playfieldTickness));

            SetProperties(
                0,
                table.Table.Data.Friction,
                table.Table.Data.Elasticity * 100.0f);

            base.name = "[Playfield]";
            base.matrix = Matrix.Translation(w, h, playfieldTickness);
        }
    }
}