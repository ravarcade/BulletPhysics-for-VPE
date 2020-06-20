using UnityEngine;
using Unity.Entities;
using BulletSharp;
using BulletSharp.Math;

namespace VisualPinball.Engine.Unity.BulletPhysics
{
    internal class PhyStatic : PhyBody
    {
        public PhyStatic(GameObject go, float mass, float friction, float elasticity) : base(PhyType.Static)
        {
            TriangleMesh btMesh = new TriangleMesh();

            var mesh = GetCombinedMesh(go);
            AddMesh(ref btMesh, mesh);

            SetupRigidBody(mass, new BvhTriangleMeshShape(btMesh, true));

            SetProperties(
                mass,
                friction,
                elasticity * 100.0f);

            base.name = go.name;
        }

        UnityEngine.Mesh GetCombinedMesh(GameObject go)
        {
            UnityEngine.Mesh mesh;
            var meshes = go.GetComponentsInChildren<MeshFilter>(true);

            if (meshes.Length == 1)
            {
                mesh = meshes[0].sharedMesh;
            }
            else
            {
                CombineInstance[] combine = new CombineInstance[meshes.Length];
                for (int i = 0; i < meshes.Length; ++i)
                {
                    combine[i].mesh = meshes[i].sharedMesh;
                    combine[i].transform = Matrix4x4.TRS(meshes[i].transform.localPosition, meshes[i].transform.localRotation, meshes[i].transform.localScale);
                }
                UnityEngine.Mesh combinedMesh = new UnityEngine.Mesh();
                combinedMesh.CombineMeshes(combine);

                mesh = combinedMesh;
            }

            return mesh;
        }

        void AddMesh(ref TriangleMesh triangleMesh, UnityEngine.Mesh mesh)
        {
            UnityEngine.Vector3[] verts = mesh.vertices;
            int[] tris = mesh.triangles;

            TriangleMesh tm = new TriangleMesh();
            for (int i = 0; i < tris.Length; i += 3)
            {
                triangleMesh.AddTriangle(verts[tris[i]].ToBullet(),
                               verts[tris[i + 1]].ToBullet(),
                               verts[tris[i + 2]].ToBullet(),
                               true);
            }
        }
    }
}