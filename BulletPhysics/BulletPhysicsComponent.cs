using System;
using System.Diagnostics;
using UnityEngine;
using VisualPinball.Unity.Physics;
using VisualPinball.Unity.Physics.Bullet;
using VisualPinball.Unity.Game;
using BulletSharp;
using BulletSharp.Math;
using Vector3 = BulletSharp.Math.Vector3;

using VisualPinball.Unity.VPT.Table;
using VisualPinball.Unity.VPT.Primitive;
using VisualPinball.Unity.VPT.Surface;
using VisualPinball.Unity.VPT.Flipper;
using Unity.Mathematics;

namespace VisualPinball.Unity.Physics
{
	[AddComponentMenu("Visual Pinball/Bullet Physics Component")]
	[DisallowMultipleComponent]
	public class BulletPhysicsComponent : BulletPhysicsHub, IPhysicsEngine
	{
		// properties
		[SerializeField]
		float _slope = 9.0f;
		public float slope { get { return _slope; } set { _slope = value; base.SetGravity(_gravity, slope); } }

		[SerializeField]
		new float _gravity = 9.81f;
		public float gravity { get { return _gravity; } set { _gravity = value; base.SetGravity(_gravity, slope); } }

		[Header("Flipper Settings")]
		[SerializeField] 
		public float flipperAcceleration = 1.5f;
		
		[SerializeField]
		[Tooltip("Logaritmic:\n-1=0.1,\n 0=1,\n 1=10,\n 2=100,\n...")]
		public float flipperMassMultiplierLog = 1.0f;
		
		[SerializeField] 
		public float flipperSolenoidOffAccelerationScale = 0.1f;
		
		[SerializeField] 
		public float flipperOnNearEndAccelerationScale = 0.1f;

		[SerializeField] 
		public float flipperNumberOfDegreeNearEnd = 5.0f;

		Player _player = null;

		enum TimingMode { RealTime, Atleast60, Locked60 };
		TimingMode timingMode = TimingMode.Locked60;
		float _currentPhysicsTime = 0;

		float GetTargetTime()
		{
			const float dt60fps = 1.0f / 60.0f;
			float t = _currentPhysicsTime + Time.deltaTime;

			switch (timingMode)
			{
				case TimingMode.Atleast60:
					float dt = Time.deltaTime;
					if (dt > dt60fps)
					{
						dt = dt60fps;
					}
					t = _currentPhysicsTime + dt;
					break;

				case TimingMode.Locked60:
					t = _currentPhysicsTime + dt60fps;
					break;
			}
			return t;
		}

		// ================================================= MonoBehaviour  ===
		protected void Awake()
		{
			// register Bullet Physics Engine
			var players = GameObject.FindObjectsOfType<Player>();
			players?[0].RegisterPhysicsEngine(this);
		}

		protected void Update()
		{
			Stopwatch stopwatch = new Stopwatch();
			stopwatch.Start();
			base.UpdatePhysics(GetTargetTime());
			if (PushUI_PhysicsProcessingTime != null)
				PushUI_PhysicsProcessingTime.Invoke((float)stopwatch.Elapsed.TotalMilliseconds);
		}

		protected override void OnDestroy()
		{			
			base.OnDestroy();
		}

		// ================================================ IPhysicsEngine  ===
		public void DebugPressKey(int key) { }
		public void Initialize(Player player) 
		{			
			_player = player;

			foreach (var table in GameObject.FindObjectsOfType<TableBehavior>())
			{
				AddPlayfield(table);

				//flippers = new BRigidBody[table.gameObject.GetComponentsInChildren<FlipperBehavior>(true).Length];

				foreach (var flipper in table.gameObject.GetComponentsInChildren<FlipperBehavior>(true))
					AddFlipper(flipper);


				foreach (var primitive in table.gameObject.GetComponentsInChildren<PrimitiveBehavior>(true))
					if (primitive.data.IsCollidable)
						AddStaticMesh(primitive.gameObject, 0, primitive.data.Friction, primitive.data.Elasticity);

				foreach (var surface in table.gameObject.GetComponentsInChildren<SurfaceBehavior>(true))
					if (surface.data.IsCollidable)
						AddStaticMesh(surface.gameObject, 0, surface.data.Friction, surface.data.Elasticity);
				
			}
		}

		public void OnCreateBall(GameObject go, float radius, float mass) { AddBall(go, radius, mass); }
		public void OnRotateToEnd(int flipperId) { PhyFlipper.OnRotateToEnd(flipperId); }
		public void OnRotateToStart(int flipperId) { PhyFlipper.OnRotateToStart(flipperId); }
		public void OnRegiesterFlipper(GameObject flipperGameObject, int flipperId) { PhyFlipper.OnRegiesterFlipper(flipperGameObject, flipperId); }
		public int GetFrameCount() { return base._physicsFrame; }
		public event Action<float> PushUI_PhysicsProcessingTime;
		public event Action<DebugFlipperData> PushUI_DebugFlipperData;
		public event Func<int, int, int> GetUI_Int;
		public event Func<int, float, float> GetUI_Float;

		BulletSharp.RigidBody ballBody = null;

		public void ManualBallRoller(UnityEngine.Vector3 cursor)
		{
			if (ballBody != null)
			{
				Vector3 target = _worldToLocal.MultiplyPoint(cursor).ToBullet();
				Matrix m;
				ballBody.GetWorldTransform(out m);
				Vector3 ballPos =((UnityEngine.Vector3) BulletPhysicsExt.ExtractTranslationFromMatrix(ref m)).ToBullet();
				target.Z = ballPos.Z;
				var dir = (target - ballPos);
				dir.Normalize();
				float dist = (target - ballPos).Length * 0.05f;
				if (dist > 10) dist = 10;
				if (dist > 0.001f)
					ballBody.ApplyCentralImpulse(dist * dist * dir);
			}
		}

		public void OnDebugDraw()
		{
			if (GetUI_Float != null)
			{
				// get params from UI
				flipperAcceleration = GetUI_Float(0, flipperAcceleration);
				flipperMassMultiplierLog = GetUI_Float(1, flipperMassMultiplierLog);
				flipperSolenoidOffAccelerationScale = GetUI_Float(2, flipperSolenoidOffAccelerationScale);
				flipperOnNearEndAccelerationScale = GetUI_Float(3, flipperOnNearEndAccelerationScale);
				flipperNumberOfDegreeNearEnd = GetUI_Float(4, flipperNumberOfDegreeNearEnd);
			}

			if (PushUI_DebugFlipperData != null)
			{				
				PhyFlipper.OnDebugDraw(PushUI_DebugFlipperData);
			}
		}

		public float GetDebugFloat(int paramIdx, float currentVal)
		{
			if (GetUI_Float != null)
				return GetUI_Float(paramIdx, currentVal);

			return currentVal;
		}

		// ====================================================================== === ===

		internal class PhyPlayfield : PhyBody
		{
			const float playfieldTickness = 5.0f; // 5mm
			public PhyPlayfield(float w, float h) : base(PhyType.Playfield)
			{
				var constructionInfo = new RigidBodyConstructionInfo(
					0f,														// static object: mass = 0
					CreateMotionState(),
					new BoxShape(w, h, playfieldTickness));
				constructionInfo.CollisionShape.Margin = 0.04f;
				constructionInfo.Friction = 0.8f;
				constructionInfo.Restitution = 0.8f;

				collisionObject = new RigidBody(constructionInfo);								
			}
		}

		PhyPlayfield _playfield = null;

		// ====================================================================== === ===

		internal class PhyBall : PhyBody
		{
			public PhyBall(float radius, float mass) : base(PhyType.Ball)
			{
				var shape = new SphereShape(radius);
				Vector3 localInertia = Vector3.Zero;
				if (mass > 1e10)
					shape.CalculateLocalInertia(mass);

				shape.Margin = 0.04f;
				var constructionInfo = new RigidBodyConstructionInfo(
					mass,                                                       // static object: mass = 0
					CreateMotionState(),
					shape,
					localInertia
				);

				collisionObject = new RigidBody(constructionInfo);								
			}
		}

		// ====================================================================== === ===

		internal class PhyMesh : PhyBody
		{
			public PhyMesh(GameObject go, float mass) : base(PhyType.Static)
			{
				TriangleMesh btMesh = new TriangleMesh();

				var mesh = GetCombinedMesh(go);
				AddMesh(ref btMesh, mesh);				
				var shape = new BvhTriangleMeshShape(btMesh, true);
				shape.Margin = 0.04f;

				var constructionInfo = new RigidBodyConstructionInfo(
					mass,                                                       // static object: mass = 0
					CreateMotionState(),
					shape
				);

				collisionObject = new RigidBody(constructionInfo);
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

			void AddMesh(ref TriangleMesh triangleMesh,  UnityEngine.Mesh mesh)
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

		// ====================================================================== === ===

		void AddPlayfield(TableBehavior table)
		{
			_localToWorld = table.gameObject.transform.localToWorldMatrix;
			_worldToLocal = table.gameObject.transform.worldToLocalMatrix;
			SetGravity(gravity, slope);

			_playfield = new PhyPlayfield(table.Table.Width * 0.5f, table.Table.Height * 0.5f);

			// ToDo: get correct playfield params
			_playfield.SetProperties(
				0,
				table.Table.Data.Friction, // get correct playfield params
				table.Table.Data.Elasticity * 100.0f);

			var tr = _worldToLocal.MultiplyPoint(UnityEngine.Vector3.zero);
			Add(_playfield, Matrix.Translation(tr.x, tr.y, tr.z));

		}

		void AddBall(GameObject go, float radius, float mass)
		{
			var ball = new PhyBall(radius, mass);

			// ToDo: get correct ball params
			ball.SetProperties(
				mass, 
				0.8f, 
				0.01f);
			Add(ball, go, true); // ball is global object,
			ballBody = ball.body;
		}

		void AddStaticMesh(GameObject go, float mass, float friction, float elasticity)
		{
			var meshes = go.GetComponentsInChildren<MeshFilter>(true);
			if (meshes.Length == 0)
				return;

			var body = new PhyMesh(go, mass);
			body.SetProperties(
				mass,
				friction,
				elasticity * 100.0f);

			Add(body, Matrix.Identity);
		}

		void AddFlipper(FlipperBehavior flipper)
		{			
			var phyFlipper = new PhyFlipper(flipper);
			phyFlipper.SetProperties(
				phyFlipper.Mass,
				flipper.data.Friction,
				flipper.data.Elasticity * 100.0f);

			Add(phyFlipper, flipper.gameObject);
		}
	}
}
