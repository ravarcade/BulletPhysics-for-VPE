using System;
using UnityEngine;
using VisualPinball.Unity.Physics.Bullet;
using VisualPinball.Unity.Physics;
using VisualPinball.Unity.Game;
using BulletSharp;
using BulletSharp.Math;
using Vector3 = BulletSharp.Math.Vector3;
using Unity.Entities;
using VisualPinball.Unity.VPT.Ball;
using Unity.Entities.UniversalDelegates;

// Added to remove VPX Physics
using ECS_World = Unity.Entities.World;
using VisualPinball.Unity.VPT.Flipper;
using System.Collections.Generic;

namespace VisualPinball.Unity.Physics.Bullet
{
	public class BulletPhysicsHub : MonoBehaviour, IDisposable
	{
		CollisionConfiguration CollisionConf = null;
		CollisionDispatcher Dispatcher = null;
		BroadphaseInterface Broadphase = null;
		ConstraintSolver Solver = null;
		protected DiscreteDynamicsWorld World = null;
		GhostPairCallback ghostPairCallback = null;
		double simulationTime = 0;
		double currentTime = 0;
		int stepsPerSecond = 1000;

		protected int _physicsFrame = 0;
		protected Vector3 _gravityVec = new Vector3(0, 9810.0f, 0);
		protected Matrix4x4 _worldToLocal = Matrix4x4.identity;
		protected static Matrix4x4 _localToWorld = Matrix4x4.identity;
		public static Matrix LocalToWorld { get { return _localToWorld.ToBullet(); } }
		public static UnityEngine.Quaternion LocalToWorldRotation { get { return _localToWorld.rotation; } }

		protected static bool _isDisposed = false;

		// =============================================================== rules for collisions ===

		private short[] collisionsMap = new short[] {
			1<<(short)PhyType.Ball,
			(short)PhyType.Everything,
			1<<(short)PhyType.Ball,
			1<<(short)PhyType.Ball,
		};
		
		// ========================================================================================
		// used only if AxisSweep3 is used as Broadphase (see: Initialize())
		Vector3 _axis3SweepBroadphaseMin = new Vector3(-1000.0f, -1000.0f, -30.0f);
        Vector3 _axis3SweepBroadphaseMax = new Vector3(2000.0f, 3000.0f, 1000.0f);
        const int _axis3SweepMaxProxies = 32766;

		// ========================================================================================
		protected BulletPhysicsHub()
		{
			Initialize();
		}

		protected virtual void OnDestroy()
		{
			Debug.Log("Destroying Physics World");
			Dispose(false);
		}

		public void Dispose()
        {
            Dispose(true);
            GC.SuppressFinalize(this);
        }

        public void Dispose(bool disposing)
        {
			if (_isDisposed)
				return;

			if (disposing)
			{
				// Dispose managed resources.
			}

			// All bullet rosources are unmanaged.
			if (World!= null)
			{
				World.Dispose();
				World = null;
			}
			if (Broadphase != null)
			{
				Broadphase.Dispose();
				Broadphase = null;
			}
			if (Dispatcher != null)
			{
				Dispatcher.Dispose();
				Dispatcher = null;
			}
			if (CollisionConf != null)
			{
				CollisionConf.Dispose();
				CollisionConf = null;
			}
			if (Solver != null)
			{
				Solver.Dispose();
				Solver = null;
			}

			_isDisposed = true;
		}

        public void Initialize()
        {
            CollisionConf = new DefaultCollisionConfiguration();
            Dispatcher = new CollisionDispatcher(CollisionConf);

            // Select Brodphase
            //Broadphase = new DbvtBroadphase();
            Broadphase = new AxisSweep3(_axis3SweepBroadphaseMin, _axis3SweepBroadphaseMax, _axis3SweepMaxProxies);

            DantzigSolver dtsolver = new DantzigSolver();
            Solver = new MlcpSolver(dtsolver);

            //Create a collision world of your choice, for now pass null as the constraint solver
            World = new DiscreteDynamicsWorld(Dispatcher, Broadphase, Solver, CollisionConf);

			World.SetGravity(ref _gravityVec);
			//ghostPairCallback = new GhostPairCallback();
			//World.PairCache.SetInternalGhostPairCallback(ghostPairCallback);
		}

		protected void SetGravity(float gravity, float slope)
		{
			// we work in mm not in m. =>  x 1000
			_gravityVec = (UnityEngine.Quaternion.Euler(slope, 0, 0) * new UnityEngine.Vector3(0, 0, -gravity * 1000.0f)).ToBullet();
			World?.SetGravity(ref _gravityVec);
		}

		protected void UpdatePhysics(float deltaTime)
		{
			_RemoveVPXPhysics();
			currentTime += (double)deltaTime;
			var td = currentTime - simulationTime;
			double stepTime = 1.0 / (double)stepsPerSecond;
			int steps = (int)(td * stepsPerSecond);

			if (steps > 0)
			{
				while (steps > 0)
				{
					int executedSteps = World.StepSimulation((float)stepTime, 1, (float)stepTime);
					_UpdateGameObjects();
					simulationTime += stepTime * executedSteps;
					_physicsFrame += executedSteps;
					--steps;
				}
			}
		}

		protected Matrix ExtractModelMatrix(GameObject go, bool isGlobal = false)
		{
			UnityEngine.Vector3 pos = go.transform.localPosition;
			UnityEngine.Quaternion rot = go.transform.localRotation;
			if (isGlobal)
			{
				pos = _worldToLocal.MultiplyPoint(pos);
				rot = _worldToLocal.rotation * rot;
			}
			Matrix4x4 m = Matrix4x4.TRS(pos, rot, UnityEngine.Vector3.one);
			return m.ToBullet();
		}

		private void _AddPhyBody(PhyBody phyBody)
		{
			World.AddCollisionObject(phyBody.body, (CollisionFilterGroups)(1 << phyBody.phyType), (CollisionFilterGroups)collisionsMap[phyBody.phyType]);
		}

		protected void Add(PhyBody phyBody, GameObject go, bool isGlobal = false)
        {
			_AddPhyBody(phyBody);

			// add pointer to MotionState
			var pbb = go.AddComponent<PhyBodyBehaviour>();
			pbb.MotionStatePtr = phyBody.GetMotionStateNativePtr();
			pbb.RigidBodyIdx = phyBody.RigidBodyIdx;
			phyBody.SetWorldTransform(ExtractModelMatrix(go, isGlobal));			
			
			if (phyBody.Constraint != null)
				World.AddConstraint(phyBody.Constraint);

			var bd = go.GetComponent<BallBehavior>();
			if (bd)
				Destroy(bd);

			var cte = go.GetComponent<ConvertToEntity>() ?? go.AddComponent<ConvertToEntity>();
			cte.ConversionMode = ConvertToEntity.Mode.ConvertAndDestroy;
		}

		protected void Add(PhyBody phyBody, Matrix m)
		{
			_AddPhyBody(phyBody);
			phyBody.SetWorldTransform(m);
		}

		internal class PhyBodyBehaviour : MonoBehaviour, IConvertGameObjectToEntity
		{
			public IntPtr MotionStatePtr;
			public int RigidBodyIdx;

			public void Convert(Entity entity, EntityManager dstManager, GameObjectConversionSystem conversionSystem)
			{
				dstManager.AddComponentData(entity, new BulletPhysicsTransformData
				{
					motionStatePtr = MotionStatePtr,
#if UNITY_EDITOR
					rigidBodyIdx = RigidBodyIdx
#endif
				});
			}
		}

		// ==========================================================

		protected void Add(PhyFlipper phyFlipper, GameObject go)
		{
			Add((PhyBody)phyFlipper, go);
		}

		// ==========================================================
		private void _UpdateGameObjects()
		{
			var bpc = (BulletPhysicsComponent)this;
			PhyFlipper.Update(ref bpc);			
		}

		bool _vpxPhysicsRemoved = false;
		private void _RemoveVPXPhysics()
		{
			if (!_vpxPhysicsRemoved)
			{
				_vpxPhysicsRemoved = true;

				ComponentSystemBase[] systemsToStop = new ComponentSystemBase[1];

				systemsToStop[0] = ECS_World.DefaultGameObjectInjectionWorld.GetExistingSystem<FlipperRotateSystem>();
//				systemsToStop[1] = ECS_World.DefaultGameObjectInjectionWorld.GetExistingSystem<FlipperRotateSystem>();
				foreach (var sys in systemsToStop)
				{
					if (sys == null)
					{
						_vpxPhysicsRemoved = false;
						break;
					}
					sys.Enabled = false;
				}
			}
		}
	}
}

/*
 * Links:
 * https://www.raywenderlich.com/2606-bullet-physics-tutorial-getting-started
 * https://github.com/AndresTraks/BulletSharpPInvoke/issues/42
 * 
 */
