using System;
using System.Diagnostics;
using System.Collections.Generic;
using UnityEngine;
using Unity.Entities;
using VisualPinball.Unity.Game;
using VisualPinball.Unity.VPT.Flipper;
using VisualPinball.Unity.Physics.SystemGroup;
using BulletSharp;
using VisualPinball.Engine.Common;
using VisualPinball.Unity.Physics.DebugUI;
using Vector3 = BulletSharp.Math.Vector3;
using Matrix = BulletSharp.Math.Matrix;
using ECS_World = Unity.Entities.World;

namespace VisualPinball.Engine.Unity.BulletPhysics
{
    public abstract class BulletPhysicsHub : MonoBehaviour, IDisposable
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
        public static Matrix4x4 LocalToWorld { get { return _localToWorld; } }
        protected static bool _isDisposed = false;

        EntityManager _entityManager;
        bool _isEntityManagerSet = false;
        public EntityManager entityManager
        {
            get
            {
                if (!_isEntityManagerSet)
                {
                    _entityManager = ECS_World.DefaultGameObjectInjectionWorld.EntityManager;
                    _isEntityManagerSet = true;
                }
                return _entityManager;
            }
        }

        // =============================================================== rules for collisions ===

        private short[] _collisionsMap = new short[] {
            1<<(short)PhyType.Ball,			// playfield :-collide-with-: ball
			(short)PhyType.Everything,      // ball :-collide-with-: everything
			1<<(short)PhyType.Ball,         // static objects :-collide-with-: ball
			1<<(short)PhyType.Ball,         // flipper :-collide-with-: ball
		};

        // ========================================================================================

        protected virtual void OnDestroy()
        {
            UnityEngine.Debug.Log("Destroying Physics World");
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
            if (World != null)
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
            if (World != null) // check if we are already initialized
                return;

            enabled = true;

            CollisionConf = new DefaultCollisionConfiguration();
            Dispatcher = new CollisionDispatcher(CollisionConf);

            // Select Brodphase
            Broadphase = new DbvtBroadphase();
            //Broadphase = new AxisSweep3(new Vector3(-1000.0f, -1000.0f, -30.0f), new Vector3(2000.0f, 3000.0f, 1000.0f), 32766);

            DantzigSolver dtsolver = new DantzigSolver();
            Solver = new MlcpSolver(dtsolver);

            //Create a collision world of your choice, for now pass null as the constraint solver
            World = new DiscreteDynamicsWorld(Dispatcher, Broadphase, Solver, CollisionConf);
            var si = World.SolverInfo;
            si.NumIterations = 2;
            si.TimeStep = 0.001f;

            World.SetGravity(ref _gravityVec);
            ghostPairCallback = new GhostPairCallback();
            World.PairCache.SetInternalGhostPairCallback(ghostPairCallback);
        }

        protected void SetGravity(float gravity, float slope)
        {
            // we work in mm not in m. =>  x 1000
            _gravityVec = (Quaternion.Euler(slope, 0, 0) * new UnityEngine.Vector3(0, 0, -gravity * 1000.0f)).ToBullet();
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
                    Stopwatch stopwatch = new Stopwatch();
                    stopwatch.Start();

                    int executedSteps = World.StepSimulation((float)stepTime, 1, (float)stepTime);
                    _UpdateGameObjects();
                    simulationTime += stepTime * executedSteps;
                    _physicsFrame += executedSteps;
                    --steps;

                    if (EngineProvider<IDebugUI>.Exists) {
                        EngineProvider<IDebugUI>.Get().OnPhysicsUpdate(1, (float)stopwatch.Elapsed.TotalMilliseconds);
                    }
                }
            }
        }

        protected Matrix ExtractModelMatrix(GameObject go)
        {
            UnityEngine.Vector3 pos = go.transform.localPosition;
            Quaternion rot = go.transform.localRotation;
            Matrix4x4 m = Matrix4x4.TRS(pos, rot, UnityEngine.Vector3.one);

            return m.ToBullet();
        }

        // ========================================================================== Add PhyBody to physics world ===

        private void _AddPhyBody(PhyBody phyBody)
        {
            World.AddCollisionObject(phyBody.body, (CollisionFilterGroups)(1 << phyBody.phyType), (CollisionFilterGroups)_collisionsMap[phyBody.phyType]);
        }

        protected void Add(PhyBody phyBody, Entity entity, Matrix m)
        {
            _AddPhyBody(phyBody);

            if (entity != Entity.Null)
            {
                entityManager.AddComponent<BulletPhysicsTransformData>(entity);
                entityManager.AddComponentData(entity, new BulletPhysicsTransformData
                {
                    motionStateView = phyBody.body.MotionState.ToView(),
#if UNITY_EDITOR
                    rigidBodyView = phyBody.body.ToView(),
#endif
                });
            }
            phyBody.SetWorldTransform(m);

            if (phyBody.Constraint != null)
                World.AddConstraint(phyBody.Constraint);
        }

        protected void Add(PhyBody phyBody, Entity entity, UnityEngine.Vector3 pos, Quaternion rot)
        {
            Add(phyBody, entity, Matrix4x4.TRS(pos, rot, UnityEngine.Vector3.one).ToBullet());
        }

        protected void Add(PhyBody phyBody, Entity entity, UnityEngine.Vector3 pos)
        {
            Add(phyBody, entity, pos, Quaternion.identity);
        }

        protected void Add(PhyBody phyBody, Matrix m)
        {
            Add(phyBody, Entity.Null, m);
        }

        protected void Add(PhyBody phyBody, GameObject go)
        {
            Add(phyBody, ExtractModelMatrix(go));
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

                List<ComponentSystemBase> systemsToStop = new List<ComponentSystemBase>();

                systemsToStop.Add(ECS_World.DefaultGameObjectInjectionWorld.GetExistingSystem<FlipperRotateSystem>());
                systemsToStop.Add(ECS_World.DefaultGameObjectInjectionWorld.GetExistingSystem<SimulateCycleSystemGroup>());
                systemsToStop.Add(ECS_World.DefaultGameObjectInjectionWorld.GetExistingSystem<FlipperVelocitySystem>());
                foreach (var sys in systemsToStop.ToArray())
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
