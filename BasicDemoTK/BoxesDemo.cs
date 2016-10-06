//using System.Collections.Generic;
//using BulletSharp;
//using BulletSharp.Math;
//using OpenTK;
//using Vector3 = OpenTK.Vector3;

////using OpenTK.Graphics;

//namespace BasicDemo
//{
//    class Physics
//    {
//        ///create 125 (5x5x5) dynamic objects
//        const int ArraySizeX = 5, ArraySizeY = 5, ArraySizeZ = 5;

//        ///scaling of the objects (0.1 = 20 centimeter boxes )
//        const float StartPosX = -5;
//        const float StartPosY = -5;
//        const float StartPosZ = -3;

//        public DiscreteDynamicsWorld World { get; set; }
//        CollisionDispatcher dispatcher;
//        DbvtBroadphase broadphase;
//        List<CollisionShape> collisionShapes = new List<CollisionShape>();
//        CollisionConfiguration collisionConf;

//        public Physics()
//        {
//            // collision configuration contains default setup for memory, collision setup
//            collisionConf = new DefaultCollisionConfiguration();
//            dispatcher = new CollisionDispatcher(collisionConf);

//            broadphase = new DbvtBroadphase();
//            World = new DiscreteDynamicsWorld(dispatcher, broadphase, null, collisionConf);
//            World.Gravity = new BulletSharp.Math.Vector3(0, -10, 0);

//            // create the ground
//            CollisionShape groundShape = new BoxShape(50, 50, 50);
//            collisionShapes.Add(groundShape);
//            CollisionObject ground = LocalCreateRigidBody(0, BulletSharp.Math.Matrix.Translation(0, -50, 0), groundShape);
//            ground.UserObject = "Ground";

//            // create a few dynamic rigidbodies
//            const float mass = 1.0f;

//            CollisionShape colShape = new BoxShape(1);
//            collisionShapes.Add(colShape);
//            var localInertia = colShape.CalculateLocalInertia(mass);

//            var rbInfo = new RigidBodyConstructionInfo(mass, null, colShape, localInertia);

//            const float start_x = StartPosX - ArraySizeX / 2;
//            const float start_y = StartPosY;
//            const float start_z = StartPosZ - ArraySizeZ / 2;

//            int k, i, j;
//            for (k = 0; k < 1; k++)
//            {
//                for (i = 0; i < 1; i++)
//                {
//                    for (j = 0; j < 1; j++)
//                    {
//                        var startTransform = Matrix.Translation(
//                            new BulletSharp.Math.Vector3(
//                                2 * i + start_x,
//                                2 * k + start_y,
//                                2 * j + start_z
//                                )
//                            );

//                        // using motionstate is recommended, it provides interpolation capabilities
//                        // and only synchronizes 'active' objects
//                        rbInfo.MotionState = new DefaultMotionState(startTransform);

//                        RigidBody body = new RigidBody(rbInfo);

//                        // make it drop from a height
//                        body.Translate(new BulletSharp.Math.Vector3(0, 20, 0));

//                        World.AddRigidBody(body);
//                    }
//                }
//            }

//            rbInfo.Dispose();
//        }

//        public virtual void Update(float elapsedTime)
//        {
//            World.StepSimulation(elapsedTime);
//        }

//        public void ExitPhysics()
//        {
//            //remove/dispose constraints
//            int i;
//            for (i = World.NumConstraints - 1; i >= 0; i--)
//            {
//                TypedConstraint constraint = World.GetConstraint(i);
//                World.RemoveConstraint(constraint);
//                constraint.Dispose();
//            }

//            //remove the rigidbodies from the dynamics world and delete them
//            for (i = World.NumCollisionObjects - 1; i >= 0; i--)
//            {
//                CollisionObject obj = World.CollisionObjectArray[i];
//                RigidBody body = obj as RigidBody;
//                if (body != null && body.MotionState != null)
//                {
//                    body.MotionState.Dispose();
//                }
//                World.RemoveCollisionObject(obj);
//                obj.Dispose();
//            }

//            //delete collision shapes
//            foreach (CollisionShape shape in collisionShapes)
//                shape.Dispose();
//            collisionShapes.Clear();

//            World.Dispose();
//            broadphase.Dispose();
//            if (dispatcher != null)
//            {
//                dispatcher.Dispose();
//            }
//            collisionConf.Dispose();
//        }

//        public RigidBody LocalCreateRigidBody(float mass, Matrix startTransform, CollisionShape shape)
//        {
//            bool isDynamic = (mass != 0.0f);

//            var localInertia = BulletSharp.Math.Vector3.Zero;
//            if (isDynamic)
//                shape.CalculateLocalInertia(mass, out localInertia);

//            DefaultMotionState myMotionState = new DefaultMotionState(startTransform);

//            RigidBodyConstructionInfo rbInfo = new RigidBodyConstructionInfo(mass, myMotionState, shape, localInertia);
//            RigidBody body = new RigidBody(rbInfo);

//            World.AddRigidBody(body);

//            return body;
//        }
//    }
//}
