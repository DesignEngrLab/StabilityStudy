using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using BulletSharp;
using BulletSharp.Math;
using GImpactTestDemo;
using OpenTK;
using TVGL;
using TVGL.IOFunctions;
//using OpenTK.Graphics;

namespace BasicDemo
{
    class Physics
    {
        ///create 125 (5x5x5) dynamic objects
        const int ArraySizeX = 5, ArraySizeY = 5, ArraySizeZ = 5;

        ///scaling of the objects (0.1 = 20 centimeter boxes )
        const float StartPosX = -5;
        const float StartPosY = -5;
        const float StartPosZ = -3;

        public DiscreteDynamicsWorld World { get; set; }
        CollisionDispatcher dispatcher;
        DbvtBroadphase broadphase;
        List<CollisionShape> collisionShapes = new List<CollisionShape>();
        CollisionConfiguration collisionConf;

        public Physics()
        {
            // collision configuration contains default setup for memory, collision setup
            collisionConf = new DefaultCollisionConfiguration();
            dispatcher = new CollisionDispatcher(collisionConf);

            broadphase = new DbvtBroadphase();
            World = new DiscreteDynamicsWorld(dispatcher, broadphase, null, collisionConf);
            World.Gravity = new BulletSharp.Math.Vector3(0, -10, 0);

            // create the ground
            CollisionShape groundShape = new BoxShape(50, 50, 50);
            collisionShapes.Add(groundShape);
            CollisionObject ground = LocalCreateRigidBody(0, Matrix.Translation(0, -50, 0), groundShape);
            ground.UserObject = "Ground";

            // create a few dynamic rigidbodies
            const float mass = 100.0f;
            //   var filename = "ABF.STL";
          //  var filename = "Beam.STL";
            var filename = "G0.STL";
            List<TessellatedSolid> ts;

            using (var fileStream = File.OpenRead(filename))
                ts = IO.Open(fileStream, filename);
            var mesh = new TriangleMesh();
            var ts0 = ts[0];
            foreach (var f in ts0.Faces)
            {

                mesh.AddTriangle(
                    new BulletSharp.Math.Vector3((float)f.Vertices[0].Position[0] , (float)f.Vertices[0].Position[1] , (float)f.Vertices[0].Position[2] ),
                    new BulletSharp.Math.Vector3((float)f.Vertices[1].Position[0] , (float)f.Vertices[1].Position[1] , (float)f.Vertices[1].Position[2] ),
                    new BulletSharp.Math.Vector3((float)f.Vertices[2].Position[0] , (float)f.Vertices[2].Position[1] , (float)f.Vertices[2].Position[2]));
            }
            //   CollisionShape colShape = new 
             // CollisionShape trimeshShape = new BoxShape(1);
            //     var trimeshShape = new BvhTriangleMeshShape(mesh, true);
          //  var trimeshShape = new GImpactMeshShape(mesh);
         var   indexVertexArrays = new TriangleIndexVertexArray(TorusMesh.Indices, TorusMesh.Vertices);
            var trimeshShape = new GImpactMeshShape(indexVertexArrays);
            //   collisionShapes.Add(trimeshShape);


            // var trimeshShape = new GImpactMeshShape(new TriangleMesh(mesh,true));
            collisionShapes.Add(trimeshShape);
            // CollisionShape trimeshShape = new BoxShape(1);
            var localInertia = trimeshShape.CalculateLocalInertia(mass);
            //  localInertia = new BulletSharp.Math. Vector3(0.6f,0.6f,0.6f);
            var rbInfo = new RigidBodyConstructionInfo(mass, null, trimeshShape, localInertia);
            const float start_x = StartPosX - ArraySizeX / 2;
            const float start_y = StartPosY;
            const float start_z = StartPosZ - ArraySizeZ / 2;

            int k, i, j;
            for (k = 0; k < 1; k++)
            {
                for (i = 0; i < 1; i++)
                {
                    for (j = 0; j < 1; j++)
                    {
                        //var startTransform = Matrix4.CreateTranslation(
                        //    new OpenTK.Vector3(
                        //        2 * i + start_x,
                        //        2 * k + start_y,
                        //        2 * j + start_z
                        //        )
                        //   );
                        var startTransform = Matrix.Translation(
                                2 * i + start_x,
                                2 * k + start_y,
                                2 * j + start_z
                                );

                        // using motionstate is recommended, it provides interpolation capabilities
                        // and only synchronizes 'active' objects
                        rbInfo.MotionState = new DefaultMotionState(startTransform);

                        RigidBody body = new RigidBody(rbInfo);

                        // make it drop from a height
                        body.Translate(new BulletSharp.Math.Vector3(0, 20, 0));

                        World.AddRigidBody(body);
                    }
                }
            }

            rbInfo.Dispose();
        }

        public virtual void Update(float elapsedTime)
        {
            World.StepSimulation(elapsedTime);
        }

        public void ExitPhysics()
        {
            //remove/dispose constraints
            int i;
            for (i = World.NumConstraints - 1; i >= 0; i--)
            {
                TypedConstraint constraint = World.GetConstraint(i);
                World.RemoveConstraint(constraint);
                constraint.Dispose();
            }

            //remove the rigidbodies from the dynamics world and delete them
            for (i = World.NumCollisionObjects - 1; i >= 0; i--)
            {
                CollisionObject obj = World.CollisionObjectArray[i];
                RigidBody body = obj as RigidBody;
                if (body != null && body.MotionState != null)
                {
                    body.MotionState.Dispose();
                }
                World.RemoveCollisionObject(obj);
                obj.Dispose();
            }

            //delete collision shapes
            foreach (CollisionShape shape in collisionShapes)
                shape.Dispose();
            collisionShapes.Clear();

            World.Dispose();
            broadphase.Dispose();
            if (dispatcher != null)
            {
                dispatcher.Dispose();
            }
            collisionConf.Dispose();
        }

        public RigidBody LocalCreateRigidBody(float mass, Matrix startTransform, CollisionShape shape)
        {
            bool isDynamic = (mass != 0.0f);

            var localInertia = BulletSharp.Math.Vector3.Zero;
            if (isDynamic)
                shape.CalculateLocalInertia(mass, out localInertia);

            DefaultMotionState myMotionState = new DefaultMotionState(startTransform);

            RigidBodyConstructionInfo rbInfo = new RigidBodyConstructionInfo(mass, myMotionState, shape, localInertia);
            RigidBody body = new RigidBody(rbInfo);

            World.AddRigidBody(body);

            return body;
        }
        //btRigidBody* BulletWrap::addTriangleMesh(Vector3 p0, Vector3 p1, Vector3 p2, Vector3 position,
        //                   bool useQuantizedBvhTree, Vector3 inertia, Scalar mass,
        //                   Scalar restitution, bool collision)
        //{
        //    TriangleMesh trimesh = new TriangleMesh();
        //    trimesh.AddTriangle(); addTriangle(p0, p1, p2);

        //    Transform trans;
        //    trans.setIdentity();
        //    trans.setOrigin(position);

        //    CollisionShape* trimeshShape = new BvhTriangleMeshShape(trimesh, useQuantizedBvhTree);
        //    trimeshShape->calculateLocalInertia(mass, inertia); //gives error

        //    DefaultMotionState* motionstate = new DefaultMotionState(trans);

        //    RigidBody* body = new RigidBody(mass, motionstate, trimeshShape, inertia);
        //    body->setRestitution(restitution);

        //    m_trianglemeshs.push_back(trimesh);
        //    m_triangleMeshBodies.push_back(body);
        //    m_world->addRigidBody(body);

        //    //return m_triangleMeshBodies.size() - 1;
        //    return body;
        //}
    }
}
