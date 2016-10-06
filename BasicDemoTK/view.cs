//using System;
//using System.Collections.Generic;
//using System.Linq;
//using System.Text;
//using System.Threading.Tasks;
//using OpenTK;
//using System.Drawing;
//using OpenTK.Graphics.ES10;

//namespace BasicDemo
//{
//    class view
//    {
//        public OpenTK.Vector2 postion;
//        public double rotation;
//        public double zoom;

//        public view(Vector2 startposition, double startzoom = 1, double startrotation = 0)
//        {
//            this.postion = startposition;
//            this.rotation = startrotation;
//            this.zoom = startzoom;
//        }

//        public void updata()
//        {
//        }

//        public void applytransform()
//        {
//            Matrix4 transform = Matrix4.Identity;
//            transform = Matrix4.Mult(transform, Matrix4.CreateTranslation(-postion.X, -postion.Y, (float)0));
//            transform = Matrix4.Mult(transform, Matrix4.CreateRotationZ(-(float)rotation));
//            transform = Matrix4.Mult(transform, Matrix4.CreateScale((float) zoom, (float)zoom, 1.0f));
//            GL.MultMatrix()
//        }
//    }
//}
