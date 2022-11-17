using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Numerics;
using System.Runtime.Intrinsics;
using System.Diagnostics.Eventing.Reader;

namespace Utilities
{
    public class JointClass2
    {
        private float a;
        private float d;
        private int index = -1;
        public JointClass2()
        {
        }
        public JointClass2(float av, float ad, int i, float w)
        {
            a = av;
            d = ad;
            index = i;
            JWeight = w;
        }
        public float A
        { get { return a; } }

        public float D
        { get { return d; } }
        public double CurAngle //当前角度值
        {
            set;
            get;
        }
        public float CurPos //当前位置值
        {
            set;
            get;
        }
        public float Velocity //当前角速度值
        {
            set;
            get;
        }
        public float MaxAngle
        {
            set;
            get;
        }
        public float MinAngle
        {
            set;
            get;
        }
        public Matrix4x4 T //位置矩阵
        {
            get
            {
                float b = (float)CurAngle;
                float sb = (float)Math.Sin(b);
                float cb = (float)Math.Cos(b);
                Matrix4x4 ansMat = new Matrix4x4();
                switch (index)
                {
                    case 1:
                        ansMat = new Matrix4x4(cb,  -sb,0, a * cb, sb, cb,0,  a * sb, 0, 0, 1, d, 0, 0, 0, 1);
                        break;
                    case 2:
                        ansMat = new Matrix4x4(cb, -sb, 0, a * cb, sb, cb, 0, a * sb, 0, 0, 1, d, 0, 0, 0, 1);
                        break;
                    default:
                        break;
                }
                return ansMat;
            }
        }


        public Matrix4x4 T1(float a) //一阶导数矩阵
        {
            return new Matrix4x4();
        }
        public float JWeight
        {
            set;
            get;
        }
        /// <summary>
        /// 关节末端的空间姿态-位置向量[px,py,pz]
        /// </summary>
        /// <param name="AD">每个关节的a,d值构成的数组列表，如GSK RB8：{{150,0},{560,0},{155,0},{0,630},{0,0},{0,155}}</param>
        /// <param name="Ang">每个关节的角度值</param>
        /// <returns>[px,py,pz]</returns>
        public static double[] R2P(double[][] AD, double[] Ang)
        {
            double A1 = AD[0][0];
            double D1 = AD[0][1];
            double A2 = AD[1][0];
            double D2 = AD[0][1];

            double C1 = Math.Cos(Ang[0]);
            double S1 = Math.Sin(Ang[0]);
            double C2 = Math.Cos(Ang[1]);
            double S2 = Math.Sin(Ang[1]);

            double S12 = Math.Sin(Ang[1] + Ang[0]);
            double C12 = Math.Cos(Ang[1] + Ang[0]);

            double px = C12 * A2 + C1 * A1;
            double py = S12 * A2 + S1 * A1;
            double pz = D2 + D1;
            return new double[] { px, py, pz };
        }
        /// <summary>
        /// 关节末端的空间姿态-n向量[nx,ny,nz]
        /// </summary>
        /// <param name="Ang">每个关节的角度值</param>
        /// <returns>[nx,ny,nz]</returns>
        public static double[] R2N(double[] Ang)
        {
            double S12 = Math.Sin(Ang[1] + Ang[0]);
            double C12 = Math.Cos(Ang[1] + Ang[0]);
            return new double[] { C12, S12, 0 };
        }
        /// <summary>
        /// 关节末端的空间姿态-o向量[ox,oy,oz]
        /// </summary>
        /// <param name="Ang">每个关节的角度值</param>
        /// <returns>[ox,oy,oz]</returns>
        public static double[] R2O(double[] Ang)
        {
            double S12 = Math.Sin(Ang[1] + Ang[0]);
            double C12 = Math.Cos(Ang[1] + Ang[0]);
            return new double[] { -S12, C12, 0 };
        }
        /// <summary>
        /// 关节末端的空间姿态-a向量[ax,ay,az]
        /// </summary>
        /// <param name="Ang">每个关节的角度值</param>
        /// <returns>[ax,ay,az]</returns>
        public static double[] R2A(double[] Ang)
        {
            return new double[] { 0, 0, 1 };
        }
        public static double[] R1P(double[][] AD, double[] Ang)
        {
            double A1 = AD[0][0];
            double D1 = AD[0][1];
            double C1 = Math.Cos(Ang[0]);
            double S1 = Math.Sin(Ang[0]);
            double px = A1 * C1;
            double py =  A1 * S1;
            double pz = D1;
            return new double[] { px, py, pz };
        }
        public static double[] R1N(double[] Ang)
        {
            double C1 = Math.Cos(Ang[0]);
            double S1 = Math.Sin(Ang[0]);
            return new double[] { C1, S1, 0 };
        }
        public static double[] R1O(double[] Ang)
        {
            double C1 = Math.Cos(Ang[0]);
            double S1 = Math.Sin(Ang[0]);
            return new double[] { -S1, C1, 0 };
        }
        public static double[] R1A(double[] Ang)
        {
            return new double[] { 0, 0, 1 };
        }



        static double eps = 1e-4;
        /// <summary>
        /// 两关节机器人逆运算
        /// </summary>
        /// <param name="r2p">末端位置</param>
        /// <param name="AD">每个关节的a,d值构成的数组列表，如GSK RB8：{{150,0},{560,0},{155,0},{0,630},{0,0},{0,155}}</param>
        /// <returns>两关节角度数组</returns>
        public static double[] InverseCal(double[] r2p, double[][] AD, double[] lAngs)
        {
            double Ө1=0, Ө2=0;

            if (lAngs == null)
                lAngs = new double[] { 0, 0};
            double A1 = AD[0][0];
            double D1 = AD[0][1];
            double A2 = AD[1][0];
            double D2 = AD[1][1];

            double tmp1 = Math.Atan2(r2p[1], r2p[0]);
            if (tmp1 < 0)
                tmp1 += Math.PI;
            if (r2p[1] < 0)
                tmp1 += Math.PI;



            double l12xy = r2p[0] * r2p[0] + r2p[1] * r2p[1];
            double l12 = Math.Sqrt(l12xy);

            double t21 = Math.Acos((A1 * A1 + l12xy - A2 * A2) / (2 * A1 * l12));//余弦定理
            double t22 = Math.Acos((A1 * A1 + A2 * A2 - l12xy) / (2 * A1 * A2));//余弦定理

            Ө1 = t21 + tmp1;
            double S1 = Math.Sin(Ө1);
            double C1 = Math.Cos(Ө1);
            double n2x = (r2p[0] - A1 * C1) / A2;
            double n2y = (r2p[1] - A1 * S1) / A2;

            double Ө11 = Ө1 + 2 * Math.PI;
            double Ө12 = Ө1 - 2 * Math.PI;
            double d1 = Math.Abs(Ө1 - lAngs[0] / 57.2958);
            double d2 = Math.Abs(Ө11 - lAngs[0] / 57.2958);
            double d3 = Math.Abs(Ө12 - lAngs[0] / 57.2958);

            if(d2<d1&&d2<d3)
                Ө1 = Ө11;
            else if(d3<d1&&d3<d2)
                Ө1 = Ө12;

            Ө2 = Math.Atan2(-n2x * S1 + n2y * C1, n2x * C1 + n2y * S1);
            if (Ө2 > 0)
                Ө2 -= Math.PI;

            return new double[] { Ө1 * 57.29578f, Ө2 * 57.29578f };
        }
        /// <summary>
        /// 两关节雅可比矩阵
        /// </summary>
        /// <param name="AD">每个关节的a,d值构成的数组列表，如GSK RB8：{{150,0},{560,0},{155,0},{0,630},{0,0},{0,155}}</param>
        /// <param name="Ang">每个关节的角度值数组</param>
        /// <returns>6X6矩阵</returns>
        public static double[,] Jacobian(double[][] AD, double[] Ang)
        {
            double A1 = AD[0][0];
            double D1 = AD[0][1];
            double A2 = AD[1][0];
            double D4 = AD[1][1];

            double C1 = Math.Cos(Ang[0]);
            double S1 = Math.Sin(Ang[0]);

            double S12 = Math.Sin(Ang[1] + Ang[0]);
            double C12 = Math.Cos(Ang[1] + Ang[0]);

            double J11 = -A1 * S1 - A2 * S12;
            double J12 = -A2 * S12;

            double J21 = A1 * C1 + A2 * C12;
            double J22 = A2 * C12;

            double[,] Mat = new double[,] { { J11, J12 }, { J21, J22} };
            return Mat;
        }

        /// <summary>
        /// 两关节机器人关节微分运算
        /// </summary>
        /// <param name="AD">每个关节的a,d值构成的数组列表，如GSK RB8：{{150,0},{560,0},{155,0},{0,630},{0,0},{0,155}}</param>
        /// <param name="Ang">每个关节的角度值</param>
        /// <param name="diffMat">末端空间位置姿态微分数组[dpx,dpy,dpz,δx,δy,δz]</param>
        /// <returns>两关节角度微分数组[dӨ1,dӨ2,dӨ3,dӨ4,dӨ5,dӨ6]</returns>
        public static double[] Differential(double[][] AD, double[] Ang, double[] diffMat)
        {
            double[] A = diffMat;
            double[,] JacobianMat = Jacobian(AD, Ang);
            double det = JacobianMat[0, 0] * JacobianMat[1, 1] - JacobianMat[0, 1] * JacobianMat[1, 0];
            if (det == 0)
                return null;
            double dt1 = JacobianMat[1, 1] * diffMat[0] - JacobianMat[1, 0] * diffMat[1];
            double dt2 = -JacobianMat[0, 1] * diffMat[0] + JacobianMat[0, 0] * diffMat[1];
            return new double[] { dt1 / det * 57.29578, dt2 / det * 57.29578 };
        }

        /// <summary>
        /// 两关节机器人关节微分运算
        /// </summary>
        /// <param name="AD">每个关节的a,d值构成的数组列表，如GSK RB8：{{150,0},{560,0},{155,0},{0,630},{0,0},{0,155}}</param>
        /// <param name="Ang">每个关节的角度值</param>
        /// <param name="diffMat">末端空间位置姿态微分数组[dpx,dpy,dpz,δx,δy,δz]</param>
        /// <returns>两关节角度微分数组[dӨ1,dӨ2,dӨ3,dӨ4,dӨ5,dӨ6]</returns>
        public static double[] Differential(double[,] JacobianMat, double[] dpxy)
        {
            double[] A = dpxy;
            double det = JacobianMat[0, 0] * JacobianMat[1, 1] - JacobianMat[0, 1] * JacobianMat[1, 0];
            if (det == 0)
                return null;
            double dt1 = JacobianMat[1, 1] * dpxy[0] - JacobianMat[0, 1] * dpxy[1];
            double dt2 = -JacobianMat[1, 0] * dpxy[0] + JacobianMat[0, 0] * dpxy[1];
            return new double[] { dt1 / det*57.29578, dt2 / det * 57.29578 };
        }
    }
}
