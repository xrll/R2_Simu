using R2_Simu.Domain;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Data;
using System.Windows.Documents;
using System.Windows.Input;
using Media = System.Windows.Media;
using System.Windows.Media.Imaging;
using System.Windows.Navigation;
using Shaps = System.Windows.Shapes;
using MaterialDesignColors;
using MaterialDesignThemes.Wpf;
using System.Drawing.Drawing2D;
using Utilities;
using R2_Simu.Controls;
using System.Threading;
using System.Collections.ObjectModel;
using System.IO.MemoryMappedFiles;
using System.Windows.Media.Media3D;
using System.Windows.Media;
using HelixToolkit.Wpf;
using System.IO;
using System.Timers;
using System.Windows.Forms;
using System.Diagnostics;
using System.Security.Cryptography;
using Xceed.Document.NET;
using System.Drawing;
using System.Numerics;

namespace R2_Simu
{
    /// <summary>
    /// Home.xaml 的交互逻辑
    /// </summary>
    public partial class Home : System.Windows.Controls.UserControl
    {
        bool switchingJoint = false;
        public static bool isAnimating = false;

        GeometryModel3D oldSelectedModel = null;
        string basePath = "";
        ModelVisual3D visual;
        double LearningRate = 0.01;

        Transform3DGroup F1;
        Transform3DGroup F2;
        RotateTransform3D R;
        TranslateTransform3D T;

        int movements = 10;
        System.Windows.Forms.Timer timer;

        //      "T201_with_cover_v02", "V301_with_cover", "V302", "T401_with_cover", "V501_10L", "V502", "V5030" 
        Point3D center = new Point3D(); 

        public Home()
        {
            InitializeComponent();
            Loaded += Home_Loaded;
            viewPort3d.RotateGesture = new MouseGesture(MouseAction.RightClick);
            viewPort3d.PanGesture = new MouseGesture(MouseAction.LeftClick);

            //var builder = new MeshBuilder(true, true);
            //var position = new Point3D(0, 0, 0);
            //builder.AddSphere(position, 40, 20, 20);
            //geom = new GeometryModel3D(builder.ToMesh(), Materials.Brown);
            //visual = new ModelVisual3D();
            //visual.Content = geom;

            timer = new System.Windows.Forms.Timer();
            timer.Interval = 5;
            timer.Tick += new System.EventHandler(timer_Tick);
            A1 = new JointClass2((float)AD[0][0], 0, 1, 0);
            A2 = new JointClass2((float)AD[1][0], 0, 2, 0);
            Point3DCollection ps = new Point3DCollection();
            ps.Add(BeginPoint);
            ps.Add(EndPoint);
            lines.Path = ps;
            double l = Math.Sqrt((EndPoint.X - BeginPoint.X) * (EndPoint.X - BeginPoint.X) + (EndPoint.Y - BeginPoint.Y) * (EndPoint.Y - BeginPoint.Y));
            LVector = new Point3D((EndPoint.X - BeginPoint.X) / l,(EndPoint.Y - BeginPoint.Y)/l,0);
            MaxL = l * 10;
            Point3DCollection eps = new Point3DCollection();
            double st = Math.Sin(eRz / 57.29578);
            double ct = Math.Cos(eRz / 57.29578);
            double i = 0;
            while(i<Math.PI*2)
            //for(int i = 0; i<360;i++)
            {
//                double a = i / 57.29578;
                double a = i;
                double sa = Math.Sin(a);
                double ca = Math.Cos(a);
                double x = eA * ca;
                double y = eA * sa / 2;
                Point3D p = new Point3D();
                p.X = (x * ct - y * st + eTx)/10;
                p.Y = (x * st + y * ct + eTy)/10;
                p.Z = BeginPoint.Z;
                eps.Add(p);
                i += 2*step / Math.Sqrt(4 * y * y + x * x / 4);
                if (i > Math.PI * 2)
                    i = Math.PI * 2;
            }
            ellipse.Path = eps;
        }
        double eTx = -420;//椭圆中心x
        double eTy = -40;//椭圆中心y
        double eA = 300;//椭圆长半轴，短轴为1/2长轴
        double eRz = 30;//椭圆绕Z轴旋转角度
        public static Point3D LVector
        {
            set;
            get;
        }
        public static Point3D BeginPoint
        {
            set;
            get;
        } = new Point3D(50, 30, (MainWindowViewModel.JointLength[2] + MainWindowViewModel.JointLength[3]) / 10);
        public static Point3D EndPoint
        {
            set;
            get;
        } = new Point3D(30, -40, (MainWindowViewModel.JointLength[2] + MainWindowViewModel.JointLength[3]) / 10);
        public static Point3D CurPoint
        {
            set;
            get;
        } = new Point3D(50, 30, (MainWindowViewModel.JointLength[2] + MainWindowViewModel.JointLength[3]) / 10);

        double step = 1.0;
        double curPos = 0;
        double dir = 1;
        double MaxL = 0;
        double eCurAng = 0;
        public void Move()
        {
            if (MainWindowViewModel.CSimu == 0)
            {
                CurPoint = new Point3D(r2p[0], r2p[1], 0);
                if (!IsInLine(CurPoint))
                {
                    CurPoint = new Point3D(BeginPoint.X * 10, BeginPoint.Y * 10, 0);
                    curPos = 0;
                    dir = 1;
                }
                else
                {
                    curPos += step * dir;
                    if (Math.Abs(curPos) > MaxL)
                        curPos = dir * MaxL;
                    MainWindowViewModel.DPxy[0] = LVector.X * dir;
                    MainWindowViewModel.DPxy[1] = LVector.Y * dir;
                    CurPoint = new Point3D((BeginPoint.X * 10 + curPos * LVector.X), (BeginPoint.Y * 10 + curPos * LVector.Y), 0);
                    v.Point1 = new Point3D(CurPoint.X / 10, CurPoint.Y / 10, BeginPoint.Z);
                    v.Point2 = new Point3D(CurPoint.X / 10 + MainWindowViewModel.DPxy[0] * 15, CurPoint.Y / 10 + MainWindowViewModel.DPxy[1] * 15, BeginPoint.Z);
                }
                double[] CAngles = JointClass2.InverseCal(new double[] { CurPoint.X, CurPoint.Y, 0 }, AD, CurAngles);
                MainWindowViewModel.JointAngles[0] = CAngles[0];
                MainWindowViewModel.JointAngles[1] = CAngles[1];
            }
            else
            {
                double st = Math.Sin(eRz / 57.29578);
                double ct = Math.Cos(eRz / 57.29578);
                double a = eCurAng;
                double sa = Math.Sin(a);
                double ca = Math.Cos(a);
                double x = eA * ca;
                double y = eA * sa / 2;
                Point3D p = new Point3D();
                p.X = (x * ct - y * st + eTx);
                p.Y = (x * st + y * ct + eTy);
                p.Z = BeginPoint.Z;
                double m = Math.Sqrt(4 * y * y + x * x / 4);
                double tx = -2 * y / m;
                double ty = 0.5 * x / m;
                double dx = (tx * ct - ty * st);
                double dy = (tx * st + ty * ct);
                MainWindowViewModel.DPxy[0] = dx;
                MainWindowViewModel.DPxy[1] = dy;
                double[] CAngles = JointClass2.InverseCal(new double[] { p.X, p.Y, 0 }, AD, CurAngles);
                MainWindowViewModel.JointAngles[0] = CAngles[0];
                MainWindowViewModel.JointAngles[1] = CAngles[1];
                v.Point1 = new Point3D(p.X / 10, p.Y / 10, p.Z);
                v.Point2 = new Point3D(p.X / 10 + MainWindowViewModel.DPxy[0] * 15, p.Y / 10 + MainWindowViewModel.DPxy[1] * 15, p.Z);
                eCurAng += step /m ;
                if (eCurAng > Math.PI * 2)
                    eCurAng = 0;
            }
            Rot_OnRotValueChanged();
        }
        bool IsInLine(Point3D p)
        {
            double x = p.X / 10;
            double y = p.Y / 10;
            double d = (x - BeginPoint.X) * (y - EndPoint.Y) - (y - BeginPoint.Y) * (x - EndPoint.X);
            if (Math.Abs(d) < 1e-3 && y <= BeginPoint.Y + 0.0001&& y > EndPoint.Y - 0.0001)
            {
                if (y < EndPoint.Y + 0.0001)
                    dir = -1;
                else if (y > BeginPoint.Y - 0.0001)
                    dir = 1;
                return true;
            }
            else
                return false;
        }


        private void Home_Loaded(object sender, RoutedEventArgs e)
        {
            Window? window = Window.GetWindow(this);
            var mwv = this.DataContext as MainWindowViewModel;
            mwv!.mainW = this;

        }
        public static T Clamp<T>(T value, T min, T max)
            where T : System.IComparable<T>
        {
            T result = value;
            if (value.CompareTo(max) > 0)
                result = max;
            if (value.CompareTo(min) < 0)
                result = min;
            return result;
        }
        public void Simu_Click(object sender, RoutedEventArgs e)
        {
            if (timer.Enabled)
            {
                //button.Content = "Go to position";
                isAnimating = false;
                timer.Stop();
                movements = 0;
                //mainPara.IsEnabled = true;
            }
            else
            {
                //button.Content = "STOP";
                //mainPara.IsEnabled = false;
                isAnimating = true;
                timer.Start();
            }
        }

        public void timer_Tick(object sender, EventArgs e)
        {
            Move();
            //if ((--movements) <= 0)
            //{
            //    //button.Content = "Simu";
            //    isAnimating = false;
            //    //mainPara.IsEnabled = true;
            //    timer.Stop();
            //}
        }



        JointClass2 A1, A2;
        object obj = new object();
        public Vector3D ForwardKinematics(double[] angles)
        {
            lock (obj)
            {
                F1 = new Transform3DGroup();
                R = new RotateTransform3D(new AxisAngleRotation3D(new Vector3D(0, 0, 1), angles[0]), new Point3D(0, 0, 0));
                F1.Children.Add(R);

                F2 = new Transform3DGroup();
                T = new TranslateTransform3D(0, 0, 0);
                R = new RotateTransform3D(new AxisAngleRotation3D(new Vector3D(0, 0, 1), angles[1]), MainWindowViewModel.J21);
                F2.Children.Add(T);
                F2.Children.Add(R);
                F2.Children.Add(F1);

               

                v1.Transform = F1;
                v12.Transform = F1;
                v22.Transform = F2;
                v2.Transform = F2;
                


                n1.Transform = F1;
                o1.Transform = F1;
                a1.Transform = F1;

                n2.Transform = F2;
                o2.Transform = F2;
                a2.Transform = F2;

                double[] Ang = new double[2] { angles[0] / 57.29578, angles[1] / 57.29578 };

                r2p = JointClass2.R2P(AD, Ang);
                MainWindowViewModel.J2P[0] = r2p[0];
                MainWindowViewModel.J2P[1] = r2p[1];
                MainWindowViewModel.J2P[2] = r2p[2];
                r2n = JointClass2.R2N(Ang);
                MainWindowViewModel.J2N[0] = r2n[0];
                MainWindowViewModel.J2N[1] = r2n[1];
                MainWindowViewModel.J2N[2] = r2n[2];
                r2o = JointClass2.R2O(Ang);
                MainWindowViewModel.J2O[0] = r2o[0];
                MainWindowViewModel.J2O[1] = r2o[1];
                MainWindowViewModel.J2O[2] = r2o[2];
                r2a = JointClass2.R2A(Ang);
                MainWindowViewModel.J2A[0] = r2a[0];
                MainWindowViewModel.J2A[1] = r2a[1];
                MainWindowViewModel.J2A[2] = r2a[2];

                r1p = JointClass2.R1P(AD, Ang);
                MainWindowViewModel.J1P[0] = r1p[0];
                MainWindowViewModel.J1P[1] = r1p[1];
                MainWindowViewModel.J1P[2] = r1p[2];
                r1n = JointClass2.R1N(Ang);
                MainWindowViewModel.J1N[0] = r1n[0];
                MainWindowViewModel.J1N[1] = r1n[1];
                MainWindowViewModel.J1N[2] = r1n[2];
                r1o = JointClass2.R1O(Ang);
                MainWindowViewModel.J1O[0] = r1o[0];
                MainWindowViewModel.J1O[1] = r1o[1];
                MainWindowViewModel.J1O[2] = r1o[2];
                r1a = JointClass2.R1A(Ang);
                MainWindowViewModel.J1A[0] = r1a[0];
                MainWindowViewModel.J1A[1] = r1a[1];
                MainWindowViewModel.J1A[2] = r1a[2];

                iangs = JointClass2.InverseCal(r2p, AD, iangs);
                MainWindowViewModel.IJointAngles[0] = iangs[0];
                MainWindowViewModel.IJointAngles[1] = iangs[1];

                double[,] jacobi = JointClass2.Jacobian(AD, Ang);
                MainWindowViewModel.Jacobian[0] = jacobi[0, 0];
                MainWindowViewModel.Jacobian[1] = jacobi[0, 1];
                MainWindowViewModel.Jacobian[2] = jacobi[1, 0];
                MainWindowViewModel.Jacobian[3] = jacobi[1, 1];
                MainWindowViewModel.Jacobian[4] = jacobi[0, 0] * jacobi[1, 1] - jacobi[0, 1] * jacobi[1, 0];
                double[] dangle = JointClass2.Differential(jacobi, MainWindowViewModel.DPxy.ToArray());
                if (dangle != null)
                {
                    MainWindowViewModel.DAngle[0] = dangle[0];
                    MainWindowViewModel.DAngle[1] = dangle[1];
                }
                A1.CurAngle= Ang[0];
                A2.CurAngle = Ang[1];

                Matrix4x4 t1 = A1.T;
                Matrix4x4 t2 = t1 * A2.T;
            }
            return new Vector3D(0, 0,0);
        }
        double[] r2n, r2o, r2a, r2p, r1n, r1o, r1a, r1p, iangs;


        double[] CurAngles = new double[2];
        private void Rot_OnRotValueChanged()
        {
            MainWindowViewModel.JointAngles.CopyTo(CurAngles, 0);
            ForwardKinematics(CurAngles);            
        }

        public double[][] AD => new double[2][] { new double[2] { MainWindowViewModel.JointLength[0], MainWindowViewModel.JointLength[2] }, new double[2] { MainWindowViewModel.JointLength[1], MainWindowViewModel.JointLength[3] } };
    }
}
