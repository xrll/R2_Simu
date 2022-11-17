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
using System.Windows.Media;
using System.Windows.Media.Imaging;
using System.Windows.Navigation;
using System.Windows.Shapes;

namespace R2_Simu.Controls
{
    /// <summary>
    /// JointPara.xaml 的交互逻辑
    /// </summary>
    public partial class JAngles : UserControl
    {
        public delegate void ValueChanged();
        public event ValueChanged? OnJointValueChanged;
        //double[][] aa;
        public JAngles()
        {
            InitializeComponent();
            //aa = new double[12][];
            //for (int i = 0; i < aa.Length; i++)
            //    aa[i] = new double[2];

        }
        public virtual void Method(RoutedPropertyChangedEventArgs<double> e = null)
        {
            if ((e?.OldValue != null && e?.NewValue != null) && e?.OldValue != e?.NewValue)
                OnJointValueChanged?.Invoke();
        }
        private void V_ValueChanged(object sender, RoutedPropertyChangedEventArgs<double> e)
        {
            //var a = sender as MaterialDesignThemes.Wpf.NumericUpDown;
            //double[]? b;
            //if (a.Name == "J1")
            //    b = aa[0];
            //else if (a.Name == "J2")
            //    b = aa[1];
            //else if (a.Name == "J3")
            //    b = aa[2];
            //else if (a.Name == "J4")
            //    b = aa[3];
            //else if (a.Name == "J5")
            //    b = aa[4];
            //else if (a.Name == "J6")
            //    b = aa[5];
            //else if (a.Name == "X")
            //    b = aa[6];
            //else if (a.Name == "Y")
            //    b = aa[7];
            //else if (a.Name == "Z")
            //    b = aa[8];
            //else if (a.Name == "fH")
            //    b = aa[9];
            //else if (a.Name == "btAng")
            //    b = aa[10];
            //else if (a.Name == "ov")
            //    b = aa[11];
            //else
            //    b = new double[2];

            //if (b[0] == e.OldValue && b[1] == e.NewValue)
            //    return;
            //b[0] = e.OldValue ?? 0;
            //b[1] = e.NewValue ?? 0;
            Method(e);
        }


        private void J1_ValueChanged(object sender, RoutedPropertyChangedEventArgs<double?> e)
        {
           //extCmd.Execute(((MaterialDesignThemes.Wpf.NumericUpDown)sender).Value);
        }


        //private ICommand extCmd;

        //public static readonly DependencyProperty MyCommandProperty =
        // DependencyProperty.RegisterAttached("MyCommand",
        // typeof(ICommand),
        // typeof(RPara),
        // new UIPropertyMetadata(null, OnMyCommandChanged));
        //public static ICommand GetMyCommand(DependencyObject obj) { return (ICommand)(obj.GetValue(MyCommandProperty)); }
        //public static void SetMyCommand(DependencyObject obj, ICommand value) { obj.SetValue(MyCommandProperty, value); }

        //private static void OnMyCommandChanged(DependencyObject d, DependencyPropertyChangedEventArgs e)
        //{
        //    RPara uc = d as RPara;
        //    if (uc == null) return;
        //    if (e.NewValue is ICommand) uc.extCmd = e.NewValue as ICommand;
        //}
    }
}
