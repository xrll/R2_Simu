using System;
using System.Collections.Generic;
using System.Collections.ObjectModel;
using System.ComponentModel;
using System.IO;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Windows.Data;
using Media3D=System.Windows.Media.Media3D;
using R2_Simu;
using Newtonsoft.Json;
using Newtonsoft.Json.Linq;
using System.Threading;
using System.Windows.Media.Media3D;
using System.Windows;
using System.Windows.Media;
using System.Diagnostics;

namespace R2_Simu.Domain
{
    public class MainWindowViewModel : ViewModelBase
    {
        public ObservableCollection<double[]>? Data { get; }
        public Home? mainW
        {
            set;
            get;
        }
        public Settings? setW
        {
            set;
            get;
        }
        public MainWindowViewModel()
        {
            //_ = AsyncOpenJson();
            MenuItems = new ObservableCollection<MenuItem>(new[]
            {
                new MenuItem(
                    "Home",
                    typeof(Home),
                    this
                ),
                new MenuItem(
                    "Settings",
                    typeof(Settings),
                    this
                )
            });

            _menuItemsView = CollectionViewSource.GetDefaultView(MenuItems);
            D6030RoboCommand = new AnotherCommandImplementation(D6030Robo);
            ResetRoboCommand = new AnotherCommandImplementation(ResetRobo);
            JoggingRoboCommand = new AnotherCommandImplementation(JogRobo);

            HomeCommand = new AnotherCommandImplementation(_ => { SelectedItem = MenuItems[0]; });
            SettingsCommand = new AnotherCommandImplementation(_ => { SelectedItem = MenuItems[1]; });
            SaveSettingCommand = new AnotherCommandImplementation(_ => { SaveSetting(); });
            SelectedItem = MenuItems[SelectedIndex];
        }



        private readonly ICollectionView? _menuItemsView;
        private MenuItem? _selectedItem;
        private int _selectedIndex;
        private bool _controlsEnabled = true;

        private void JogRobo(object? o)
        {
            mainW.Move();
        }
        private void ResetRobo(object? o)
        {
            JointAngles[0] = 0;
            JointAngles[1] = 0;
        }
        private void D6030Robo(object? o)
        {
            JointAngles[0] = 60;
            JointAngles[1] = -30;
        }
        private void SaveSetting()
        {
            Properties.Settings.Default.Save();
            DialogsViewModel.ShowSaveSucsseceDiag.Execute(null);
            Process p = new Process();
            p.StartInfo.FileName = System.AppDomain.CurrentDomain.BaseDirectory + "R2_Simu.exe";
            p.StartInfo.UseShellExecute = false;
            p.Start();
            Application.Current.Shutdown();
        }
        public ObservableCollection<MenuItem> MenuItems { get; }
        public ObservableCollection<MenuItem> MainMenuItems { get; }

        public MenuItem? SelectedItem
        {
            get => _selectedItem; 
            set => SetProperty(ref _selectedItem, value);
        }

        public int SelectedIndex
        {
            get => _selectedIndex;
            set => SetProperty(ref _selectedIndex, value);
        }

        public bool ControlsEnabled
        {
            get => _controlsEnabled;
            set => SetProperty(ref _controlsEnabled, value);
        }
        public AnotherCommandImplementation HomeCommand { get; }
        public AnotherCommandImplementation SettingsCommand { get; }
        public AnotherCommandImplementation SaveSettingCommand { get; }
        public AnotherCommandImplementation? ComputeCommand { get; }
        public AnotherCommandImplementation? JoggingRoboCommand { get; }
        public AnotherCommandImplementation? ResetRoboCommand { get; }
        public AnotherCommandImplementation? D6030RoboCommand { get; }
        public AnotherCommandImplementation? ForwardCommand { get; }



        public void Init(object o)
        {
        }
        public static ObservableCollection<double[]>? RotAngles
        {
            set;
            get;
        } = new ObservableCollection<double[]>(new List<double[]>()) { new double[] { 0, 0, 0 }, new double[] { 0, 0, 0 } };
        public static ObservableCollection<double>? JointLength=> new ObservableCollection<double>(new List<double>() { Properties.Settings.Default.l1, Properties.Settings.Default.l2, Properties.Settings.Default.d1, Properties.Settings.Default.d2 });
        public static ObservableCollection<double>? JointAngles
        {
            set;
            get;
        } = new ObservableCollection<double>(new List<double>() { 60, -30 });

        public static Point3D J10
        {
            set;
            get;
        } = new Point3D(0, 0, 0);
        public static Point3D J11
        {
            set;
            get;
        }=        new Point3D(0, 0, JointLength[2] / 10);

        public static Point3D J12
        {
            set;
            get;
        } = new Point3D(JointLength[0] / 10, 0, JointLength[2]/10);
        public static Point3D J121
        {
            set;
            get;
        } = new Point3D(JointLength[0] / 10,  0, JointLength[2]/10 + JointLength[3]/20);
        public static Point3D J122
        {
            set;
            get;
        } = new Point3D(JointLength[0] / 10, 0, (JointLength[2])/10 + 4);
        public static Point3D J123
        {
            set;
            get;
        } = new Point3D(JointLength[0] / 10, 0, (JointLength[2] + JointLength[3]) / 10 - 4);



        public static Point3D J21
        {
            set;
            get;
        } = new Point3D(JointLength[0] / 10, 0, (JointLength[2] + JointLength[3]) / 10);
        public static Point3D J1n
        {
            set;
            get;
        } = new Point3D(J12.X + 15, J12.Y, J12.Z);
        public static Point3D J1o
        {
            set;
            get;
        } = new Point3D(J12.X, J12.Y + 15, J12.Z);
        public static Point3D J1a
        {
            set;
            get;
        } = new Point3D(J12.X, J12.Y, J12.Z + 15);

        public static Point3D J22
        {
            set;
            get;
        } = new Point3D((JointLength[0]+ JointLength[1]) / 10, 0, (JointLength[2]+ JointLength[3]) / 10);

        public static Point3D J2n
        {
            set;
            get;
        } = new Point3D(J22.X + 25, J22.Y, J22.Z);
        public static Point3D J2o
        {
            set;
            get;
        } = new Point3D(J22.X, J22.Y + 25, J22.Z);
        public static Point3D J2a
        {
            set;
            get;
        } = new Point3D(J22.X, J22.Y, J22.Z + 25);



        public static ObservableCollection<double>? EndP
        {
            set;
            get;
        } = new ObservableCollection<double>(new List<double>() { JointLength[0] + JointLength[1], 0, JointLength[2] + JointLength[3] });
        public static ObservableCollection<double>? EndN
        {
            set;
            get;
        } = new ObservableCollection<double>(new List<double>() { 1, 0, 0 });
        public static ObservableCollection<double>? EndO
        {
            set;
            get;
        } = new ObservableCollection<double>(new List<double>() { 0, 1, 0 });
        public static ObservableCollection<double>? EndA
        {
            set;
            get;
        } = new ObservableCollection<double>(new List<double>() { 0, 0, 1 }); 
        public static ObservableCollection<double>? DPxy
        {
            set;
            get;
        } = new ObservableCollection<double>(new List<double>() { 0, 0 });

        public static ObservableCollection<double>? DAngle
        {
            set;
            get;
        } = new ObservableCollection<double>(new List<double>() { 0, 0 });



        public static ObservableCollection<double>? J2P
        {
            set;
            get;
        } = new ObservableCollection<double>(new List<double>() { JointLength[0]+JointLength[1], 0, JointLength[2] + JointLength[3] });
        public static ObservableCollection<double>? J2N
        {
            set;
            get;
        } = new ObservableCollection<double>(new List<double>() { 0, 0, 1 });
        public static ObservableCollection<double>? J2O
        {
            set;
            get;
        } = new ObservableCollection<double>(new List<double>() { 1, 0, 0 });
        public static ObservableCollection<double>? J2A
        {
            set;
            get;
        } = new ObservableCollection<double>(new List<double>() { 0, 1, 0 });

        public static ObservableCollection<double>? J1P
        {
            set;
            get;
        } = new ObservableCollection<double>(new List<double>() { JointLength[0], 0, JointLength[2] });
        public static ObservableCollection<double>? J1N
        {
            set;
            get;
        } = new ObservableCollection<double>(new List<double>() { 1, 0, 0 });
        public static ObservableCollection<double>? J1O
        {
            set;
            get;
        } = new ObservableCollection<double>(new List<double>() { 0, 0, -1 });
        public static ObservableCollection<double>? J1A
        {
            set;
            get;
        } = new ObservableCollection<double>(new List<double>() { 0, 1, 0 });

        public static ObservableCollection<double>? IJointAngles
        {
            set;
            get;
        } = new ObservableCollection<double>(new List<double>() { 0, 0 });

        public static ObservableCollection<double>? Jacobian
        {
            set;
            get;
        } = new ObservableCollection<double>(new List<double>() { 0, 0, 0, 0, 0 }); 
        public static int CSimu
        {
            set;
            get;
        } = 0;

        public static double[][] AD=> new double[2][] { new double[2] { JointLength[0], JointLength[2] }, new double[2] { JointLength[1], JointLength[3] } };
    }
}
