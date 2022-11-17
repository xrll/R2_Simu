using System;
using System.Collections.Generic;
using System.Collections.ObjectModel;
using System.Globalization;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Windows.Data;

namespace R2_Simu.Converters
{
    internal class Array2DValueConverter : IMultiValueConverter
    {
        #region interface implementations

        bool b1 = false;
        int index1=-1,index2 = -1;
        ObservableCollection<double[]>? a;
        public object Convert(object[] values, Type targetType, object parameter, CultureInfo culture)
        {
            a = values[1] as ObservableCollection<double[]>;
            b1 = System.Convert.ToBoolean(values[0]);
            index1 = b1 ? 0 : 1;
            index2 = System.Convert.ToInt32(parameter);
            if (a == null)
                return null;
            return a[System.Convert.ToInt32(index1)][index2];
        }

        public object[] ConvertBack(object value, Type[] targetTypes, object parameter, CultureInfo culture)
        {
            // index1 = b1 ? 0 : 1;
            index2 = System.Convert.ToInt32(parameter);
            a[index1][index2] = System.Convert.ToDouble(value);
            return new object[] { b1, a };
        }


        #endregion
    }
}
