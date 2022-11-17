using System;
using System.Globalization;
using System.Windows;
using System.Windows.Data;
using System.Windows.Media;

namespace R2_Simu.Converters
{
    public class RadioButtonCheckedConverter : IValueConverter
    {
        public object Convert(object value, Type targetType, object parameter,System.Globalization.CultureInfo culture)
        {
            
            int v = 0;
            int.TryParse(value.ToString(), out v);
            return v.Equals(System.Convert.ToInt32(parameter)); ;
        }

        public object ConvertBack(object value, Type targetType, object parameter, System.Globalization.CultureInfo culture)
        {
            return value.Equals(true) ? parameter : Binding.DoNothing;
        }
    }
}

