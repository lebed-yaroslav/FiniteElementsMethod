using System.Globalization;
using System.Linq;
using App.ViewModels;
using Avalonia;
using Avalonia.Controls;
using Avalonia.Interactivity;
using Avalonia.Markup.Xaml;
using Avalonia.Media;
using Avalonia.Platform.Storage;

namespace App
{
    public partial class MainWindow : Window
    {
        public MainWindow()
        {
            InitializeComponent();
            DataContext = new MainWindowViewModel();
        }

        private async void OnPickMeshFileClick(object? sender, RoutedEventArgs e)
        {
            if (DataContext is not MainWindowViewModel vm)
                return;

            var files = await StorageProvider.OpenFilePickerAsync(
                new FilePickerOpenOptions
                {
                    Title = "Выбор файла сетки",
                    AllowMultiple = false
                }
            );

            var file = files.FirstOrDefault();
            if (file is null)
                return;

            vm.MeshFilePath = file.Path.LocalPath;
            vm.LoadGeometryCommand.Execute().Subscribe();
        }
        public static FormattedText ConvertToAvaloniaText(string text)
        {
            return new FormattedText(
                text,
                CultureInfo.CurrentCulture, FlowDirection.LeftToRight,
                new Typeface("Times"), 20,
                null)
            {
                TextAlignment = TextAlignment.Left,                
            };
        }

        private void Button_Click(object? sender, RoutedEventArgs e)
        {
            var gdrawing = new DrawingGroup();


            gdrawing.Children.Add(new GeometryDrawing()
            {
                Brush = Brushes.Red,
                Pen = new Pen(Colors.Blue.ToUInt32()),
                Geometry = new EllipseGeometry(new Rect(100, 100, 300, 200))
                
            });
            var text = ConvertToAvaloniaText("Hello from Avalonia");
            text.SetForegroundBrush(Brushes.Green);
            gdrawing.Children.Add(new GeometryDrawing()
            {
                Brush = Brushes.Black,
                Pen = new Pen(Colors.Blue.ToUInt32()),
                Geometry = text.BuildGeometry(new Point(200,200))                        
            });



            DrawHost.Source = new DrawingImage(gdrawing);

        }
    }
}
