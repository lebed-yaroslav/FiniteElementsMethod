using System.Linq;
using Avalonia;
using Avalonia.Controls;
using Avalonia.Interactivity;
using Avalonia.Markup.Xaml;
using Avalonia.Platform.Storage;
using App.ViewModels;

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

    }
}
