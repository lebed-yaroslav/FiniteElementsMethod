using System.Linq;
using App.ViewModels;
using Avalonia;
using Avalonia.Controls;
using Avalonia.Platform.Storage;
using ReactiveUI;
using ReactiveUI.Avalonia;

namespace App.Pages;

public partial class GeometryMeshPage : ReactiveUserControl<MainWindowViewModel>
{
    public GeometryMeshPage()
    {
        InitializeComponent();

        this.WhenActivated(disposables =>
        {
            if (ViewModel is null)
                return;

            var registration = ViewModel.PickMeshFileInteraction.RegisterHandler(async interaction =>
            {
                var topLevel = TopLevel.GetTopLevel(this);
                if (topLevel is null)
                {
                    interaction.SetOutput(null);
                    return;
                }

                var files = await topLevel.StorageProvider.OpenFilePickerAsync(
                    new FilePickerOpenOptions
                    {
                        Title = "Выбор файла сетки",
                        AllowMultiple = false
                    });

                interaction.SetOutput(files.FirstOrDefault()?.Path.LocalPath);
            });

            disposables.Add(registration);
        });
    }
}
