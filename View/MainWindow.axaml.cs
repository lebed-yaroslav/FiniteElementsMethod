using System.Threading.Tasks;
using System.Reactive;
using App.Pages;
using App.ViewModels;
using Avalonia.Controls;
using Dock.Avalonia.Controls;
using Dock.Model.Avalonia;
using Dock.Model.Avalonia.Controls;
using Dock.Model.Controls;
using Dock.Model.Core;
using ReactiveUI;
using ReactiveUI.Avalonia;

namespace App;

public partial class MainWindow : ReactiveWindow<MainWindowViewModel>
{
    private readonly Factory _dockFactory = new();

    public MainWindow()
    {
        InitializeComponent();

        _dockFactory.HostWindowLocator = new Dictionary<string, Func<IHostWindow?>>
        {
            [nameof(IDockWindow)] = () => new HostWindow()
        };
        _dockFactory.DefaultHostWindowLocator = () => new HostWindow();

        if (DataContext is MainWindowViewModel vm)
            ViewModel = vm;

        this.WhenActivated(disposables =>
        {
            if (ViewModel is null)
            {
                ViewModel = new MainWindowViewModel();
                DataContext = ViewModel;
            }

            InitializeWorkspace();

            var registration = ViewModel.ResetWorkspaceInteraction.RegisterHandler(context =>
            {
                WorkspaceDock.Layout = CreateWorkspaceLayout();
                context.SetOutput(Unit.Default);
                return Task.CompletedTask;
            });

            disposables.Add(registration);
        });
    }

    private void InitializeWorkspace()
    {
        WorkspaceDock.Factory = _dockFactory;
        WorkspaceDock.Layout = CreateWorkspaceLayout();
    }

    private object CreatePageContent<T>() where T : Control, new()
    {
        return new Func<IServiceProvider, object>(_ =>
        {
            var page = new T();
            if (page is IViewFor<MainWindowViewModel> viewFor)
                viewFor.ViewModel = ViewModel;
            else
                page.DataContext = ViewModel;
            return page;
        });
    }

    private IRootDock CreateWorkspaceLayout()
    {
        var geometryDocument = new Document
        {
            Id = "GeometryMesh",
            Title = "Геометрия и сетка",
            Content = CreatePageContent<GeometryMeshPage>(),
            CanFloat = true
        };
        var parametersDocument = new Document
        {
            Id = "Parameters",
            Title = "Параметры задачи",
            Content = CreatePageContent<ParametersPage>(),
            CanFloat = true
        };
        var solutionDocument = new Document
        {
            Id = "Solution",
            Title = "Решение",
            Content = CreatePageContent<SolutionPage>(),
            CanFloat = true
        };
        var resultsDocument = new Document
        {
            Id = "ResultsLogs",
            Title = "Результаты и логи",
            Content = CreatePageContent<ResultsPage>(),
            CanFloat = true
        };

        var documentDock = new DocumentDock
        {
            Id = "Documents",
            Title = "Рабочая область",
            VisibleDockables = _dockFactory.CreateList<IDockable>(
                geometryDocument,
                parametersDocument,
                solutionDocument,
                resultsDocument),
            ActiveDockable = geometryDocument,
            DefaultDockable = geometryDocument,
            CanCloseLastDockable = false
        };

        var root = _dockFactory.CreateRootDock();
        root.Id = "Root";
        root.VisibleDockables = _dockFactory.CreateList<IDockable>(documentDock);
        root.ActiveDockable = documentDock;
        root.DefaultDockable = documentDock;
        root.FloatingWindowHostMode = DockFloatingWindowHostMode.Native;

        _dockFactory.InitLayout(root);
        return root;
    }
}
