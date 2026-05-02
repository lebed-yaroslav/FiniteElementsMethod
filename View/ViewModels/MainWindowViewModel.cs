using System.Collections.ObjectModel;
using System.Diagnostics;
using System.Globalization;
using System.Reactive;
using System.Reactive.Linq;
using System.Text;
using System.Text.Json;
using System.Text.RegularExpressions;
using Model.Core.CoordinateSystem;
using Model.Core.Matrix;
using Model.Core.Solver;
using Model.Core.Solver.Precondition;
using Model.Fem.Elements;
using Model.Fem.Integrator;
using Model.Fem.Mesh;
using Model.Fem.Problem;
using ReactiveUI;
using Telma;

namespace App.ViewModels;

public class MainWindowViewModel : ViewModelBase
{
    private const string ElementTriLinear = "Triangle.Linear";
    private const string ElementTriHierarchicalQuadratic = "Triangle.HierarchicalQuadratic";
    private const string ElementTriHierarchicalCubic = "Triangle.HierarchicalCubic";
    private const string ElementTriLagrangeCubic = "Triangle.LagrangeCubic";
    private const string ElementQuadBilinear = "Quadrangle.Bilinear";
    private const string ElementQuadLagrangeQuadratic = "Quadrangle.LagrangeQuadratic";
    private const string ElementQuadLagrangeCubic = "Quadrangle.LagrangeCubic";
    private const string ElementQuadHermite = "Quadrangle.Hermite";
    private const string TsExplicitTwoLayer = "Явная двухслойная";
    private const string TsImplicitTwoLayer = "Неявная двухслойная";
    private const string TsExplicitThreeLayer = "Явная трехслойная";
    private const string TsImplicitThreeLayer = "Неявная трехслойная";

    private CancellationTokenSource? _calculationCts;
    private StationarySolution<Vector2D, Vector1D>? _lastSolution;
    private double _lastTime;
    private CalculationRequest? _lastRequest;
    private List<TimeLayerSolution> _lastTimeLayers = [];
    private List<LayerErrorMetrics> _lastLayerMetrics = [];
    private List<SolverStepMetrics> _lastSolverSteps = [];
    private static readonly JsonSerializerOptions JsonOptions = new() { WriteIndented = true };

    private static readonly IReadOnlyDictionary<string, (IFiniteElementFactory2D Volume, IBoundaryElementFactory2D Boundary)> ElementFactoryMap
        = new Dictionary<string, (IFiniteElementFactory2D, IBoundaryElementFactory2D)>(StringComparer.Ordinal)
        {
            [ElementTriLinear] = (FiniteElements.Triangle.Linear, FiniteElements.Segment.Linear),
            [ElementTriHierarchicalQuadratic] = (FiniteElements.Triangle.HierarchicalQuadratic, FiniteElements.Segment.HierarchicalQuadratic),
            [ElementTriHierarchicalCubic] = (FiniteElements.Triangle.HierarchicalCubic, FiniteElements.Segment.LagrangeCubic),
            [ElementTriLagrangeCubic] = (FiniteElements.Triangle.LagrangeCubic, FiniteElements.Segment.LagrangeCubic),
            [ElementQuadBilinear] = (FiniteElements.Quadrangle.Bilinear, FiniteElements.Segment.Linear),
            [ElementQuadLagrangeQuadratic] = (FiniteElements.Quadrangle.LagrangeQuadratic, FiniteElements.Segment.LagrangeQuadratic),
            [ElementQuadLagrangeCubic] = (FiniteElements.Quadrangle.LagrangeCubic, FiniteElements.Segment.LagrangeCubic),
            [ElementQuadHermite] = (FiniteElements.Quadrangle.Hermite, FiniteElements.Segment.Hermite)
        };

    public ObservableCollection<string> ElementTypes { get; } =
    [
        ElementQuadBilinear,
        ElementQuadLagrangeQuadratic,
        ElementQuadLagrangeCubic,
        ElementQuadHermite,
        ElementTriLinear,
        ElementTriHierarchicalQuadratic,
        ElementTriHierarchicalCubic,
        ElementTriLagrangeCubic
    ];
    public ObservableCollection<string> DiffusionCoefficients { get; }
    public ObservableCollection<string> SigmaCoefficients { get; }
    public ObservableCollection<string> SourceFunctions { get; }
    public ObservableCollection<string> BoundaryFunctions { get; }
    public ObservableCollection<string> SolverMethods { get; } = ["PCG", "PCG+ILU", "Pardiso"];

    public ObservableCollection<string> TimeSchemes { get; } =
    [
        TsExplicitTwoLayer,
        TsImplicitTwoLayer,
        TsExplicitThreeLayer,
        TsImplicitThreeLayer
    ];

    private string _statusMessage = "Готов к работе";
    public string StatusMessage
    {
        get => _statusMessage;
        set => this.RaiseAndSetIfChanged(ref _statusMessage, value);
    }

    private string _selectedElementType = ElementQuadBilinear;
    public string SelectedElementType
    {
        get => _selectedElementType;
        set => this.RaiseAndSetIfChanged(ref _selectedElementType, value);
    }

    private string _meshFilePath = string.Empty;
    public string MeshFilePath
    {
        get => _meshFilePath;
        set => this.RaiseAndSetIfChanged(ref _meshFilePath, value);
    }

    private string _meshText = string.Empty;
    public string MeshText
    {
        get => _meshText;
        set => this.RaiseAndSetIfChanged(ref _meshText, value);
    }

    private string _meshStatus = "Режим: автоматическая сетка";
    public string MeshStatus
    {
        get => _meshStatus;
        set => this.RaiseAndSetIfChanged(ref _meshStatus, value);
    }

    private string _selectedDiffusionCoefficient = "1.0";
    public string SelectedDiffusionCoefficient
    {
        get => _selectedDiffusionCoefficient;
        set => this.RaiseAndSetIfChanged(ref _selectedDiffusionCoefficient, value);
    }

    private string _selectedXiCoefficient = "0.0";
    public string SelectedXiCoefficient
    {
        get => _selectedXiCoefficient;
        set => this.RaiseAndSetIfChanged(ref _selectedXiCoefficient, value);
    }

    private string _selectedSigmaCoefficient = "1.0";
    public string SelectedSigmaCoefficient
    {
        get => _selectedSigmaCoefficient;
        set => this.RaiseAndSetIfChanged(ref _selectedSigmaCoefficient, value);
    }

    private string _selectedSourceFunction = "0";
    public string SelectedSourceFunction
    {
        get => _selectedSourceFunction;
        set => this.RaiseAndSetIfChanged(ref _selectedSourceFunction, value);
    }

    private string _selectedBoundaryFunction = "x * t";
    public string SelectedBoundaryFunction
    {
        get => _selectedBoundaryFunction;
        set => this.RaiseAndSetIfChanged(ref _selectedBoundaryFunction, value);
    }

    private string _selectedSolverMethod = "PCG";
    public string SelectedSolverMethod
    {
        get => _selectedSolverMethod;
        set => this.RaiseAndSetIfChanged(ref _selectedSolverMethod, value);
    }

    private string _selectedPreconditioner = "Identity";
    public string SelectedPreconditioner
    {
        get => _selectedPreconditioner;
        set => this.RaiseAndSetIfChanged(ref _selectedPreconditioner, value);
    }

    private string _tolerance = "1e-10";
    public string Tolerance
    {
        get => _tolerance;
        set => this.RaiseAndSetIfChanged(ref _tolerance, value);
    }

    private string _timeStart = "0";
    public string TimeStart
    {
        get => _timeStart;
        set => this.RaiseAndSetIfChanged(ref _timeStart, value);
    }

    private string _timeStep = "0.1";
    public string TimeStep
    {
        get => _timeStep;
        set => this.RaiseAndSetIfChanged(ref _timeStep, value);
    }

    private string _timeLayerCount = "5";
    public string TimeLayerCount
    {
        get => _timeLayerCount;
        set => this.RaiseAndSetIfChanged(ref _timeLayerCount, value);
    }

    private string _timePointsText = "0.0, 0.25, 0.5, 0.75, 1.0";
    public string TimePointsText
    {
        get => _timePointsText;
        set => this.RaiseAndSetIfChanged(ref _timePointsText, value);
    }

    private string _experimentsRootPath = Path.Combine(Environment.CurrentDirectory, "experiments");
    public string ExperimentsRootPath
    {
        get => _experimentsRootPath;
        set => this.RaiseAndSetIfChanged(ref _experimentsRootPath, value);
    }

    private string _meshDiagnosticsText = string.Empty;
    public string MeshDiagnosticsText
    {
        get => _meshDiagnosticsText;
        set => this.RaiseAndSetIfChanged(ref _meshDiagnosticsText, value);
    }

    private string _validationSummary = string.Empty;
    public string ValidationSummary
    {
        get => _validationSummary;
        set => this.RaiseAndSetIfChanged(ref _validationSummary, value);
    }

    private string _executionLog = string.Empty;
    public string ExecutionLog
    {
        get => _executionLog;
        set => this.RaiseAndSetIfChanged(ref _executionLog, value);
    }

    private string _selectedTimeScheme = TsImplicitThreeLayer;
    public string SelectedTimeScheme
    {
        get => _selectedTimeScheme;
        set => this.RaiseAndSetIfChanged(ref _selectedTimeScheme, value);
    }

    private double _calculationProgress;
    public double CalculationProgress
    {
        get => _calculationProgress;
        set => this.RaiseAndSetIfChanged(ref _calculationProgress, value);
    }

    private string _calculationStatus = "Ожидание";
    public string CalculationStatus
    {
        get => _calculationStatus;
        set => this.RaiseAndSetIfChanged(ref _calculationStatus, value);
    }

    private string _solutionPointsText = string.Empty;
    public string SolutionPointsText
    {
        get => _solutionPointsText;
        set => this.RaiseAndSetIfChanged(ref _solutionPointsText, value);
    }

    private bool _isCalculationRunning;
    public bool IsCalculationRunning
    {
        get => _isCalculationRunning;
        set => this.RaiseAndSetIfChanged(ref _isCalculationRunning, value);
    }

    public ReactiveCommand<Unit, Unit> NewProjectCommand { get; private set; } = null!;
    public ReactiveCommand<Unit, Unit> LoadProjectCommand { get; private set; } = null!;
    public ReactiveCommand<Unit, Unit> SaveProjectCommand { get; private set; } = null!;
    public ReactiveCommand<Unit, Unit> ExitCommand { get; private set; } = null!;
    public ReactiveCommand<Unit, Unit> CalculateCommand { get; private set; } = null!;
    public ReactiveCommand<Unit, Unit> StopCalculationCommand { get; private set; } = null!;
    public ReactiveCommand<Unit, Unit> AboutCommand { get; private set; } = null!;
    public ReactiveCommand<Unit, Unit> SaveGeometryCommand { get; private set; } = null!;
    public ReactiveCommand<Unit, Unit> LoadGeometryCommand { get; private set; } = null!;
    public ReactiveCommand<Unit, Unit> ResetGeometryCommand { get; private set; } = null!;
    public ReactiveCommand<Unit, Unit> SaveMeshTextCommand { get; private set; } = null!;
    public ReactiveCommand<Unit, Unit> ClearMeshTextCommand { get; private set; } = null!;
    public ReactiveCommand<Unit, Unit> SaveParametersCommand { get; private set; } = null!;
    public ReactiveCommand<Unit, Unit> LoadParametersCommand { get; private set; } = null!;
    public ReactiveCommand<Unit, Unit> ResetParametersCommand { get; private set; } = null!;
    public ReactiveCommand<Unit, Unit> ValidateMeshCommand { get; private set; } = null!;
    public ReactiveCommand<Unit, Unit> LoadMeshTemplateCommand { get; private set; } = null!;
    public ReactiveCommand<Unit, Unit> ResetResultsCommand { get; private set; } = null!;
    public ReactiveCommand<Unit, Unit> ExportToCsvCommand { get; private set; } = null!;
    public ReactiveCommand<Unit, Unit> ExportToPngCommand { get; private set; } = null!;
    public ReactiveCommand<Unit, Unit> ClearLogCommand { get; private set; } = null!;

    public MainWindowViewModel()
    {
        var canCalculate = this.WhenAnyValue(vm => vm.IsCalculationRunning).Select(x => !x);
        var canStop = this.WhenAnyValue(vm => vm.IsCalculationRunning);

        NewProjectCommand = ReactiveCommand.Create(NewProject);
        LoadProjectCommand = ReactiveCommand.Create(() => { StatusMessage = "Загрузка проекта пока не реализована"; });
        SaveProjectCommand = ReactiveCommand.Create(() => { StatusMessage = "Сохранение проекта пока не реализовано"; });
        ExitCommand = ReactiveCommand.Create(() => Environment.Exit(0));
        CalculateCommand = ReactiveCommand.CreateFromTask(RunCalculationAsync, canCalculate);
        StopCalculationCommand = ReactiveCommand.Create(StopCalculation, canStop);
        AboutCommand = ReactiveCommand.Create(() => { StatusMessage = "Finite Elements Method"; });

        SaveGeometryCommand = ReactiveCommand.Create(SaveGeometry);
        LoadGeometryCommand = ReactiveCommand.Create(LoadGeometry);
        ResetGeometryCommand = ReactiveCommand.Create(ResetGeometry);
        SaveMeshTextCommand = ReactiveCommand.Create(SaveMeshText);
        ClearMeshTextCommand = ReactiveCommand.Create(ClearMeshText);

        SaveParametersCommand = ReactiveCommand.Create(() => { StatusMessage = "Параметры задачи применены"; });
        LoadParametersCommand = ReactiveCommand.Create(() => { StatusMessage = "Загрузка параметров пока не реализована"; });
        ResetParametersCommand = ReactiveCommand.Create(ResetParameters);

        ResetResultsCommand = ReactiveCommand.Create(ResetResults);
        ExportToCsvCommand = ReactiveCommand.Create(ExportToCsv);
        ExportToPngCommand = ReactiveCommand.Create(() => { StatusMessage = "Экспорт PNG пока не реализован"; });
        ApplyImplementedHandlers();
        AppendLog("Готово к работе.");
    }

    private void ApplyImplementedHandlers()
    {
        var canCalculate = this.WhenAnyValue(vm => vm.IsCalculationRunning).Select(x => !x);
        CalculateCommand = ReactiveCommand.CreateFromTask(RunCalculationWorkflowAsync, canCalculate);
        ValidateMeshCommand = ReactiveCommand.Create(ValidateMesh);
        LoadMeshTemplateCommand = ReactiveCommand.Create(LoadMeshTemplate);
        ExportToPngCommand = ReactiveCommand.Create(ExportToPng);
        ClearLogCommand = ReactiveCommand.Create(() => { ExecutionLog = string.Empty; });
    }

    private void NewProject()
    {
        ResetGeometry();
        ResetParameters();
        ResetResults();
        StatusMessage = "Создан новый проект";
    }

    private void ResetGeometry()
    {
        SelectedElementType = ElementQuadBilinear;
        MeshFilePath = string.Empty;
        MeshText = string.Empty;
        MeshStatus = "Задайте сетку вручную или загрузите из файла";
        StatusMessage = "Геометрия сброшена";
    }

    private void SaveGeometry()
    {
        try
        {
            EnsureCustomMeshIsValid();

            if (!string.IsNullOrWhiteSpace(MeshFilePath))
            {
                var path = Path.GetFullPath(MeshFilePath);
                File.WriteAllText(path, MeshText, Encoding.UTF8);
                MeshFilePath = path;
                MeshStatus = $"Сетка сохранена: {path}";
                StatusMessage = "Пользовательская сетка сохранена";
                return;
            }

            MeshStatus = "Пользовательская сетка проверена";
            StatusMessage = "Пользовательская сетка применена";
        }
        catch (Exception ex)
        {
            MeshStatus = $"Ошибка сетки: {ex.Message}";
            StatusMessage = "Ошибка сохранения сетки";
        }
    }

    private void SaveMeshText()
    {
        try
        {
            EnsureCustomMeshIsValid();

            if (string.IsNullOrWhiteSpace(MeshFilePath))
                throw new ArgumentException("Укажите путь к файлу сетки");

            var path = Path.GetFullPath(MeshFilePath);
            File.WriteAllText(path, MeshText, Encoding.UTF8);
            MeshFilePath = path;
            MeshStatus = $"Сетка сохранена: {path}";
            StatusMessage = "Текст сетки сохранен";
        }
        catch (Exception ex)
        {
            MeshStatus = $"Ошибка сетки: {ex.Message}";
            StatusMessage = "Ошибка сохранения текста сетки";
        }
    }

    private void ClearMeshText()
    {
        MeshText = string.Empty;
        MeshStatus = "Текст сетки очищен";
        StatusMessage = "Текст сетки очищен";
    }

    private void LoadGeometry()
    {
        try
        {
            if (string.IsNullOrWhiteSpace(MeshFilePath))
                throw new ArgumentException("Укажите путь к файлу сетки");

            var path = Path.GetFullPath(MeshFilePath);
            var text = File.ReadAllText(path, Encoding.UTF8);

            MeshText = text;
            MeshFilePath = path;

            EnsureCustomMeshIsValid();

            MeshStatus = $"Сетка загружена: {path}";
            StatusMessage = "Пользовательская сетка загружена";
        }
        catch (Exception ex)
        {
            MeshStatus = $"Ошибка сетки: {ex.Message}";
            StatusMessage = "Ошибка загрузки сетки";
        }
    }

    private void EnsureCustomMeshIsValid()
    {
        if (string.IsNullOrWhiteSpace(MeshText))
            throw new ArgumentException("Текст сетки пуст");

        var report = ValidateMeshText(MeshText, SelectedElementType);
        MeshDiagnosticsText = report.Diagnostics;
        ValidationSummary = report.Summary;
    }

    private void ResetParameters()
    {
        SelectedDiffusionCoefficient = "1.0";
        SelectedXiCoefficient = "0.0";
        SelectedSigmaCoefficient = "1.0";
        SelectedSourceFunction = "x";
        SelectedBoundaryFunction = "x * t";
        SelectedSolverMethod = "PCG";
        Tolerance = "1e-10";
        TimeStart = "0";
        TimeStep = "0.25";
        TimeLayerCount = "5";
        TimePointsText = string.Empty;
        SelectedTimeScheme = TsImplicitThreeLayer;
        StatusMessage = "Параметры сброшены";
    }

    private void ResetResults()
    {
        _lastSolution = null;
        _lastTime = 0.0;
        _lastRequest = null;
        _lastTimeLayers.Clear();
        _lastLayerMetrics.Clear();
        _lastSolverSteps.Clear();
        SolutionPointsText = string.Empty;
        CalculationProgress = 0;
        ValidationSummary = string.Empty;
        CalculationStatus = "Ожидание";
        StatusMessage = "Результаты сброшены";
    }

    private void StopCalculation()
    {
        _calculationCts?.Cancel();
        AppendLog("Запрошена остановка расчета.");
        StatusMessage = "Остановка расчета...";
    }

    private async Task RunCalculationAsync(CancellationToken reactiveToken)
    {
        _calculationCts = CancellationTokenSource.CreateLinkedTokenSource(reactiveToken);
        var token = _calculationCts.Token;
        IsCalculationRunning = true;
        CalculationProgress = 5;
        CalculationStatus = "Подготовка";

        try
        {
            var request = CreateRequest();
            _lastRequest = request;
            CalculationProgress = 20;
            CalculationStatus = "Расчет (ParabolicSolver)...";

            var result = await Task.Run(() => SolveParabolicProblem(request, token), token);
            _lastSolution = result.Solution;
            _lastTime = result.Time;
            _lastTimeLayers = [.. result.TimeLayers];
            SolutionPointsText = BuildSolutionPointsText(_lastSolution, _lastTime);

            CalculationProgress = 100;
            CalculationStatus = $"Готово: t={result.Time:F4}, u(center)={result.CenterValue:F6}";
            StatusMessage = "Расчет завершен";
        }
        catch (OperationCanceledException)
        {
            CalculationStatus = "Остановлено";
            StatusMessage = "Расчет остановлен";
        }
        catch (Exception ex)
        {
            CalculationStatus = "Ошибка";
            StatusMessage = $"Ошибка расчета: {ex.Message}";
        }
        finally
        {
            IsCalculationRunning = false;
            _calculationCts.Dispose();
            _calculationCts = null;
        }
    }

    private CalculationRequest CreateRequest()
    {
        var tolerance = ParseDouble(Tolerance, nameof(Tolerance));
        var diffusion = ParseDouble(SelectedDiffusionCoefficient, nameof(SelectedDiffusionCoefficient));
        var xi = ParseDouble(SelectedXiCoefficient, nameof(SelectedXiCoefficient));
        var sigma = ParseDouble(SelectedSigmaCoefficient, nameof(SelectedSigmaCoefficient));
        var solverMethod = NormalizeSolverMethod(SelectedSolverMethod);
        var timeScheme = NormalizeTimeScheme(SelectedTimeScheme);
        var timeStart = ParseDouble(TimeStart, nameof(TimeStart));
        var timeStep = ParseDouble(TimeStep, nameof(TimeStep));
        var timeLayerCount = ParseInt(TimeLayerCount, nameof(TimeLayerCount));
        var timePoints = Array.Empty<double>();
        EnsureCustomMeshIsValid();
        if (!ElementFactoryMap.ContainsKey(SelectedElementType))
            throw new ArgumentException($"Неизвестный тип элемента: {SelectedElementType}");

        if (tolerance <= 0)
            throw new ArgumentException("Точность должна быть > 0");
        if (timeStep <= 0)
            throw new ArgumentException("dt должно быть > 0");
        if (timeLayerCount < 2)
            throw new ArgumentException("Количество временных слоев должно быть не меньше 2");

        if (ValidationSummary.Contains("внутренних", StringComparison.OrdinalIgnoreCase))
            AppendLog("Предупреждение: сетка без внутренних DOF.");

        return new CalculationRequest(
            tolerance,
            diffusion, xi, sigma,
            SelectedSourceFunction, SelectedBoundaryFunction,
            SelectedElementType, solverMethod,
            timeStart, timeStep, timeLayerCount, timeScheme, timePoints, MeshText
        );
    }

    private static CalculationResult SolveParabolicProblem(CalculationRequest request, CancellationToken token)
    {
        token.ThrowIfCancellationRequested();

        var mesh = ReadMeshFromText(request.CustomMeshText, request.ElementType);
        var boundaryIndices = mesh.BoundaryElements.Select(x => x.BoundaryIndex).Distinct().ToArray();
        var boundaryCount = boundaryIndices.Length == 0 ? 0 : boundaryIndices.Max() + 1;
        var allDirichlet = Enumerable.Range(0, boundaryCount)
            .Select(_ => (BoundaryCondition<Vector2D>)new BoundaryCondition<Vector2D>.Dirichlet((p, t) => EvaluateBoundary(request.BoundaryFunction, p, t)))
            .ToArray();

        var boundaryConditions = boundaryCount == 0
            ? [new BoundaryCondition<Vector2D>.Dirichlet((p, t) => EvaluateBoundary(request.BoundaryFunction, p, t))]
            : Enumerable.Range(0, boundaryCount)
                .Select(_ => (BoundaryCondition<Vector2D>)new BoundaryCondition<Vector2D>.Dirichlet((p, t) => EvaluateBoundary(request.BoundaryFunction, p, t)))
                .ToArray();

        var dofInfo = Model.Fem.Assembly.DofManager.NumerateDof(mesh, boundaryConditions);
        bool NoInternalDof = dofInfo?.FreeDofCount == 0;

        var problem = new HyperbolicProblem<Vector2D>(
            Materials:
            [
                new HyperbolicMaterial<Vector2D>(
                    Lambda: (_, _) => request.Diffusion,
                    Xi: (_, _) => request.Xi,
                    Sigma: (_, _) => request.Sigma,
                    Source: (p, t) => EvaluateSource(request.SourceFunction, p, t)
                )
            ],
            InitialCondition: p => EvaluateBoundary(request.BoundaryFunction, p, request.TimeStart),
            BoundaryConditions: boundaryConditions,
            Mesh: mesh
        );

        var solver = new ParabolicSolver<Vector2D, Vector1D, MatrixOperations.Ops2X2>(
            BuildTimeSchemes(request.TimeScheme),
            CsrMatrix.Factory,
            NumericIntegrator<Vector2D, Vector1D, MatrixOperations.Ops2X2>.Instance,
            CreateAlgebraicSolver(request)
        );

        var timePoints = request.TimePoints.Length >= 2
            ? request.TimePoints
            : BuildTimePoints(request.TimeStart, request.TimeStep, request.TimeLayerCount);

        var timeLayers = new List<TimeLayerSolution>(timePoints.Length);
        var initialValues = new double[mesh.VertexCount];
        for (int i = 0; i < mesh.VertexCount; i++)
            initialValues[i] = EvaluateBoundary(request.BoundaryFunction, mesh[i], timePoints[0]);
        timeLayers.Add(new TimeLayerSolution(
            timePoints[0],
            new StationarySolution<Vector2D, Vector1D>(mesh, initialValues)
        ));

        StationarySolution<Vector2D, Vector1D>? last = null;
        var layerIndex = 1;
        var solverSteps = new List<TimeStepSolveInfo>();
        foreach (var solution in solver.Solve(problem, timePoints, new ISolver.Params(request.Tolerance, 10_000), solverSteps))
        {
            token.ThrowIfCancellationRequested();
            last = solution;
            if (layerIndex < timePoints.Length)
                timeLayers.Add(new TimeLayerSolution(timePoints[layerIndex], solution));
            layerIndex++;
        }

        if (last is null)
            throw new InvalidOperationException("Расчет не вернул временных слоев");

        var minX = double.PositiveInfinity;
        var maxX = double.NegativeInfinity;
        var minY = double.PositiveInfinity;
        var maxY = double.NegativeInfinity;
        for (int i = 0; i < mesh.VertexCount; i++)
        {
            var p = mesh[i];
            if (p.X < minX) minX = p.X;
            if (p.X > maxX) maxX = p.X;
            if (p.Y < minY) minY = p.Y;
            if (p.Y > maxY) maxY = p.Y;
        }

        var center = new Vector2D((minX + maxX) * 0.5, (minY + maxY) * 0.5);
        var centerValue = last.Evaluate(center);
        var lastTime = timePoints[^1];

        var mappedSteps = solverSteps
            .Select(x => new SolverStepMetrics(x.Time, x.Residual, x.Iterations, x.StepSeconds))
            .ToList();

        return new CalculationResult(last, lastTime, centerValue, timeLayers, mappedSteps, NoInternalDof);
    }

    private static ITimeScheme[] BuildTimeSchemes(string timeScheme)
    {
        return timeScheme switch
        {
            TsExplicitTwoLayer => [global::Model.Fem.Problem.TimeSchemes.ForwardEuler],
            TsImplicitTwoLayer => [global::Model.Fem.Problem.TimeSchemes.BackwardEuler],
            TsExplicitThreeLayer => [global::Model.Fem.Problem.TimeSchemes.ForwardEuler, global::Model.Fem.Problem.TimeSchemes.ExplicitThreeLayer],
            TsImplicitThreeLayer => [global::Model.Fem.Problem.TimeSchemes.BackwardEuler, global::Model.Fem.Problem.TimeSchemes.ImplicitThreeLayer],
            _ => [global::Model.Fem.Problem.TimeSchemes.BackwardEuler]
        };
    }

    private static ISolver CreateAlgebraicSolver(CalculationRequest request)
    {
        return request.SolverMethod switch
        {
            "PCG+ILU" => new PCGSolver(matrix =>
            {
                if (matrix is not CsrMatrix csr)
                    throw new InvalidOperationException("Для PCG+ILU требуется CsrMatrix");

                return CsrILUFactorization.Create((CsrMatrix)csr.Clone());
            }),

            //Обычный PCG
            _ => new PCGSolver(_ => IdentityPreconditioner.Instance)
        };
    }

    private static double[] BuildTimePoints(double start, double step, int count)
    {
        var points = new double[count];
        points[0] = start;
        for (int i = 1; i < count; i++)
            points[i] = points[i - 1] + step;
        return points;
    }

    private static double EvaluateBoundary(string boundaryFunction, Vector2D p, double t)
    {
        return EvaluateExpression(boundaryFunction, p, t);
    }

    private static double EvaluateSource(string sourceFunction, Vector2D p, double t)
    {
        return EvaluateExpression(sourceFunction, p, t);
    }

    private static double EvaluateExpression(string expression, Vector2D p, double t)
    {
        if (string.IsNullOrWhiteSpace(expression))
            return 0.0;

        // Only allow basic arithmetic to keep expression evaluation predictable.
        if (!Regex.IsMatch(expression, @"^[0-9xytXYTeE+\-*/^().,\s]+$"))
            throw new ArgumentException($"Недопустимое выражение: {expression}");

        var ci = CultureInfo.InvariantCulture;
        var expr = expression;
        expr = Regex.Replace(expr, @"\bx\b", $"({p.X.ToString(ci)})", RegexOptions.IgnoreCase);
        expr = Regex.Replace(expr, @"\by\b", $"({p.Y.ToString(ci)})", RegexOptions.IgnoreCase);
        expr = Regex.Replace(expr, @"\bt\b", $"({t.ToString(ci)})", RegexOptions.IgnoreCase);

        // Normalize decimal separator for parser.
        expr = expr.Replace(',', '.');

        int pos = 0;
        var value = ParseExpressionCore();
        SkipWhitespace();
        if (pos != expr.Length)
            throw new ArgumentException($"РќРµРєРѕСЂСЂРµРєС‚РЅС‹Р№ С„СЂР°РіРјРµРЅС‚ РІС‹СЂР°Р¶РµРЅРёСЏ: {expr[pos..]}");

        return value;

        double ParseExpressionCore()
        {
            var result = ParseTerm();
            while (true)
            {
                SkipWhitespace();
                if (Match('+'))
                {
                    result += ParseTerm();
                    continue;
                }

                if (Match('-'))
                {
                    result -= ParseTerm();
                    continue;
                }

                return result;
            }
        }

        double ParseTerm()
        {
            var result = ParseUnary();
            while (true)
            {
                SkipWhitespace();
                if (Match('*'))
                {
                    result *= ParseUnary();
                    continue;
                }

                if (Match('/'))
                {
                    var divisor = ParseUnary();
                    result /= divisor;
                    continue;
                }

                return result;
            }
        }

        double ParseUnary()
        {
            SkipWhitespace();
            if (Match('+'))
                return ParseUnary();
            if (Match('-'))
                return -ParseUnary();
            return ParsePower();
        }

        double ParsePower()
        {
            var left = ParsePrimary();
            SkipWhitespace();
            if (!Match('^'))
                return left;

            var right = ParseUnary();
            return Math.Pow(left, right);
        }

        double ParsePrimary()
        {
            SkipWhitespace();
            if (Match('('))
            {
                var nested = ParseExpressionCore();
                SkipWhitespace();
                if (!Match(')'))
                    throw new ArgumentException("РћР¶РёРґР°РµС‚СЃСЏ Р·Р°РєСЂС‹РІР°СЋС‰Р°СЏ СЃРєРѕР±РєР°");
                return nested;
            }

            return ParseNumber();
        }

        double ParseNumber()
        {
            SkipWhitespace();
            var start = pos;

            while (pos < expr.Length && char.IsDigit(expr[pos]))
                pos++;

            if (pos < expr.Length && expr[pos] == '.')
            {
                pos++;
                while (pos < expr.Length && char.IsDigit(expr[pos]))
                    pos++;
            }

            if (pos < expr.Length && (expr[pos] == 'e' || expr[pos] == 'E'))
            {
                pos++;
                if (pos < expr.Length && (expr[pos] == '+' || expr[pos] == '-'))
                    pos++;

                var expStart = pos;
                while (pos < expr.Length && char.IsDigit(expr[pos]))
                    pos++;

                if (expStart == pos)
                    throw new ArgumentException("РќРµРєРѕСЂСЂРµРєС‚РЅР°СЏ Р·Р°РїРёСЃСЊ РїРѕРєР°Р·Р°С‚РµР»СЏ СЃС‚РµРїРµРЅРё");
            }

            if (start == pos)
                throw new ArgumentException("РћР¶РёРґР°РµС‚СЃСЏ С‡РёСЃР»Рѕ РёР»Рё РїРѕРґРІС‹СЂР°Р¶РµРЅРёРµ РІ СЃРєРѕР±РєР°С…");

            var token = expr[start..pos];
            if (!double.TryParse(token, NumberStyles.Float, CultureInfo.InvariantCulture, out var parsed))
                throw new ArgumentException($"РќРµ СѓРґР°Р»РѕСЃСЊ СЂР°Р·РѕР±СЂР°С‚СЊ С‡РёСЃР»Рѕ: {token}");

            return parsed;
        }

        void SkipWhitespace()
        {
            while (pos < expr.Length && char.IsWhiteSpace(expr[pos]))
                pos++;
        }

        bool Match(char ch)
        {
            if (pos >= expr.Length || expr[pos] != ch)
                return false;
            pos++;
            return true;
        }
    }

    private static double[] ParseTimePoints(string timePointsText, double timeStart)
    {
        if (string.IsNullOrWhiteSpace(timePointsText))
            return [];

        var separators = new[] { ',', ';', ' ', '\t', '\r', '\n' };
        var tokens = timePointsText.Split(separators, StringSplitOptions.RemoveEmptyEntries | StringSplitOptions.TrimEntries);
        if (tokens.Length == 0)
            return [];

        var points = new double[tokens.Length];
        for (int i = 0; i < tokens.Length; i++)
            points[i] = ParseDouble(tokens[i], nameof(TimePointsText));

        if (points[0] > timeStart)
            throw new ArgumentException("Первый момент времени не должен быть больше t0");

        for (int i = 1; i < points.Length; i++)
        {
            if (points[i] <= points[i - 1])
                throw new ArgumentException("Моменты времени должны строго возрастать");
        }

        return points;
    }

    private static Mesh2D ReadMeshFromText(string meshText, string elementType)
    {
        if (string.IsNullOrWhiteSpace(meshText))
            throw new ArgumentException("Текст сетки пуст");

        if (!ElementFactoryMap.TryGetValue(elementType, out var factories))
            throw new ArgumentException($"Неизвестный тип элемента: {elementType}");

        using var reader = new StringReader(meshText);

        return reader.ReadMesh2D(
            coordinateSystem: IdentityTransform<Vector2D>.Instance,
            finiteElementFactory: factories.Volume,
            boundaryElementFactory: factories.Boundary
        );
    }

    private void ExportToCsv()
    {
        if (_lastTimeLayers.Count == 0 || _lastRequest is null)
        {
            StatusMessage = "Сначала выполните расчет";
            return;
        }

        var outputPath = Path.Combine(Environment.CurrentDirectory, "result.csv");
        if (_lastLayerMetrics.Count == 0)
            _lastLayerMetrics = ComputeLayerMetrics(_lastRequest.Value.BoundaryFunction, _lastTimeLayers);
        WriteLayersCsv(outputPath, _lastRequest.Value, _lastTimeLayers, _lastLayerMetrics, _lastSolverSteps);
        StatusMessage = $"CSV сохранен: {outputPath}";
        AppendLog(StatusMessage);
        return;

#pragma warning disable CS0162
        var ci = CultureInfo.InvariantCulture;
        using var writer = new StreamWriter(outputPath, false, Encoding.UTF8);
        writer.WriteLine($"Схема;{_lastRequest.Value.TimeScheme}");
        writer.WriteLine($"lambda;{_lastRequest.Value.Diffusion.ToString(ci)}");
        writer.WriteLine($"sigma;{_lastRequest.Value.Sigma.ToString(ci)}");
        writer.WriteLine($"u(x,y,t);{_lastRequest.Value.BoundaryFunction}");
        writer.WriteLine($"f(x,y,t);{_lastRequest.Value.SourceFunction}");
        writer.WriteLine($"t;{{{string.Join(", ", _lastTimeLayers.Select(x => x.Time.ToString(ci)))}}}");
        writer.WriteLine();
        writer.WriteLine("t;v;x;y;u*;u;u-u*");

        foreach (var layer in _lastTimeLayers)
        {
            var solution = layer.Solution;
            for (int i = 0; i < solution.Mesh.VertexCount; i++)
            {
                var p = solution.Mesh[i];
                var exact = EvaluateBoundary(_lastRequest.Value.BoundaryFunction, p, layer.Time);
                var actual = solution.Evaluate(p);
                var err = actual - exact;
                writer.WriteLine(
                    $"{FormatSci(layer.Time)};{i};{FormatSci(p.X)};{FormatSci(p.Y)};{FormatSci(exact)};{FormatSci(actual)};{FormatSci(err)}"
                );
            }
        }

        StatusMessage = $"CSV сохранен: {outputPath}";
    }

#pragma warning restore CS0162
    private static string FormatSci(double value) =>
        value.ToString("0.0000000000000000E+000", CultureInfo.InvariantCulture);

    private static string BuildSolutionPointsText(StationarySolution<Vector2D, Vector1D> solution, double time)
    {
        var ci = CultureInfo.InvariantCulture;
        var sb = new StringBuilder();
        sb.AppendLine($"t = {time.ToString(ci)}");
        sb.AppendLine("i\tx\ty\tu");

        for (int i = 0; i < solution.Mesh.VertexCount; i++)
        {
            var p = solution.Mesh[i];
            var u = solution.Evaluate(p);
            sb.AppendLine($"{i}\t{p.X.ToString(ci)}\t{p.Y.ToString(ci)}\t{u.ToString(ci)}");
        }

        return sb.ToString();
    }

    private void AppendLog(string message)
    {
        var line = $"[{DateTime.Now:HH:mm:ss}] {message}";
        ExecutionLog = string.IsNullOrWhiteSpace(ExecutionLog)
            ? line
            : ExecutionLog + Environment.NewLine + line;
    }

    private void LoadMeshTemplate()
    {
        MeshText = BuildMeshTemplateText(SelectedElementType);
        StatusMessage = $"Шаблон сетки загружен для {SelectedElementType}";
        AppendLog(StatusMessage);
        ValidateMesh();
    }

    private void ValidateMesh()
    {
        try
        {
            var report = ValidateMeshText(MeshText, SelectedElementType);
            MeshDiagnosticsText = report.Diagnostics;
            ValidationSummary = report.Summary;
            MeshStatus = report.Summary;
            StatusMessage = report.Summary;
            AppendLog($"Проверка сетки: {report.Summary}");
        }
        catch (Exception ex)
        {
            ValidationSummary = $"Ошибка проверки сетки: {ex.Message}";
            MeshStatus = ValidationSummary;
            StatusMessage = ValidationSummary;
            MeshDiagnosticsText = ex.Message;
            AppendLog(ValidationSummary);
        }
    }

    private static string BuildMeshTemplateText(string elementType)
    {
        var isTriangle = elementType.StartsWith("Triangle", StringComparison.Ordinal);
        if (isTriangle)
        {
            return string.Join(Environment.NewLine,
            [
                "4",
                "0 0",
                "1 0",
                "1 1",
                "0 1",
                "2",
                "0 1 2 0",
                "0 2 3 0",
                "4",
                "0 1 0",
                "1 2 0",
                "2 3 0",
                "0 3 0"
            ]);
        }

        return string.Join(Environment.NewLine,
        [
            "4",
            "0 0",
            "1 0",
            "1 1",
            "0 1",
            "1",
            "0 1 2 3 0",
            "4",
            "0 1 0",
            "1 2 0",
            "2 3 0",
            "0 3 0"
        ]);
    }

    private static MeshValidationReport ValidateMeshText(string meshText, string elementType)
    {
        if (string.IsNullOrWhiteSpace(meshText))
            throw new ArgumentException("Текст сетки пуст");

        var lines = meshText
            .Split(['\r', '\n'], StringSplitOptions.RemoveEmptyEntries | StringSplitOptions.TrimEntries);

        int lineIndex = 0;
        int NextInt(string caption)
        {
            if (lineIndex >= lines.Length)
                throw new FormatException($"Ожидалось поле \"{caption}\"");
            if (!int.TryParse(lines[lineIndex], NumberStyles.Integer, CultureInfo.InvariantCulture, out var value))
                throw new FormatException($"Строка {lineIndex + 1}: ожидалось целое число для \"{caption}\"");
            lineIndex++;
            return value;
        }

        var vertexCount = NextInt("количество вершин");
        if (vertexCount <= 0)
            throw new FormatException("Количество вершин должно быть > 0");

        for (int i = 0; i < vertexCount; i++, lineIndex++)
        {
            if (lineIndex >= lines.Length)
                throw new FormatException($"Строка {lineIndex + 1}: ожидалась вершина");
            var parts = lines[lineIndex].Split(' ', StringSplitOptions.RemoveEmptyEntries);
            if (parts.Length != 2)
                throw new FormatException($"Строка {lineIndex + 1}: вершина должна иметь 2 координаты");
            _ = ParseDouble(parts[0], $"vertex[{i}].x");
            _ = ParseDouble(parts[1], $"vertex[{i}].y");
        }

        var elementCount = NextInt("количество элементов");
        var expectedElementTokens = elementType.StartsWith("Triangle", StringComparison.Ordinal) ? 4 : 5;
        for (int i = 0; i < elementCount; i++, lineIndex++)
        {
            if (lineIndex >= lines.Length)
                throw new FormatException($"Строка {lineIndex + 1}: ожидался элемент");
            var parts = lines[lineIndex].Split(' ', StringSplitOptions.RemoveEmptyEntries);
            if (parts.Length != expectedElementTokens)
                throw new FormatException($"Строка {lineIndex + 1}: ожидалось {expectedElementTokens} целых чисел");
            for (int j = 0; j < parts.Length - 1; j++)
            {
                var v = ParseInt(parts[j], $"element[{i}].v{j}");
                if (v < 0 || v >= vertexCount)
                    throw new FormatException($"Строка {lineIndex + 1}: индекс вершины {v} вне диапазона [0..{vertexCount - 1}]");
            }
            _ = ParseInt(parts[^1], $"element[{i}].material");
        }

        var boundaryCount = NextInt("количество граничных элементов");
        const int expectedBoundaryTokens = 3;
        for (int i = 0; i < boundaryCount; i++, lineIndex++)
        {
            if (lineIndex >= lines.Length)
                throw new FormatException($"Строка {lineIndex + 1}: ожидалась граница");
            var parts = lines[lineIndex].Split(' ', StringSplitOptions.RemoveEmptyEntries);
            if (parts.Length != expectedBoundaryTokens)
                throw new FormatException($"Строка {lineIndex + 1}: ожидалось {expectedBoundaryTokens} целых чисел");
            var a = ParseInt(parts[0], $"boundary[{i}].v0");
            var b = ParseInt(parts[1], $"boundary[{i}].v1");
            if (a < 0 || a >= vertexCount || b < 0 || b >= vertexCount)
                throw new FormatException($"Строка {lineIndex + 1}: индекс вершины границы вне диапазона [0..{vertexCount - 1}]");
        }

        if (lineIndex != lines.Length)
            throw new FormatException($"После разбора сетки остались лишние строки, начиная с {lineIndex + 1}");

        var mesh = ReadMeshFromText(meshText, elementType);
        var boundaryIndices = mesh.BoundaryElements.Select(x => x.BoundaryIndex).Distinct().OrderBy(x => x).ToArray();
        var conditions = boundaryIndices.Length == 0
            ? []
            : Enumerable.Range(0, boundaryIndices.Max() + 1)
                .Select(_ => (BoundaryCondition<Vector2D>)new BoundaryCondition<Vector2D>.Dirichlet((p, t) => 0.0))
                .ToArray();

        int freeDof = 0;
        int fixedDof = 0;
        if (conditions.Length > 0)
        {
            var dof = Model.Fem.Assembly.DofManager.NumerateDof(mesh, conditions);
            freeDof = dof.FreeDofCount;
            fixedDof = dof.FixedDofCount;
        }

        var diagnostics = new StringBuilder();
        diagnostics.AppendLine($"Тип элемента: {elementType}");
        diagnostics.AppendLine($"Вершин: {vertexCount}");
        diagnostics.AppendLine($"Элементов: {elementCount}");
        diagnostics.AppendLine($"Граничных сегментов: {boundaryCount}");
        diagnostics.AppendLine($"DOF (оценка): free={freeDof}, fixed={fixedDof}");
        diagnostics.AppendLine($"Материалы: {string.Join(", ", mesh.FiniteElements.Select(x => x.MaterialIndex).Distinct().OrderBy(x => x))}");
        diagnostics.AppendLine($"Границы: {string.Join(", ", boundaryIndices)}");

        diagnostics.AppendLine("Preview vertices (first 12):");
        for (int i = 0; i < Math.Min(12, mesh.VertexCount); i++)
            diagnostics.AppendLine($"  v{i}: ({mesh[i].X.ToString(CultureInfo.InvariantCulture)}, {mesh[i].Y.ToString(CultureInfo.InvariantCulture)})");
        diagnostics.AppendLine("Preview boundary elements (first 12):");
        foreach (var b in mesh.BoundaryElements.Take(12))
            diagnostics.AppendLine($"  b[{b.BoundaryIndex}] : {string.Join(' ', b.Vertices.ToArray())}");

        var summary = freeDof == 0
            ? "Сетка корректна, но внутренних степеней свободы нет (расчет будет пограничным)."
            : "Сетка корректна.";

        return new MeshValidationReport(summary, diagnostics.ToString());
    }

    private void ExportToPng()
    {
        try
        {
            var path = Path.Combine(Environment.CurrentDirectory, "run-summary.txt");
            var sb = new StringBuilder();
            sb.AppendLine("Сводка расчета");
            sb.AppendLine($"Дата: {DateTime.Now:yyyy-MM-dd HH:mm:ss}");
            sb.AppendLine($"Схема: {_lastRequest?.TimeScheme}");
            sb.AppendLine($"Метод СЛАУ: {SelectedSolverMethod} ({SelectedPreconditioner})");
            sb.AppendLine($"Слоев: {_lastTimeLayers.Count}");
            if (_lastLayerMetrics.Count > 0)
            {
                var last = _lastLayerMetrics[^1];
                sb.AppendLine($"Последний слой t={last.Time}: L2={last.L2NormError:E6}, Linf={last.LInfError:E6}, avg={last.AvgAbsError:E6}");
            }
            sb.AppendLine(ValidationSummary);
            File.WriteAllText(path, sb.ToString(), Encoding.UTF8);
            StatusMessage = $"Сводка сохранена: {path}";
            AppendLog(StatusMessage);
        }
        catch (Exception ex)
        {
            StatusMessage = $"Ошибка экспорта: {ex.Message}";
            AppendLog(StatusMessage);
        }
    }

    private async Task RunCalculationWorkflowAsync(CancellationToken reactiveToken)
    {
        _calculationCts = CancellationTokenSource.CreateLinkedTokenSource(reactiveToken);
        var token = _calculationCts.Token;
        IsCalculationRunning = true;
        CalculationProgress = 1;
        CalculationStatus = "Подготовка";
        AppendLog("Старт расчета.");

        try
        {
            ValidateMesh();
            var request = CreateRequest();

            var result = await Task.Run(() => SolveParabolicProblem(request, token), token);

            _lastRequest = request;
            _lastSolution = result.Solution;
            _lastTime = result.Time;
            _lastTimeLayers = [.. result.TimeLayers];
            _lastLayerMetrics = ComputeLayerMetrics(request.BoundaryFunction, result.TimeLayers);
            _lastSolverSteps = [.. result.SolverSteps];
            SolutionPointsText = BuildSolutionPointsText(_lastSolution, _lastTime);

            CalculationProgress = 100;
            CalculationStatus = $"Готово: t={result.Time:F4}";
            StatusMessage = "Расчет завершен";
            AppendLog(StatusMessage);
        }
        catch (Exception ex)
        {
            CalculationStatus = "Ошибка";
            StatusMessage = $"Ошибка расчета: {ex.Message}";
            AppendLog(StatusMessage);
        }
        finally
        {
            IsCalculationRunning = false;
            _calculationCts?.Dispose();
            _calculationCts = null;
        }
    }

    private static void WriteSolverCsv(string path, IReadOnlyList<SolverStepMetrics> steps)
    {
        using var writer = new StreamWriter(path, false, Encoding.UTF8);
        writer.WriteLine("t;iterations;residual;seconds");
        foreach (var s in steps)
            writer.WriteLine($"{s.Time.ToString(CultureInfo.InvariantCulture)};{s.Iterations};{s.Residual.ToString(CultureInfo.InvariantCulture)};{s.StepSeconds.ToString(CultureInfo.InvariantCulture)}");
    }

    private static void WriteLayersCsv(
        string path,
        CalculationRequest request,
        IReadOnlyList<TimeLayerSolution> layers,
        IReadOnlyList<LayerErrorMetrics> metrics,
        IReadOnlyList<SolverStepMetrics> solverSteps)
    {
        var ci = CultureInfo.InvariantCulture;
        using var writer = new StreamWriter(path, false, Encoding.UTF8);
        writer.WriteLine($"scheme;{request.TimeScheme}");
        writer.WriteLine($"lambda;{request.Diffusion.ToString(ci)}");
        writer.WriteLine($"sigma;{request.Sigma.ToString(ci)}");
        writer.WriteLine($"u(x,y,t);{request.BoundaryFunction}");
        writer.WriteLine($"f(x,y,t);{request.SourceFunction}");
        writer.WriteLine($"element;{request.ElementType}");
        writer.WriteLine($"solver;{request.SolverMethod}");
        writer.WriteLine();
        writer.WriteLine("t;vertex;x;y;u_exact;u;abs_error");

        foreach (var layer in layers)
        {
            var solution = layer.Solution;
            for (int i = 0; i < solution.Mesh.VertexCount; i++)
            {
                var p = solution.Mesh[i];
                var exact = EvaluateBoundary(request.BoundaryFunction, p, layer.Time);
                var actual = solution.Evaluate(p);
                var err = Math.Abs(actual - exact);
                writer.WriteLine($"{layer.Time.ToString(ci)};{i};{p.X.ToString(ci)};{p.Y.ToString(ci)};{exact.ToString(ci)};{actual.ToString(ci)};{err.ToString(ci)}");
            }
        }

        writer.WriteLine();
        writer.WriteLine("layer_t;l2;linf;avg_abs");
        foreach (var m in metrics)
            writer.WriteLine($"{m.Time.ToString(ci)};{m.L2NormError.ToString(ci)};{m.LInfError.ToString(ci)};{m.AvgAbsError.ToString(ci)}");

        writer.WriteLine();
        writer.WriteLine("solver_t;iterations;residual;seconds");
        foreach (var s in solverSteps)
            writer.WriteLine($"{s.Time.ToString(ci)};{s.Iterations};{s.Residual.ToString(ci)};{s.StepSeconds.ToString(ci)}");
    }

    private static List<LayerErrorMetrics> ComputeLayerMetrics(string exactFunction, IReadOnlyList<TimeLayerSolution> layers)
    {
        var metrics = new List<LayerErrorMetrics>(layers.Count);
        foreach (var layer in layers)
        {
            var sumSq = 0.0;
            var maxAbs = 0.0;
            var sumAbs = 0.0;
            var n = layer.Solution.Mesh.VertexCount;
            for (int i = 0; i < n; i++)
            {
                var p = layer.Solution.Mesh[i];
                var exact = EvaluateBoundary(exactFunction, p, layer.Time);
                var actual = layer.Solution.Evaluate(p);
                var err = actual - exact;
                var absErr = Math.Abs(err);
                sumSq += err * err;
                sumAbs += absErr;
                if (absErr > maxAbs)
                    maxAbs = absErr;
            }

            metrics.Add(new LayerErrorMetrics(
                layer.Time,
                Math.Sqrt(sumSq / Math.Max(1, n)),
                maxAbs,
                sumAbs / Math.Max(1, n)
            ));
        }
        return metrics;
    }

    private static string BuildValidationSummary(IReadOnlyList<BatchSummaryRow> rows, double tolerance, string manufacturedCase)
    {
        if (rows.Count == 0)
            return "Нет результатов";

        var best = rows.OrderBy(x => x.FinalL2).First();
        var worst = rows.OrderByDescending(x => x.FinalL2).First();
        var sb = new StringBuilder();
        sb.Append($"Лучший запуск: {best.RunName}, L2={best.FinalL2:E4}; худший запуск: {worst.RunName}, L2={worst.FinalL2:E4}.");
        if (!string.Equals(manufacturedCase, "Manual", StringComparison.Ordinal))
        {
            var pass = best.FinalLInf <= Math.Max(1e-12, tolerance * 20);
            sb.Append(pass
                ? " Автопроверка manufactured-case: PASS."
                : " Автопроверка manufactured-case: FAIL.");
        }

        return sb.ToString();
    }

    private static double[] ParseDoubleList(string text, string fieldName)
    {
        if (string.IsNullOrWhiteSpace(text))
            return [];
        var tokens = text.Split([',', ';', ' ', '\t', '\r', '\n'], StringSplitOptions.RemoveEmptyEntries | StringSplitOptions.TrimEntries);
        return [.. tokens.Select(x => ParseDouble(x, fieldName)).Distinct()];
    }

    private static string[] ParseStringList(string text, string fieldName)
    {
        if (string.IsNullOrWhiteSpace(text))
            return [];
        var tokens = text.Split([',', ';', '\r', '\n'], StringSplitOptions.RemoveEmptyEntries | StringSplitOptions.TrimEntries);
        if (tokens.Length == 0)
            throw new ArgumentException($"Пустое поле списка: {fieldName}");
        return tokens;
    }

    private static string NormalizeSolverMethod(string solverMethod)
{
    if (string.IsNullOrWhiteSpace(solverMethod))
        return "PCG";

    var normalized = Regex.Replace(solverMethod.Trim(), @"\s+", "").ToUpperInvariant();

    return normalized switch
    {
        "PCG+ILU" or "PCGILU" => "PCG+ILU",
        _ => "PCG"   // всё остальное = обычный PCG
    };
}

    private static string NormalizePreconditioner(string preconditioner)
    {
        if (string.IsNullOrWhiteSpace(preconditioner))
            return "Identity";

        var normalized = Regex.Replace(preconditioner.Trim(), @"\s+", string.Empty).ToUpperInvariant();
        return normalized switch
        {
            "IDENTITY" or "ЕДИНИЧНЫЙ" => "Identity",
            "ILU" => "ILU",
            _ => throw new ArgumentException($"Неизвестный предобуславливатель: {preconditioner}. Допустимо: Identity, ILU.")
        };
    }

    private static string NormalizeTimeScheme(string timeScheme)
    {
        if (string.IsNullOrWhiteSpace(timeScheme))
            throw new ArgumentException("Схема по времени не задана");

        var normalized = Regex.Replace(timeScheme.Trim(), @"\s+", string.Empty).ToUpperInvariant();
        return normalized switch
        {
            "ЯВНАЯДВУХСЛОЙНАЯ" or "EXPLICITTWOLAYER" or "FORWARDEULER" => TsExplicitTwoLayer,
            "НЕЯВНАЯДВУХСЛОЙНАЯ" or "IMPLICITTWOLAYER" or "BACKWARDEULER" => TsImplicitTwoLayer,
            "ЯВНАЯТРЕХСЛОЙНАЯ" or "ЯВНАЯТРЁХСЛОЙНАЯ" or "EXPLICITTHREELAYER" => TsExplicitThreeLayer,
            "НЕЯВНАЯТРЕХСЛОЙНАЯ" or "НЕЯВНАЯТРЁХСЛОЙНАЯ" or "IMPLICITTHREELAYER" => TsImplicitThreeLayer,
            _ => throw new ArgumentException(
                $"Неизвестная схема по времени: {timeScheme}. " +
                $"Допустимо: {TsExplicitTwoLayer}, {TsImplicitTwoLayer}, {TsExplicitThreeLayer}, {TsImplicitThreeLayer}."
            )
        };
    }

    private void ApplyManufacturedCase(string caseName)
    {
        switch (caseName)
        {
            case "Case: u=t*x; f=x":
                SelectedBoundaryFunction = "t*x";
                SelectedSourceFunction = "x";
                break;
            case "Case: u=t*(x^2+y^2); f=x^2+y^2-4*t":
                SelectedBoundaryFunction = "t*(x^2+y^2)";
                SelectedSourceFunction = "x^2+y^2-4*t";
                break;
            case "Case: u=(1+t)*(x+y); f=(x+y)":
                SelectedBoundaryFunction = "(1+t)*(x+y)";
                SelectedSourceFunction = "(x+y)";
                break;
            default:
                break;
        }
    }

    private static double ParseDouble(string value, string fieldName)
    {
        if (double.TryParse(value, NumberStyles.Float, CultureInfo.CurrentCulture, out var d))
            return d;
        if (double.TryParse(value, NumberStyles.Float, CultureInfo.InvariantCulture, out d))
            return d;
        if (double.TryParse(value.Replace(',', '.'), NumberStyles.Float, CultureInfo.InvariantCulture, out d))
            return d;
        throw new ArgumentException($"Некорректное значение в поле {fieldName}: {value}");
    }

    private static int ParseInt(string value, string fieldName)
    {
        if (int.TryParse(value, NumberStyles.Integer, CultureInfo.CurrentCulture, out var n))
            return n;
        if (int.TryParse(value, NumberStyles.Integer, CultureInfo.InvariantCulture, out n))
            return n;
        throw new ArgumentException($"Некорректное целое в поле {fieldName}: {value}");
    }

    private readonly record struct CalculationRequest(
        double Tolerance,
        double Diffusion,
        double Xi,
        double Sigma,
        string SourceFunction,
        string BoundaryFunction,
        string ElementType,
        string SolverMethod,
        double TimeStart,
        double TimeStep,
        int TimeLayerCount,
        string TimeScheme,
        double[] TimePoints,
        string CustomMeshText
    );

    private readonly record struct CalculationResult(
        StationarySolution<Vector2D, Vector1D> Solution,
        double Time,
        double CenterValue,
        IReadOnlyList<TimeLayerSolution> TimeLayers,
        IReadOnlyList<SolverStepMetrics> SolverSteps,
        bool NoInternalDof
    );

    private readonly record struct TimeLayerSolution(
        double Time,
        StationarySolution<Vector2D, Vector1D> Solution
    );

    private readonly record struct LayerErrorMetrics(
        double Time,
        double L2NormError,
        double LInfError,
        double AvgAbsError
    );

    private readonly record struct SolverStepMetrics(
        double Time,
        double Residual,
        int Iterations,
        double StepSeconds
    );

    private readonly record struct MeshValidationReport(
        string Summary,
        string Diagnostics
    );

    private sealed class BatchSummaryRow
    {
        public string RunName { get; set; } = string.Empty;
        public string TimeScheme { get; set; } = string.Empty;
        public string ElementType { get; set; } = string.Empty;
        public double Dt { get; set; }
        public double Diffusion { get; set; }
        public double Sigma { get; set; }
        public double Tolerance { get; set; }
        public double FinalTime { get; set; }
        public double FinalL2 { get; set; }
        public double FinalLInf { get; set; }
        public double FinalAvgAbs { get; set; }
        public int MaxIterations { get; set; }
        public double MaxResidual { get; set; }
        public double TotalSolveSeconds { get; set; }
    }
}
