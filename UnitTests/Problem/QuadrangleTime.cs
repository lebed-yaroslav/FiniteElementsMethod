using Model.Core.CoordinateSystem;
using Model.Core.Matrix;
using Model.Core.Solver;
using Model.Core.Solver.Precondition;
using Model.Fem.Elements;
using Model.Fem.Integrator;
using Model.Fem.Mesh;
using Model.Fem.Problem;
using Telma;
using static Model.Core.CoordinateSystem.MatrixOperations;

namespace UnitTests.Problem;

public class ParabolicProblemQuadrangleTests
{
    private const double Dt = 0.25;

    private static double[] TimePoints { get; } = [0.0, 0.25, 0.50, 0.75, 1.0];

    private const double TStart = 0.0;
    private const double TEnd = 1.0;
    private const double Eps = 1e-10;

    private static double A(double t) => t;
    private static double dA(double t) => 1.0;

    private static void AssertScaled(ReadOnlySpan<double> solution, ReadOnlySpan<double> spatialValues)
    {
        var scale = A(TEnd);

        Assert.Equal(spatialValues.Length, solution.Length);
        for (int i = 0; i < solution.Length; i++)
            Assert.Equal(scale * spatialValues[i], solution[i], Eps);
    }

    [Theory]
    [InlineData(true)]
    [InlineData(false)]
    public void МастерЭлемент_ЛинейнаяФункция_БиЛинейныйБазис_СУчетомВремени(bool isImplicit)
    {
        string TestMesh1 =
        """
        4
        0 0
        1 0
        1 1
        0 1
        1
        0 1 2 3 0
        4
        0 1 0
        1 2 0
        2 3 0
        3 0 0
        """;

        var mesh = new StringReader(TestMesh1).ReadMesh2D(
            coordinateSystem: IdentityTransform<Vector2D>.Instance,
            FiniteElements.Quadrangle.Bilinear,
            FiniteElements.Segment.Linear
        );

        var problem = new HyperbolicProblem2D(
            Materials:
            [
                new HyperbolicMaterial2D(
                    Lambda: (p, t) => 1.0,
                    Xi:     (p, t) => 0.0,
                    Sigma:  (p, t) => 1.0,
                    Source: (p, t) => dA(t) * p.X
                )
            ],
            InitialCondition: p => A(0) * p.X,
            BoundaryConditions:
            [
                new BoundaryCondition2D.Dirichlet(
                    Value: (p, t) => A(t) * p.X
                )
            ],
            Mesh: mesh
        );

        var solver = new ParabolicSolver2D(
            [isImplicit ? TimeSchemes.BackwardEuler : TimeSchemes.ForwardEuler],
            DenseMatrix.Factory,
            NumericItegrator2D.Instance,
            new PCGSolver(m => IdentityPreconditioner.Instance)
        );

        var solution = solver.Solve(
            problem,
            TimePoints,
            new ISolver.Params(1e-12, 10000)
        );

        AssertScaled(solution.Last().Coefficients, [0.0, 1.0, 1.0, 0.0]);
    }

    [Theory]
    [InlineData(true)]
    [InlineData(false)]
    public void МастерЭлемент_НулеваяГраница_НулевойИсточник_ДаетНуль(bool isImplicit)
    {
        string TestMesh1 =
        """
        4
        0 0
        1 0
        1 1
        0 1
        1
        0 1 2 3 0
        4
        0 1 0
        1 2 0
        2 3 0
        3 0 0
        """;

        var mesh = new StringReader(TestMesh1).ReadMesh2D(
            coordinateSystem: IdentityTransform<Vector2D>.Instance,
            FiniteElements.Quadrangle.Bilinear,
            FiniteElements.Segment.Linear
        );

        var problem = new HyperbolicProblem2D(
            Materials:
            [
                new(
                    Lambda: (p, t) => 1.0,
                    Xi:     (p, t) => 0.0,
                    Sigma:  (p, t) => 1.0,
                    Source: (p, t) => 0.0
                )
            ],
            InitialCondition: p => 0.0,
            BoundaryConditions:
            [
                new BoundaryCondition2D.Dirichlet(
                Value: (p, t) => 0.0
            )
            ],
            Mesh: mesh
        );

        var solver = new ParabolicSolver2D(
            [isImplicit ? TimeSchemes.BackwardEuler : TimeSchemes.ForwardEuler],
            DenseMatrix.Factory,
            NumericItegrator2D.Instance,
            new PCGSolver(m => IdentityPreconditioner.Instance)
        );

        var solution = solver.Solve(problem, TimePoints, new ISolver.Params(1e-12, 10000));

        AssertScaled(solution.Last().Coefficients, new double[4]);
    }

    [Theory]
    [InlineData(true)]
    [InlineData(false)]
    public void МастерЭлемент_НулевыеДанные_НулевоеРешение(bool isImplicit)
    {
        string TestMesh1 =
        """
    4
    0 0
    1 0
    1 1
    0 1
    1
    0 1 2 3 0
    4
    0 1 0
    1 2 0
    2 3 0
    3 0 0
    """;

        var mesh = new StringReader(TestMesh1).ReadMesh2D(
            coordinateSystem: IdentityTransform<Vector2D>.Instance,
            FiniteElements.Quadrangle.LagrangeQuadratic,
            FiniteElements.Segment.LagrangeQuadratic
        );

        var problem = new HyperbolicProblem2D(
            Materials:
            [
                new(
                    Lambda: (p, t) => 1.0,
                    Xi:     (p, t) => 0.0,
                    Sigma:  (p, t) => 1.0,
                    Source: (p, t) => 0.0
                )
            ],
            InitialCondition: p => A(0) * p.X,
            BoundaryConditions:
            [
                new BoundaryCondition2D.Dirichlet(
                Value: (p, t) => 0.0
            )
            ],
            Mesh: mesh
        );

        var solver = new ParabolicSolver2D(
            [isImplicit ? TimeSchemes.BackwardEuler : TimeSchemes.ForwardEuler],
            DenseMatrix.Factory,
            NumericItegrator2D.Instance,
            new PCGSolver(m => IdentityPreconditioner.Instance)
        );

        var solution = solver.Solve(
            problem,
            TimePoints,
            new ISolver.Params(1e-12, 10000)
        );

        AssertScaled(solution.Last().Coefficients, new double[9]);
    }

    [Theory]
    [InlineData(true)]
    [InlineData(false)]
    public void ПроизвольныйЭлемент_ЛинейнаяФункция_БиЛинейныйБазис_СУчетомВремени(bool isImplicit)
    {
        string TestMesh1 =
        """
        4
        0 0
        3 -2
        3 4
        -1 2
        1
        0 1 2 3 0
        4
        0 1 0
        1 2 0
        2 3 0
        3 0 0
        """;

        var mesh = new StringReader(TestMesh1).ReadMesh2D(
            coordinateSystem: IdentityTransform<Vector2D>.Instance,
            FiniteElements.Quadrangle.Bilinear,
            FiniteElements.Segment.Linear
        );

        var problem = new HyperbolicProblem2D(
            Materials:
            [
                new(
                    Lambda: (p, t) => 1.0,
                    Xi:     (p, t) => 0.0,
                    Sigma:  (p, t) => 1.0,
                    Source: (p, t) => dA(t) * p.X
                )
            ],
            InitialCondition: p => A(0) * p.X,
            BoundaryConditions:
            [
                new BoundaryCondition2D.Dirichlet(
                    Value: (p, t) => A(t) * p.X
                )
            ],
            Mesh: mesh
        );

        var solver = new ParabolicSolver2D(
            [isImplicit ? TimeSchemes.BackwardEuler : TimeSchemes.ForwardEuler],
            DenseMatrix.Factory,
            NumericItegrator2D.Instance,
            new PCGSolver(m => IdentityPreconditioner.Instance)
        );

        var solution = solver.Solve(
            problem,
            TimePoints,
            new ISolver.Params(1e-12, 10000)
        );

        AssertScaled(solution.Last().Coefficients, [-1.0, 3.0, 3.0, 0.0]);
    }

    [Theory]
    [InlineData(true)]
    [InlineData(false)]
    public void РавномернаяСеткаСВнутреннимУзлом_ЛинейнаяФункция_БиЛинейныйБазис_СУчетомВремени(bool isImplicit)
    {
        string TestMesh1 =
        """
        9
        0 0
        1 0
        2 0
        0 1
        1 1
        2 1
        0 2
        1 2
        2 2
        4
        0 1 4 3 0
        1 2 5 4 0
        3 4 7 6 0
        4 5 8 7 0
        8
        0 1 0
        1 2 0
        2 5 0
        5 8 0
        7 8 0
        6 7 0
        0 3 0
        3 6 0
        """;

        var mesh = new StringReader(TestMesh1).ReadMesh2D(
            coordinateSystem: IdentityTransform<Vector2D>.Instance,
            FiniteElements.Quadrangle.Bilinear,
            FiniteElements.Segment.Linear
        );

        var problem = new HyperbolicProblem2D(
            Materials:
            [
                new HyperbolicMaterial2D(
                    Lambda: (p, t) => 1.0,
                    Xi:     (p, t) => 0.0,
                    Sigma:  (p, t) => 1.0,
                    Source: (p, t) => dA(t) * p.X
                )
            ],
            InitialCondition: p => A(0) * p.X,
            BoundaryConditions:
            [
                new BoundaryCondition2D.Dirichlet(
                    Value: (p, t) => A(t) * p.X
                )
            ],
            Mesh: mesh
        );

        var solver = new ParabolicSolver2D(
            [isImplicit ? TimeSchemes.BackwardEuler : TimeSchemes.ForwardEuler],
            DenseMatrix.Factory,
            NumericItegrator2D.Instance,
            new PCGSolver(m => IdentityPreconditioner.Instance)
        );

        var solution = solver.Solve(
            problem,
            TimePoints,
            new ISolver.Params(1e-12, 10000)
        );

        AssertScaled(solution.Last().Coefficients, [1.0, 2.0, 1.0, 0.0, 2.0, 0.0, 2.0, 1.0, 0.0]);
    }

    [Theory]
    [InlineData(true)]
    [InlineData(false)]
    public void ПроизвольныйЭлемент_КвадратичнаяФункция_БиКвадратичныйБазис_СУчетомВремени(bool isImplicit)
    {
        string TestMesh1 =
        """
        4
        0 0
        1 0
        1 1
        0 1
        1
        0 1 2 3 0
        4
        0 1 0
        1 2 0
        2 3 0
        3 0 0
        """;

        var mesh = new StringReader(TestMesh1).ReadMesh2D(
            coordinateSystem: IdentityTransform<Vector2D>.Instance,
            FiniteElements.Quadrangle.LagrangeQuadratic,
            FiniteElements.Segment.LagrangeQuadratic
        );

        var problem = new HyperbolicProblem2D(
            Materials:
            [
                new(
                    Lambda: (p, t) => 1.0,
                    Xi:     (p, t) => 0.0,
                    Sigma:  (p, t) => 1.0,
                    Source: (p, t) => dA(t) * p.X * p.X - 2.0 * A(t)
                )
            ],
            InitialCondition: p => A(0) * p.X * p.X,
            BoundaryConditions:
            [
                new BoundaryCondition2D.Dirichlet(
                    Value: (p, t) => A(t) * p.X * p.X
                )
            ],
            Mesh: mesh
        );

        var solver = new ParabolicSolver2D(
            [isImplicit ? TimeSchemes.BackwardEuler : TimeSchemes.ForwardEuler],
            DenseMatrix.Factory,
            NumericItegrator2D.Instance,
            new PCGSolver(m => IdentityPreconditioner.Instance)
        );

        var solution = solver.Solve(
            problem,
            TimePoints,
            new ISolver.Params(1e-12, 10000)
        );

        AssertScaled(solution.Last().Coefficients, [0.25, 0.0, 0.25, 1.0, 0.25, 0.0, 1.0, 1.0, 0.0]);
    }

    [Theory]
    [InlineData(true)]
    [InlineData(false)]
    public void МастерЭлемент_ЛинейнаяФункция_БиКвадратичныйБазис_СУчетомВремени(bool isImplicit)
    {
        string TestMesh1 =
        """
        4
        0 0
        1 0
        1 1
        0 1
        1
        0 1 2 3 0
        4
        0 1 0
        1 2 0
        2 3 0
        3 0 0
        """;

        var mesh = new StringReader(TestMesh1).ReadMesh2D(
            coordinateSystem: IdentityTransform<Vector2D>.Instance,
            FiniteElements.Quadrangle.LagrangeQuadratic,
            FiniteElements.Segment.LagrangeQuadratic
        );

        var problem = new HyperbolicProblem2D(
            Materials:
            [
                new(
                    Lambda: (p, t) => 1.0,
                    Xi:     (p, t) => 0.0,
                    Sigma:  (p, t) => 1.0,
                    Source: (p, t) => dA(t) * p.X
                )
            ],
            InitialCondition: p => A(0) * p.X,
            BoundaryConditions:
            [
                new BoundaryCondition2D.Dirichlet(
                    Value: (p, t) => A(t) * p.X
                )
            ],
            Mesh: mesh
        );

        var solver = new ParabolicSolver2D(
            [isImplicit ? TimeSchemes.BackwardEuler : TimeSchemes.ForwardEuler],
            DenseMatrix.Factory,
            NumericItegrator2D.Instance,
            new PCGSolver(m => IdentityPreconditioner.Instance)
        );

        var solution = solver.Solve(
            problem,
            TimePoints,
            new ISolver.Params(1e-12, 10000)
        );

        AssertScaled(solution.Last().Coefficients, [0.5, 0.0, 0.5, 1.0, 0.5, 0.0, 1.0, 1.0, 0.0]);
    }

    [Theory]
    [InlineData(true)]
    [InlineData(false)]
    public void РавномернаяСеткаСВнутреннимУзлом_ЛинейнаяФункция_БиКвадратичныйБазис_СУчетомВремени(bool isImplicit)
    {
        string TestMesh1 =
        """
        9
        0 0
        0.5 0
        1 0
        0 0.5
        0.5 0.5
        1 0.5
        0 1
        0.5 1
        1 1
        4
        0 1 4 3 0
        1 2 5 4 0
        3 4 7 6 0
        4 5 8 7 0
        8
        0 1 0
        1 2 0
        2 5 0
        5 8 0
        7 8 0
        6 7 0
        0 3 0
        3 6 0
        """;

        var mesh = new StringReader(TestMesh1).ReadMesh2D(
            coordinateSystem: IdentityTransform<Vector2D>.Instance,
            FiniteElements.Quadrangle.LagrangeQuadratic,
            FiniteElements.Segment.LagrangeQuadratic
        );

        var problem = new HyperbolicProblem2D(
            Materials:
            [
                new(
                    Lambda: (p, t) => 1.0,
                    Xi:     (p, t) => 0.0,
                    Sigma:  (p, t) => 1.0,
                    Source: (p, t) => dA(t) * p.X
                )
            ],
            InitialCondition: p => A(0) * p.X,
            BoundaryConditions:
            [
                new BoundaryCondition2D.Dirichlet(
                    Value: (p, t) => A(t) * p.X
                )
            ],
            Mesh: mesh
        );

        var solver = new ParabolicSolver2D(
            [isImplicit ? TimeSchemes.BackwardEuler : TimeSchemes.ForwardEuler],
            DenseMatrix.Factory,
            NumericItegrator2D.Instance,
            new PCGSolver(m => IdentityPreconditioner.Instance)
        );

        var solution = solver.Solve(
            problem,
            TimePoints,
            new ISolver.Params(1e-12, 10000)
        );

        AssertScaled(solution.Last().Coefficients, [
            0.5, 0.5, 0.25, 0.75, 0.5,
            0.25, 0.75, 0.25, 0.75, 0.75,
            1.0, 0.0, 0.25, 1.0, 0.75,
            0.0, 0.25, 1.0, 0.5, 0.0,
            1.0, 0.0, 1.0, 0.5, 0.0
        ]);
    }

    [Theory]
    [InlineData(true)]
    [InlineData(false)]
    public void РавномернаяСеткаСВнутреннимУзлом_КвадратичнаяФункция_БиКвадратичныйБазис_СУчетомВремени(bool isImplicit)
    {
        string TestMesh1 =
        """
        9
        0 0
        0.5 0
        1 0
        0 0.5
        0.5 0.5
        1 0.5
        0 1
        0.5 1
        1 1
        4
        0 1 4 3 0
        1 2 5 4 0
        3 4 7 6 0
        4 5 8 7 0
        8
        0 1 0
        1 2 0
        2 5 0
        5 8 0
        7 8 0
        6 7 0
        0 3 0
        3 6 0
        """;

        var mesh = new StringReader(TestMesh1).ReadMesh2D(
            coordinateSystem: IdentityTransform<Vector2D>.Instance,
            FiniteElements.Quadrangle.LagrangeQuadratic,
            FiniteElements.Segment.LagrangeQuadratic
        );

        var problem = new HyperbolicProblem2D(
            Materials:
            [
                new HyperbolicMaterial2D(
                    Lambda: (p, t) => 1.0,
                    Xi:     (p, t) => 0.0,
                    Sigma:  (p, t) => 1.0,
                    Source: (p, t) => dA(t) * p.X * p.X - 2.0 * A(t)
                )
            ],
            InitialCondition: p => A(0) * p.X * p.X,
            BoundaryConditions:
            [
                new BoundaryCondition2D.Dirichlet(
                    Value: (p, t) => A(t) * p.X * p.X
                )
            ],
            Mesh: mesh
        );

        var solver = new ParabolicSolver2D(
            [isImplicit ? TimeSchemes.BackwardEuler : TimeSchemes.ForwardEuler],
            DenseMatrix.Factory,
            NumericItegrator2D.Instance,
            new PCGSolver(m => IdentityPreconditioner.Instance)
        );

        var solution = solver.Solve(
            problem,
            TimePoints,
            new ISolver.Params(1e-12, 10000)
        );

        AssertScaled(solution.Last().Coefficients, [
            0.2500, 0.2500, 0.0625, 0.5625, 0.2500,
            0.0625, 0.5625, 0.0625, 0.5625, 0.5625,
            1.0000, 0.0000, 0.0625, 1.0000, 0.5625,
            0.0000, 0.0625, 1.0000, 0.2500, 0.0000,
            1.0000, 0.0000, 1.0000, 0.2500, 0.0000
        ]);
    }

    [Theory]
    [InlineData(true)]
    [InlineData(false)]
    public void РавномернаяСеткаСВнутреннимУзлом_КвадратичнаяФункция_ПостоянноеПоВремениA(bool isImplicit)
    {
        string TestMesh1 =
        """
    9
    0 0
    0.5 0
    1 0
    0 0.5
    0.5 0.5
    1 0.5
    0 1
    0.5 1
    1 1
    4
    0 1 4 3 0
    1 2 5 4 0
    3 4 7 6 0
    4 5 8 7 0
    8
    0 1 0
    1 2 0
    2 5 0
    5 8 0
    7 8 0
    6 7 0
    0 3 0
    3 6 0
    """;

        double A(double t) => 1.0;
        double dA(double t) => 0.0;

        var mesh = new StringReader(TestMesh1).ReadMesh2D(
            coordinateSystem: IdentityTransform<Vector2D>.Instance,
            FiniteElements.Quadrangle.LagrangeQuadratic,
            FiniteElements.Segment.LagrangeQuadratic
        );

        var problem = new HyperbolicProblem2D(
            Materials:
            [
                new(
                    Lambda: (p, t) => 1.0,
                    Xi:     (p, t) => 0.0,
                    Sigma:  (p, t) => 1.0,
                    Source: (p, t) => dA(t) * p.X * p.X - 2.0 * A(t)
                )
            ],
            InitialCondition: p => A(0) * p.X * p.X,
            BoundaryConditions:
            [
                new BoundaryCondition2D.Dirichlet(
                Value: (p, t) => A(t) * p.X * p.X
            )
            ],
            Mesh: mesh
        );

        var solver = new ParabolicSolver2D(
            [isImplicit ? TimeSchemes.BackwardEuler : TimeSchemes.ForwardEuler],
            DenseMatrix.Factory,
            NumericItegrator2D.Instance,
            new PCGSolver(m => IdentityPreconditioner.Instance)
        );

        var solution = solver.Solve(
            problem,
            TimePoints,
            new ISolver.Params(1e-12, 10000)
        );

        AssertScaled(solution.Last().Coefficients, [
            0.2500, 0.2500, 0.0625, 0.5625, 0.2500,
        0.0625, 0.5625, 0.0625, 0.5625, 0.5625,
        1.0000, 0.0000, 0.0625, 1.0000, 0.5625,
        0.0000, 0.0625, 1.0000, 0.2500, 0.0000,
        1.0000, 0.0000, 1.0000, 0.2500, 0.0000
        ]);
    }
}

public class ParabolicProblemQuadrangleHermiteTests
{
    private const double Eps = 1e-10;

    private const string квадратик_эталонный =
        """
        4
        0 0
        1 0
        1 1
        0 1
        1
        0 1 2 3 0
        4
        0 1 0
        1 2 0
        2 3 0
        3 0 0
        """;

    private const string сетка_на_6_элементов =
        """
        12
        0 0
        2 0
        4 0
        6 0
        0 1
        2 1
        4 1
        6 1
        0 2
        2 2
        4 2
        6 2
        6
        0 1 5 4 0
        1 2 6 5 0
        2 3 7 6 0
        4 5 9 8 0
        5 6 10 9 0
        6 7 11 10 0
        10
        0 1 0
        1 2 0
        2 3 0
        3 7 0
        7 11 0
        11 10 0
        10 9 0
        9 8 0
        8 4 0
        4 0 0
        """;

    private const string ромб_без_выреза =
        """
        8
        0 0
        1 0
        3 3
        3 4
        3 -4
        3 -3
        5 0
        6 0
        5
        0 1 2 3 0
        0 4 5 1 0
        2 6 7 3 0
        4 7 6 5 0
        1 5 6 2 0
        4
        0 4 0
        4 7 0
        7 3 0
        3 0 0
    
        """;

    private const string ромб_c_вырезом =
        """
        8
        0 0
        1 0
        3 3
        3 4
        3 -4
        3 -3
        5 0
        6 0
        4
        0 1 2 3 0
        0 4 5 1 0
        2 6 7 3 0
        4 7 6 5 0
        8
        0 4 0
        4 7 0
        7 3 0
        3 0 0
        1 5 0
        5 6 0
        6 2 0
        2 1 0
        """;

    private const string RefinedMesh =
        """
        24
        0 0 
        1 0
        3 3 
        3 4
        3 -4 
        3 -3
        5 0 
        6 0
        0.5 0
        2 1.5
        3 3.5
        1.5 2
        1.5 -2
        3 -3.5
        2 -1.5
        4 1.5
        5.5 0
        4.5 2
        4.5 -2
        4 -1.5
        1.75 1.75
        1.75 -1.75
        4.25 1.75
        4.25 -1.75
        16
        0 8 20 11 0
        8 1 9 20 0
        20 9 2 10 0
        11 20 10 3 0
        0 12 21 8 0
        12 4 13 21 0
        21 13 5 14 0
        8 21 14 1 0
        2 15 22 10 0
        15 6 16 22 0
        22 16 7 17 0
        10 22 17 3 0
        4 18 23 13 0
        18 7 16 23 0
        23 16 6 19 0
        13 23 19 5 0
        16
        0 12 0
        12 4 0
        4 18 0
        18 7 0
        7 17 0
        17 3 0
        3 11 0
        11 0 0
        1 14 0
        14 5 0
        5 19 0
        19 6 0
        6 15 0
        15 2 0
        2 9 0
        9 1 0
        """;

    private const string ТестИзПрошлого =
        """
        12
        0 0
        1 0
        1.5 0
        3.5 0
        0   0.1
        1   0.1
        1.5 0.1
        3.5 0.1
        0   1.1
        1   1.1
        1.5 1.1
        3.5 1.1
        6
        0 1 5 4 0
        1 2 6 5 0
        2 3 7 6 0
        4 5 9 8 0
        5 6 10 9 0
        6 7 11 10 0
        10
        0 1 0
        1 2 0
        2 3 0
        3 7 0
        7 11 0
        11 10 0
        10 9 0
        9 8 0
        8 4 0
        4 0 0
        """;

    private const string одна_внутренняя_точка =
        """
        9
        0 0
        1 0
        2 0
        0 1
        1 1
        2 1
        0 2
        1 2
        2 2
        4
        0 1 4 3 0
        1 2 5 4 0
        3 4 7 6 0
        4 5 8 7 0
        8
        0 1 0
        1 2 0
        2 5 0
        5 8 0
        7 8 0
        6 7 0
        0 3 0
        3 6 0
        """;
    private const string две_внутренние_точки =
        """
        12
        0 0
        1 0
        2 0
        3 0
        0 1
        1 1
        2 1
        3 1
        0 2
        1 2
        2 2
        3 2
        6
        0 1 5 4 0
        1 2 6 5 0
        2 3 7 6 0
        4 5 9 8 0
        5 6 10 9 0
        6 7 11 10 0
        10
        0 1 0
        1 2 0
        2 3 0 
        3 7 0
        7 11 0
        11 10 0
        10 9 0
        9 8 0
        8 4 0
        4 0 0
        """;
    private const string четыре_внутренние_точки =
            """
        16
        0 0
        1 0
        2 0
        3 0
        0 1
        1 1
        2 1
        3 1
        0 2
        1 2
        2 2
        3 2
        0 3
        1 3
        2 3
        3 3
        9
        0 1 5 4 0
        1 2 6 5 0
        2 3 7 6 0
        4 5 9 8 0
        5 6 10 9 0
        6 7 11 10 0
        8 9 13 12 0
        9 10 14 13 0
        10 11 15 14 0
        12
        0 1 0
        1 2 0
        2 3 0 
        3 7 0
        7 11 0
        11 15 0
        15 14 0
        14 13 0
        13 12 0
        12 8 0
        8 4 0
        4 0 0
        """;

    private const string четыре_внутренние_точки_произвольный =
        """
        8
        0 0
        2 -1
        6 1
        1 1
        2 0.5
        3.1 1
        2 2.5
        2 6
        5
        0 1 4 3 0
        1 2 5 4 0
        2 7 6 5 0
        0 3 6 7 0
        3 4 5 6 0
        4
        0 1 0
        1 2 0
        2 7 0 
        7 0 0
        """;

    private const string большая_четырехугольная_сетка =
        """
        22
        0 0
        2 0.5
        5 0
        8 0.5
        0.5 1
        2.5 2
        5.5 1
        6.5 1
        0.5 4
        1 4.5
        5.5 3
        6.5 2
        6.7 2.5
        1.5 6
        2.5 7
        4 6
        4.4 5.5
        7.1 3.5
        4.5 6
        6 7
        6.5 5.5
        6.5 3
        15
        0 1 5 4 0
        1 2 6 5 0
        2 3 7 6 0
        3 12 11 7 0
        3 17 21 12 0
        17 21 10 20 0
        20 19 18 10 0
        19 14 15 18 0
        14 13 9 5 0
        4 5 9 8 0
        5 6 10 14 0
        6 7 11 10 0
        11 12 21 10 0
        14 10 16 15 0
        15 16 10 18 0
        12
        0 1 0
        1 2 0
        2 3 0
        3 17 0
        17 20 0
        20 19 0
        19 14 0
        14 13 0
        13 9 0
        9 8 0
        8 4 0
        4 0 0
        """;
    private const string сетка_на_6_элементов_произвольные_внутренние_точки_прямые_внешние =
            """
            12
            0.0 0.0
            1.7 0.0
            4.3 0.0
            7.0 0.0
            0.0 1.0
            2.1 0.8
            4.0 1.3
            7.0 1.0
            0.0 2.2
            1.9 2.2
            4.6 2.2
            7.0 2.2
            6
            0 1 5 4 0
            1 2 6 5 0
            2 3 7 6 0
            4 5 9 8 0
            5 6 10 9 0
            6 7 11 10 0
            10
            0 1 0
            1 2 0
            2 3 0
            3 7 0
            7 11 0
            11 10 0
            10 9 0
            9 8 0
            8 4 0
            4 0 0
            """;

    private const string четыре_внутренние_точки_произвольные_внутренние_точки_прямые_внешние =
        """
            16
            0.0 0.0
            1.2 0.0
            2.9 0.0
            5.0 0.0
            0.0 1.0
            1.4 0.8
            2.7 1.2
            5.0 1.0
            0.0 2.1
            1.1 2.3
            3.1 1.9
            5.0 2.2
            0.0 3.3
            1.3 3.3
            3.0 3.3
            5.0 3.3
            9
            0 1 5 4 0
            1 2 6 5 0
            2 3 7 6 0
            4 5 9 8 0
            5 6 10 9 0
            6 7 11 10 0
            8 9 13 12 0
            9 10 14 13 0
            10 11 15 14 0
            12
            0 1 0
            1 2 0
            2 3 0
            3 7 0
            7 11 0
            11 15 0
            15 14 0
            14 13 0
            13 12 0
            12 8 0
            8 4 0
            4 0 0
            """;

    private const string сетка_4_внутренних_КЭ_произвольные_внутренние_ребра =
    """
    25
    0.0 0.0
    2.0 0.0
    4.2 0.0
    6.1 0.0
    8.0 0.0
    0.0 0.9
    2.1 0.9
    4.0 1.2
    6.2 1.0
    8.0 1.0
    0.0 2.3
    1.8 2.1
    4.3 2.5
    6.0 2.2
    8.0 2.3
    0.0 3.6
    2.2 3.4
    4.1 3.7
    5.8 3.5
    8.0 3.6
    0.0 4.5
    2.0 4.5
    4.2 4.5
    6.1 4.5
    8.0 4.5
    16
    0 1 6 5 0
    1 2 7 6 0
    2 3 8 7 0
    3 4 9 8 0
    5 6 11 10 0
    6 7 12 11 0
    7 8 13 12 0
    8 9 14 13 0
    10 11 16 15 0
    11 12 17 16 0
    12 13 18 17 0
    13 14 19 18 0
    15 16 21 20 0
    16 17 22 21 0
    17 18 23 22 0
    18 19 24 23 0
    16
    0 1 0
    1 2 0
    2 3 0
    3 4 0
    4 9 0
    9 14 0
    14 19 0
    19 24 0
    24 23 0
    23 22 0
    22 21 0
    21 20 0
    20 15 0
    15 10 0
    10 5 0
    5 0 0
    """;
    private const string сетка_4_внутренних_КЭ_произвольные_все_ребра =
       """
25
0.000 0.000
2.125 0.150
4.250 0.300
6.375 0.450
8.500 0.600
0.250 1.175
2.450 1.100
4.350 1.550
6.700 1.420
8.725 1.750
0.500 2.350
2.400 2.600
4.900 2.350
6.650 2.850
8.950 2.900
0.750 3.525
2.950 3.400
4.750 4.000
7.050 3.700
9.175 4.050
1.000 4.700
3.100 4.825
5.200 4.950
7.300 5.075
9.400 5.200
16
0 1 6 5 0
1 2 7 6 0
2 3 8 7 0
3 4 9 8 0
5 6 11 10 0
6 7 12 11 0
7 8 13 12 0
8 9 14 13 0
10 11 16 15 0
11 12 17 16 0
12 13 18 17 0
13 14 19 18 0
15 16 21 20 0
16 17 22 21 0
17 18 23 22 0
18 19 24 23 0
16
0 1 0
1 2 0
2 3 0
3 4 0
4 9 0
9 14 0
14 19 0
19 24 0
24 23 0
23 22 0
22 21 0
21 20 0
20 15 0
15 10 0
10 5 0
5 0 0
""";
    public enum TimeSchemeKind
    {
        ForwardEuler,
        BackwardEuler,
        ExplicitThreeLayer,
        ImplicitThreeLayer
    }

    public static IEnumerable<object[]> TestCases()
    {
        var meshes = new[]
        {
            квадратик_эталонный,
            сетка_на_6_элементов,
            ромб_без_выреза,
            ромб_c_вырезом,
            RefinedMesh,
            ТестИзПрошлого,
            одна_внутренняя_точка,
            две_внутренние_точки,
            четыре_внутренние_точки,
            четыре_внутренние_точки_произвольный,
            большая_четырехугольная_сетка,
            сетка_на_6_элементов_произвольные_внутренние_точки_прямые_внешние,
            четыре_внутренние_точки_произвольные_внутренние_точки_прямые_внешние,
            сетка_4_внутренних_КЭ_произвольные_внутренние_ребра,
            сетка_4_внутренних_КЭ_произвольные_все_ребра
        };

        var schemes = new[]
        {
            TimeSchemeKind.ForwardEuler,
            TimeSchemeKind.BackwardEuler,
            TimeSchemeKind.ExplicitThreeLayer,
            TimeSchemeKind.ImplicitThreeLayer
        };

        foreach (var mesh in meshes)
            foreach (var scheme in schemes)
                yield return new object[] { mesh, scheme };
    }
    [Theory]
    [MemberData(nameof(TestCases))]
    public void HermiteQuadrangle_Parabolic_Dirichlet_IsErrorWithinTolerance(string meshText, TimeSchemeKind schemeKind)
    {
        static double analyticSolution(Vector2D p, double t) => t * p.X;

        ITimeScheme[] timeSchemes = schemeKind switch
        {
            TimeSchemeKind.ForwardEuler => [TimeSchemes.ForwardEuler],
            TimeSchemeKind.BackwardEuler => [TimeSchemes.BackwardEuler],
            TimeSchemeKind.ExplicitThreeLayer => [TimeSchemes.ForwardEuler, TimeSchemes.ExplicitThreeLayer],
            TimeSchemeKind.ImplicitThreeLayer => [TimeSchemes.BackwardEuler, TimeSchemes.ImplicitThreeLayer],
            _ => throw new ArgumentOutOfRangeException(nameof(schemeKind))
        };

        var mesh = new StringReader(meshText).ReadMesh2D(
            coordinateSystem: IdentityTransform<Vector2D>.Instance,
            FiniteElements.Quadrangle.Hermite,
            FiniteElements.Segment.Hermite
        );

        var problem = new HyperbolicProblem2D(
            Materials:
            [
                new(
                Lambda: (p, t) => 1.0,
                Xi:     (p, t) => 0.0,
                Sigma:  (p, t) => 1.0,
                Source: (p, t) => p.X
            )
            ],
            InitialCondition: p => 0.0,
            BoundaryConditions:
            [
                new BoundaryCondition2D.Dirichlet(
                Value: (p, t) => analyticSolution(p, t)
            )
            ],
            Mesh: mesh
        );

        var solver = new ParabolicSolver2D(
            timeSchemes,
            DenseMatrix.Factory,
            NumericItegrator2D.Instance,
            new PCGSolver(m => IdentityPreconditioner.Instance)
        );

        double[] timePoints = [0.0, 0.5, 1.0];

        var solutions = solver
            .Solve(problem, timePoints, new ISolver.Params(1e-15, 10000))
            .ToList();

        var finalSolution = solutions[^1];
        const double finalTime = 1.0;

        for (int i = 0; i < mesh.VertexCount; i++)
        {
            var p = mesh[i];
            Assert.Equal(analyticSolution(p, finalTime), finalSolution.Evaluate(p), Eps);
        }
    }
}
