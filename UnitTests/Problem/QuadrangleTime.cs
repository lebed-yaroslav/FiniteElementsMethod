using Model.Core.CoordinateSystem;
using Model.Core.Matrix;
using Model.Core.Solver;
using Model.Core.Solver.Precondition;
using Model.Model.Elements;
using Model.Model.Integrator;
using Model.Model.Mesh;
using Model.Model.Problem;
using Telma;
using static Model.Core.CoordinateSystem.MatrixOperations;

namespace UnitTests.Problem;

public class ParabolicProblemQuadrangleTests
{
    private const double Dt = 0.25;
    private const double TStart = 0.0;
    private const double TEnd = 1.0;
    private const double Eps = 1e-10;

    private static double A(double t) => t;
    private static double dA(double t) => 1.0;

    private static void AssertScaled(double[] solution, double[] spatialValues)
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
            BoundaryConditions:
            [
                new BoundaryCondition2D.Dirichlet(
                    Value: (p, t) => A(t) * p.X
                )
            ],
            Mesh: mesh
        );

        var solver = new ParabolicSolver2D(
            DenseMatrix.Factory,
            NumericItegrator2D.Instance,
            new PCGSolver(m => IdentityPreconditioner.Instance)
        );

        var solution = solver.Solve(
            problem,
            TStart,
            TEnd,
            Dt,
            isImplicit,
            new ISolver.Params(1e-12, 10000)
        );

        AssertScaled(solution, [0.0, 1.0, 1.0, 0.0]);
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
                new HyperbolicMaterial2D(
                Lambda: (p, t) => 1.0,
                Xi:     (p, t) => 0.0,
                Sigma:  (p, t) => 1.0,
                Source: (p, t) => 0.0
            )
            ],
            BoundaryConditions:
            [
                new BoundaryCondition2D.Dirichlet(
                Value: (p, t) => 0.0
            )
            ],
            Mesh: mesh
        );

        var solver = new ParabolicSolver2D(
            DenseMatrix.Factory,
            NumericItegrator2D.Instance,
            new PCGSolver(m => IdentityPreconditioner.Instance)
        );

        var solution = solver.Solve(problem, TStart, TEnd, Dt, isImplicit, new ISolver.Params(1e-12, 10000));

        AssertScaled(solution, new double[4]);
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
                new HyperbolicMaterial2D(
                Lambda: (p, t) => 1.0,
                Xi:     (p, t) => 0.0,
                Sigma:  (p, t) => 1.0,
                Source: (p, t) => 0.0
            )
            ],
            BoundaryConditions:
            [
                new BoundaryCondition2D.Dirichlet(
                Value: (p, t) => 0.0
            )
            ],
            Mesh: mesh
        );

        var solver = new ParabolicSolver2D(
            DenseMatrix.Factory,
            NumericItegrator2D.Instance,
            new PCGSolver(m => IdentityPreconditioner.Instance)
        );

        var solution = solver.Solve(
            problem,
            TStart,
            TEnd,
            Dt,
            isImplicit,
            new ISolver.Params(1e-12, 10000)
        );

        AssertScaled(solution, new double[9]);
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
                new HyperbolicMaterial2D(
                    Lambda: (p, t) => 1.0,
                    Xi:     (p, t) => 0.0,
                    Sigma:  (p, t) => 1.0,
                    Source: (p, t) => dA(t) * p.X
                )
            ],
            BoundaryConditions:
            [
                new BoundaryCondition2D.Dirichlet(
                    Value: (p, t) => A(t) * p.X
                )
            ],
            Mesh: mesh
        );

        var solver = new ParabolicSolver2D(
            DenseMatrix.Factory,
            NumericItegrator2D.Instance,
            new PCGSolver(m => IdentityPreconditioner.Instance)
        );

        var solution = solver.Solve(
            problem,
            TStart,
            TEnd,
            Dt,
            isImplicit,
            new ISolver.Params(1e-12, 10000)
        );

        AssertScaled(solution, [-1.0, 3.0, 3.0, 0.0]);
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
            BoundaryConditions:
            [
                new BoundaryCondition2D.Dirichlet(
                    Value: (p, t) => A(t) * p.X
                )
            ],
            Mesh: mesh
        );

        var solver = new ParabolicSolver2D(
            DenseMatrix.Factory,
            NumericItegrator2D.Instance,
            new PCGSolver(m => IdentityPreconditioner.Instance)
        );

        var solution = solver.Solve(
            problem,
            TStart,
            TEnd,
            Dt,
            isImplicit,
            new ISolver.Params(1e-12, 10000)
        );

        AssertScaled(solution, [1.0, 2.0, 1.0, 0.0, 2.0, 0.0, 2.0, 1.0, 0.0]);
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
                new HyperbolicMaterial2D(
                    Lambda: (p, t) => 1.0,
                    Xi:     (p, t) => 0.0,
                    Sigma:  (p, t) => 1.0,
                    Source: (p, t) => dA(t) * p.X * p.X - 2.0 * A(t)
                )
            ],
            BoundaryConditions:
            [
                new BoundaryCondition2D.Dirichlet(
                    Value: (p, t) => A(t) * p.X * p.X
                )
            ],
            Mesh: mesh
        );

        var solver = new ParabolicSolver2D(
            DenseMatrix.Factory,
            NumericItegrator2D.Instance,
            new PCGSolver(m => IdentityPreconditioner.Instance)
        );

        var solution = solver.Solve(
            problem,
            TStart,
            TEnd,
            Dt,
            isImplicit,
            new ISolver.Params(1e-12, 10000)
        );

        AssertScaled(solution, [0.25, 0.0, 0.25, 1.0, 0.25, 0.0, 1.0, 1.0, 0.0]);
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
                new HyperbolicMaterial2D(
                    Lambda: (p, t) => 1.0,
                    Xi:     (p, t) => 0.0,
                    Sigma:  (p, t) => 1.0,
                    Source: (p, t) => dA(t) * p.X
                )
            ],
            BoundaryConditions:
            [
                new BoundaryCondition2D.Dirichlet(
                    Value: (p, t) => A(t) * p.X
                )
            ],
            Mesh: mesh
        );

        var solver = new ParabolicSolver2D(
            DenseMatrix.Factory,
            NumericItegrator2D.Instance,
            new PCGSolver(m => IdentityPreconditioner.Instance)
        );

        var solution = solver.Solve(
            problem,
            TStart,
            TEnd,
            Dt,
            isImplicit,
            new ISolver.Params(1e-12, 10000)
        );

        AssertScaled(solution, [0.5, 0.0, 0.5, 1.0, 0.5, 0.0, 1.0, 1.0, 0.0]);
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
                new HyperbolicMaterial2D(
                    Lambda: (p, t) => 1.0,
                    Xi:     (p, t) => 0.0,
                    Sigma:  (p, t) => 1.0,
                    Source: (p, t) => dA(t) * p.X
                )
            ],
            BoundaryConditions:
            [
                new BoundaryCondition2D.Dirichlet(
                    Value: (p, t) => A(t) * p.X
                )
            ],
            Mesh: mesh
        );

        var solver = new ParabolicSolver2D(
            DenseMatrix.Factory,
            NumericItegrator2D.Instance,
            new PCGSolver(m => IdentityPreconditioner.Instance)
        );

        var solution = solver.Solve(
            problem,
            TStart,
            TEnd,
            Dt,
            isImplicit,
            new ISolver.Params(1e-12, 10000)
        );

        AssertScaled(solution, [
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
            BoundaryConditions:
            [
                new BoundaryCondition2D.Dirichlet(
                    Value: (p, t) => A(t) * p.X * p.X
                )
            ],
            Mesh: mesh
        );

        var solver = new ParabolicSolver2D(
            DenseMatrix.Factory,
            NumericItegrator2D.Instance,
            new PCGSolver(m => IdentityPreconditioner.Instance)
        );

        var solution = solver.Solve(
            problem,
            TStart,
            TEnd,
            Dt,
            isImplicit,
            new ISolver.Params(1e-12, 10000)
        );

        AssertScaled(solution, [
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
                new HyperbolicMaterial2D(
                Lambda: (p, t) => 1.0,
                Xi:     (p, t) => 0.0,
                Sigma:  (p, t) => 1.0,
                Source: (p, t) => dA(t) * p.X * p.X - 2.0 * A(t)
            )
            ],
            BoundaryConditions:
            [
                new BoundaryCondition2D.Dirichlet(
                Value: (p, t) => A(t) * p.X * p.X
            )
            ],
            Mesh: mesh
        );

        var solver = new ParabolicSolver2D(
            DenseMatrix.Factory,
            NumericItegrator2D.Instance,
            new PCGSolver(m => IdentityPreconditioner.Instance)
        );

        var solution = solver.Solve(
            problem,
            TStart,
            TEnd,
            Dt,
            isImplicit,
            new ISolver.Params(1e-12, 10000)
        );

        AssertScaled(solution, [
            0.2500, 0.2500, 0.0625, 0.5625, 0.2500,
        0.0625, 0.5625, 0.0625, 0.5625, 0.5625,
        1.0000, 0.0000, 0.0625, 1.0000, 0.5625,
        0.0000, 0.0625, 1.0000, 0.2500, 0.0000,
        1.0000, 0.0000, 1.0000, 0.2500, 0.0000
        ]);
    }
}
