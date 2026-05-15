using Model.Core.CoordinateSystem;
using Model.Core.Matrix;
using Model.Core.Solver;
using Model.Core.Solver.Precondition;
using Model.Fem.Basis;
using Model.Fem.Elements;
using Model.Fem.Integrator;
using Model.Fem.Mesh;
using Model.Fem.Problem;
using Telma;

namespace UnitTests.AnalyticalIntegrator;

public class AnalyticalIntegratorTests
{
    [Fact]
    public void CorrectPolynomialSum()
    {
        //(x + 1) + (-x + 2y^2 + 3) = 2y^2 + 4
        var p1 = new Polynomial2D() { Summands = { [(1, 0)] = 1, [(0, 0)] = 1 } };
        var p2 = new Polynomial2D() { Summands = { [(1, 0)] = -1, [(0, 2)] = 2, [(0, 0)] = 3 } };

        var p3 = Polynomial2D.Sum(p1, p2);
        p1.Add(p2);

        Assert.Equal(2, p1.Summands.Count); //delete 0 value check
        Assert.Equal(2, p1.Summands[(0, 2)]); // 2y^2
        Assert.Equal(4, p1.Summands[(0, 0)]); // const
        Assert.Equal(2, p1.Degree); // degree

        Assert.Equal(2, p3.Summands.Count); //delete 0 value check
        Assert.Equal(2, p3.Summands[(0, 2)]); // 2y^2
        Assert.Equal(4, p3.Summands[(0, 0)]); // const
        Assert.Equal(2, p3.Degree); // degree
    }

    [Fact]
    public void CorrectPolynomialMult()
    {
        //(x + 1) * (-x + 2y^2 + 3) = -x^2 + 2xy^2 + 2x + 2y^2 + 4
        var p1 = new Polynomial2D() { Summands = { [(1, 0)] = 1, [(0, 0)] = 1 } };
        var p2 = new Polynomial2D() { Summands = { [(1, 0)] = -1, [(0, 2)] = 2, [(0, 0)] = 3 } };

        var p3 = Polynomial2D.Mult(p1, p2);
        p1.Mult(p2);

        Assert.Equal(5, p1.Summands.Count); 
        Assert.Equal(2, p1.Summands[(1, 2)]); // 2y^2
        Assert.Equal(2, p1.Summands[(1, 0)]); // 2x
        Assert.Equal(3, p1.Summands[(0, 0)]); // const
        Assert.Equal(3, p1.Degree); // degree

        Assert.Equal(5, p3.Summands.Count);
        Assert.Equal(2, p3.Summands[(1, 2)]); // 2y^2
        Assert.Equal(2, p3.Summands[(1, 0)]); // 2x
        Assert.Equal(3, p3.Summands[(0, 0)]); // const
        Assert.Equal(3, p3.Degree); // degree
    }

    [Fact]
    public void CorrectPolynomialGradient()
    {
        // d/dx (-x + 2y^2 + 3) = -1
        // d/dy (-x + 2y^2 + 3) = 4y
        var p2 = new Polynomial2D() { Summands = { [(1, 0)] = -1, [(0, 2)] = 2, [(0, 0)] = 3 } };
        var g = p2.Gradient();

        var dx = g[0];
        var dy = g[1];

        Assert.Equal(-1, dx.Summands[(0, 0)]);
        Assert.Single(dx.Summands);
        Assert.Equal(4, dy.Summands[(0, 1)]);
        Assert.Single(dy.Summands);
    }

    [Fact]
    public void CorrectPolynomialValue()
    {
        // (-x + 2y^2 + 3)|(1.5, 1) = 3.5
        var p2 = new Polynomial2D() { Summands = { [(1, 0)] = -1, [(0, 2)] = 2, [(0, 0)] = 3 } };
        var point = new Vector2D(1.5, 1.0);
        double res = p2.Value(point);

        Assert.Equal(3.5, res, 1e-13);
    }

    [Fact]
    public void CorrectPolynomialOnTriangle()
    {
        // Int((-x + 2y^2 + 3)*1) = 1.5
        var J = new Polynomial2D() { Summands = { [(0, 0)] = 1 } };
        var p2 = new Polynomial2D() { Summands = { [(1, 0)] = -1, [(0, 2)] = 2, [(0, 0)] = 3 } };
        var p = Polynomial2D.Mult(J, p2);
        double res = AnalyticalIntegration.IntegrateTriangle(p);

        Assert.Equal(1.5, res, 1e-13);
    }

    [Fact]
    public void CorrectPolynomialOnQuadrangle()
    {
        // Int((-x + 2y^2 + 3)*1) = 19/6
        var J = new Polynomial2D() { Summands = { [(0, 0)] = 1 } };
        var p2 = new Polynomial2D() { Summands = { [(1, 0)] = -1, [(0, 2)] = 2, [(0, 0)] = 3 } };
        var p = Polynomial2D.Mult(J, p2);
        double res = AnalyticalIntegration.IntegrateQuadrangle(p);

        Assert.Equal(19.0/6, res, 1e-13);
    }

    [Fact]
    public void CorrectPolynomialOnBound()
    {
        // Int((-x + 2x^2 + 3)*6) = 19
        var J = new Polynomial1D() { Summands = { [0] = 6 } };
        var p2 = new Polynomial1D() { Summands = { [1] = -1, [2] = 2, [0] = 3 } };
        var p = Polynomial1D.Mult(J, p2);
        double res = AnalyticalIntegration.IntegrateBoundary(p);

        Assert.Equal(19.0, res, 1e-13);
    }
}

//--------------------------------------------------------------------------------------------------------------------//

public class AnalyticIntegratorOnTriangleTests
{
    private static IFiniteElement2D BuildTriangle(Vector2D a, Vector2D b, Vector2D c, IFiniteElementFactory2D factory)
    {
        var mesh = new Mesh2D();
        mesh.AddVertices(a, b, c);
        return mesh.AddElement(factory, [0, 1, 2], materialIndex: 0);
    }

    private static IBoundaryElement2D BuildSegment(Vector2D a, Vector2D b, IBoundaryElementFactory2D factory)
    {
        var mesh = new Mesh2D();
        mesh.AddVertices(a, b);
        return mesh.AddBoundary(factory, [0, 1], boundaryIndex: 0);
    }

    [Fact]
    public void MassMatrixOnStandardTriangleMatchesReference()
    {
        // Треугольник (0,0), (1,0), (0,1), γ = 1, линейные базисные функции
        // M = (1/24) * [[2,1,1],[1,2,1],[1,1,2]]
        var element = BuildTriangle(new(0, 0), new(1, 0), new(0, 1), FiniteElements.Triangle.Linear);
        var mass = AnalyticIntegrator2D.Instance.CalculateLocalMass(element, _ => 1.0);

        Assert.Equal(2.0 / 24, mass[0, 0], 1e-13);
        Assert.Equal(2.0 / 24, mass[1, 1], 1e-13);
        Assert.Equal(2.0 / 24, mass[2, 2], 1e-13);
        Assert.Equal(1.0 / 24, mass[0, 1], 1e-13);
        Assert.Equal(1.0 / 24, mass[0, 2], 1e-13);
        Assert.Equal(1.0 / 24, mass[1, 2], 1e-13);
    }

    [Fact]
    public void StiffnessMatrixOnStandardTriangleMatchesReference()
    {
        // Треугольник (0,0), (1,0), (0,1), λ = 1, линейные базисные функции
        // G = (1/2) * [[2,-1,-1],[-1,1,0],[-1,0,1]]
        var element = BuildTriangle(new(0, 0), new(1, 0), new(0, 1), FiniteElements.Triangle.Linear);
        var stiffness = AnalyticIntegrator2D.Instance.CalculateLocalStiffness(element, _ => 1.0);

        Assert.Equal(1.0, stiffness[0, 0], 1e-13);
        Assert.Equal(0.5, stiffness[1, 1], 1e-13);
        Assert.Equal(0.5, stiffness[2, 2], 1e-13);
        Assert.Equal(-0.5, stiffness[0, 1], 1e-13);
        Assert.Equal(-0.5, stiffness[0, 2], 1e-13);
        Assert.Equal(0.0, stiffness[1, 2], 1e-13);
    }

    [Fact]
    public void MassMatrixOnTriangleMatchesReference()
    {
        // Треугольник (1,2), (3,2), (1,5)
        // M = (1/4) * [[2,1,1],[1,2,1],[1,1,2]]
        var element = BuildTriangle(new(1, 2), new(3, 2), new(1, 5), FiniteElements.Triangle.Linear);
        var mass = AnalyticIntegrator2D.Instance.CalculateLocalMass(element, _ => 1.0);

        Assert.Equal(2.0 / 4, mass[0, 0], 1e-13);
        Assert.Equal(2.0 / 4, mass[1, 1], 1e-13);
        Assert.Equal(2.0 / 4, mass[2, 2], 1e-13);
        Assert.Equal(1.0 / 4, mass[0, 1], 1e-13);
        Assert.Equal(1.0 / 4, mass[0, 2], 1e-13);
        Assert.Equal(1.0 / 4, mass[1, 2], 1e-13);
    }

    [Theory]
    [InlineData(nameof(FiniteElements.Triangle.Linear))]
    [InlineData(nameof(FiniteElements.Triangle.HierarchicalQuadratic))]
    [InlineData(nameof(FiniteElements.Triangle.HierarchicalCubic))]
    [InlineData(nameof(FiniteElements.Triangle.LagrangeCubic))]
    public void MassAndStiffnessAgreeWithNumericForConstantLambda(string factoryName)
    {
        var factory = factoryName switch
        {
            nameof(FiniteElements.Triangle.Linear) => FiniteElements.Triangle.Linear,
            nameof(FiniteElements.Triangle.HierarchicalQuadratic) => FiniteElements.Triangle.HierarchicalQuadratic,
            nameof(FiniteElements.Triangle.HierarchicalCubic) => FiniteElements.Triangle.HierarchicalCubic,
            nameof(FiniteElements.Triangle.LagrangeCubic) => FiniteElements.Triangle.LagrangeCubic,
            _ => throw new ArgumentException(factoryName),
        };

        var element = BuildTriangle(new(1, 2), new(4, 2), new(2, 6), factory);

        var massA = AnalyticIntegrator2D.Instance.CalculateLocalMass(element, _ => 2.5);
        var massN = NumericItegrator2D.Instance.CalculateLocalMass(element, _ => 2.5);
        var stiffA = AnalyticIntegrator2D.Instance.CalculateLocalStiffness(element, _ => 0.7);
        var stiffN = NumericItegrator2D.Instance.CalculateLocalStiffness(element, _ => 0.7);

        Assert.Equal(massN.Size, massA.Size);
        for (int i = 0; i < massA.Size; i++)
            for (int j = 0; j < massA.Size; j++)
            {
                Assert.Equal(massN[i, j], massA[i, j], 1e-10);
                Assert.Equal(stiffN[i, j], stiffA[i, j], 1e-10);
            }
    }

    [Fact]
    public void BoundaryMassOnHorizontalSegmentMatchesReference()
    {
        // Сегмент (0,0)-(2,0), γ=1
        // M = (1/3) * [[2,1],[1,2]]
        var element = BuildSegment(new(0, 0), new(2, 0), FiniteElements.Segment.Linear);
        var mass = AnalyticIntegrator2D.Instance.CalculateLocalMass(element, _ => 1.0);

        Assert.Equal(2.0 / 3, mass[0, 0], 1e-13);
        Assert.Equal(2.0 / 3, mass[1, 1], 1e-13);
        Assert.Equal(1.0 / 3, mass[0, 1], 1e-13);
        Assert.Equal(1.0 / 3, mass[1, 0], 1e-13);
    }

    [Fact]
    public void BoundaryMassOnSegmentAgreesWithNumeric()
    {
        // Сегмент (1,1)-(4,5)
        var element = BuildSegment(new(1, 1), new(4, 5), FiniteElements.Segment.Linear);
        var massA = AnalyticIntegrator2D.Instance.CalculateLocalMass(element, _ => 1.0);
        var massN = NumericItegrator2D.Instance.CalculateLocalMass(element, _ => 1.0);

        for (int i = 0; i < 2; i++)
        {
            for (int j = 0; j < 2; j++)
            {
                Assert.Equal(massN[i, j], massA[i, j], 1e-12);
            }
        }
    }

    [Fact]
    public void LoadDelegatesToNumeric()
    {
        var element = BuildTriangle(new(0, 0), new(1, 0), new(0, 1), FiniteElements.Triangle.Linear);

        var loadA = new double[3];
        var loadN = new double[3];
        AnalyticIntegrator2D.Instance.CalculateLocalLoad(element, p => p.X + p.Y, loadA);
        NumericItegrator2D.Instance.CalculateLocalLoad(element, p => p.X + p.Y, loadN);

        for (int i = 0; i < 3; i++)
        { 
            Assert.Equal(loadN[i], loadA[i], 1e-13);
        }
    }

    [Fact]
    public void NonTriangleElementThrowsNotSupported()
    {
        var mesh = new Mesh2D();
        mesh.AddVertices(new Vector2D(0, 0), new(1, 0), new(1, 1), new(0, 1));
        var quad = mesh.AddElement(FiniteElements.Quadrangle.Bilinear, [0, 1, 2, 3], 0);

        Assert.Throws<NotSupportedException>(() => AnalyticIntegrator2D.Instance.CalculateLocalMass(quad, _ => 1.0));
        Assert.Throws<NotSupportedException>(() => AnalyticIntegrator2D.Instance.CalculateLocalStiffness(quad, _ => 1.0));
    }

    [Fact]
    public void NonCartesianMeshThrowsNotSupported()
    {
        var mesh = new Mesh2D(PolarCoordinateSystem.Instance);
        mesh.AddVertices(new Vector2D(0, 0), new(1, 0), new(0, 1));
        var element = mesh.AddElement(FiniteElements.Triangle.Linear, [0, 1, 2], 0);

        Assert.Throws<NotSupportedException>(() => AnalyticIntegrator2D.Instance.CalculateLocalMass(element, _ => 1.0));
    }
}

//--------------------------------------------------------------------------------------------------------------------//
// Тесты взяты старые (кубический лагранж базис), численный интегратор заменён на аналитический
public class AnalyticIntegratorOnTriangleEllipticProblemTests
{
    public static string TriangleWithAllBC => """
        3
        0 0
        2 0
        0 2
        1
        0 1 2 0
        3
        0 1 0
        1 2 1
        0 2 2
        """;
    [Fact]
    public void LagrangeCubicWithAllBCWithQuadraticFunc()
    {
        var mesh = new StringReader(TriangleWithAllBC).ReadMesh2D(
            coordinateSystem: IdentityTransform<Vector2D>.Instance,
            FiniteElements.Triangle.LagrangeCubic,
            FiniteElements.Segment.LagrangeCubic
        );
        static double u(Vector2D p) => p.X * p.X;
        var problem = new EllipticProblem2D(
            Materials: [new(
                Lambda: _ => 1.0,
                Gamma: _ => 1.0,
                Source: p => -2 + p.X * p.X
            )],
            BoundaryConditions: [
                new BoundaryCondition2D.Dirichlet(Value: (p,_) => u(p)),
                new BoundaryCondition2D.Neumann(Flux: (p,_) => 2.0 * p.X/Math.Sqrt(2.0)),
                new BoundaryCondition2D.Robin(Beta: (p,_) => 1.0,UBeta: (p,_) => -2.0 * p.X),
            ],
            mesh
        );

        var solver = new EllipticSolver2D(
            DenseMatrix.Factory,
            AnalyticIntegrator2D.Instance,
            new PCGSolver(m => IdentityPreconditioner.Instance)
        );

        var solution = solver.Solve(problem, new ISolver.Params(1e-13, 10000));
        for (int i = 0; i < mesh.VertexCount; i++)
        {
            var point = mesh[i];
            Console.WriteLine(point);
            Assert.Equal(u(point), solution.Evaluate(point), 1e-12);
        }

    }
    private static string TriangleWithDirichletRobin => """
        3
        0 0
        2 0
        0 2
        1
        0 1 2 0
        3
        0 1 0
        1 2 1
        0 2 2
        """;
    [Fact]
    public void LagrangeCubicWithDirichletRobinWithQuadraticFunc2()
    {
        var mesh = new StringReader(TriangleWithDirichletRobin).ReadMesh2D(
            coordinateSystem: IdentityTransform<Vector2D>.Instance,
            FiniteElements.Triangle.LagrangeCubic,
            FiniteElements.Segment.LagrangeCubic
        );
        static double u(Vector2D p) => p.X * p.X;
        var problem = new EllipticProblem2D(
            Materials: [new(
                Lambda: _ => 1.0,
                Gamma: _ => 1.0,
                Source: p => -2 + p.X * p.X
            )],
            BoundaryConditions: [
                new BoundaryCondition2D.Dirichlet(Value: (p,_) => u(p)),
                new BoundaryCondition2D.Robin(Beta: (p,_) => 1.0,UBeta: (p,_) => 2.0*p.X/Math.Sqrt(2.0) + p.X*p.X),
                new BoundaryCondition2D.Robin(Beta: (p,_) => 1.0,UBeta: (p,_) => -2.0 * p.X),
            ],
            mesh
        );

        var solver = new EllipticSolver2D(
            DenseMatrix.Factory,
            AnalyticIntegrator2D.Instance,
            new PCGSolver(m => IdentityPreconditioner.Instance)
        );

        var solution = solver.Solve(problem, new ISolver.Params(1e-13, 10000));
        for (int i = 0; i < mesh.VertexCount; i++)
        {
            var point = mesh[i];
            Console.WriteLine(point);
            Assert.Equal(u(point), solution.Evaluate(point), 1e-12);
        }
    }

    private static string TwoNotMasterTriangles => """
        4
        0 0 
        2 0
        0 1
        2 1
        2
        0 1 2 0
        1 2 3 0
        4
        0 1 0 
        2 3 2 
        0 2 0 
        1 3 1 
        """;
    [Fact]
    public void LagrangeCubicMeshWithAllBCWithQuadraticFunc()
    {
        var mesh = new StringReader(TwoNotMasterTriangles).ReadMesh2D(
            coordinateSystem: IdentityTransform<Vector2D>.Instance,
            FiniteElements.Triangle.LagrangeCubic,
            FiniteElements.Segment.LagrangeCubic
        );
        static double u(Vector2D p) => p.X * p.X;
        var problem = new EllipticProblem2D(
            Materials: [new(
                Lambda: _ => 1.0,
                Gamma: _ => 1.0,
                Source: p => -2 + p.X*p.X
            )],
            BoundaryConditions: [
                new BoundaryCondition2D.Dirichlet(Value: (p,_) => u(p)),
                new BoundaryCondition2D.Neumann(Flux: (p,_) => 2.0 * p.X),
                new BoundaryCondition2D.Robin(Beta: (p,_) => 1.0,UBeta: (p,_) => p.X*p.X),
            ],
            mesh
        );

        var solver = new EllipticSolver2D(
            DenseMatrix.Factory,
            AnalyticIntegrator2D.Instance,
            new PCGSolver(m => IdentityPreconditioner.Instance)
        );

        var solution = solver.Solve(problem, new ISolver.Params(1e-13, 10000));
        for (int i = 0; i < mesh.VertexCount; i++)
        {
            var point = mesh[i];
            Console.WriteLine(point);
            Assert.Equal(u(point), solution.Evaluate(point), 1e-12);
        }
    }
    [Fact]
    public void CSRLagrangeCubicMeshWithAllBCWithQuadraticFunc()
    {
        var mesh = new StringReader(TwoNotMasterTriangles).ReadMesh2D(
            coordinateSystem: IdentityTransform<Vector2D>.Instance,
            FiniteElements.Triangle.LagrangeCubic,
            FiniteElements.Segment.LagrangeCubic
        );
        static double u(Vector2D p) => p.X * p.X;
        var problem = new EllipticProblem2D(
            Materials: [new(
                Lambda: _ => 1.0,
                Gamma: _ => 1.0,
                Source: p => -2 + p.X*p.X
            )],
            BoundaryConditions: [
                new BoundaryCondition2D.Dirichlet(Value: (p,_) => u(p)),
                new BoundaryCondition2D.Neumann(Flux: (p,_) => 2.0 * p.X),
                new BoundaryCondition2D.Robin(Beta: (p,_) => 1.0,UBeta: (p,_) => p.X*p.X),
            ],
            mesh
        );

        var solver = new EllipticSolver2D(
            CsrMatrix.Factory,
            AnalyticIntegrator2D.Instance,
            new PCGSolver(m => IdentityPreconditioner.Instance)
        );

        var solution = solver.Solve(problem, new ISolver.Params(1e-13, 10000));
        for (int i = 0; i < mesh.VertexCount; i++)
        {
            var point = mesh[i];
            Console.WriteLine(point);
            Assert.Equal(u(point), solution.Evaluate(point), 1e-12);
        }
    }
}

//--------------------------------------------------------------------------------------------------------------------//
// Тесты взяты старые (линейный и иерархический квадратичный базис), численный интегратор заменён на аналитический
public class AnalyticIntegratorOnTriangleParabolicProblemTests
{
    private const string TriangleWithAllBC = """
        3
        0 0
        3 0
        0 1
        1
        0 1 2 0
        3
        0 1 0
        1 2 1
        2 0 2
        """;
    private const string TestMesh1 = """
        3
        0 0
        3 0
        0 1
        1
        0 1 2 0
        3
        0 1 0
        1 2 0
        2 0 0
        """;

    [Fact]
    public void AllBCWithLinearTimeFuncExplicitTwoLayer()
    {
        static double analyticSolution(Vector2D p, double t) => t;

        var mesh = new StringReader(TriangleWithAllBC).ReadMesh2D(
            coordinateSystem: IdentityTransform<Vector2D>.Instance,
            FiniteElements.Triangle.Linear,
            FiniteElements.Segment.Linear
        );

        var problem = new HyperbolicProblem2D(
            Materials: [new(
                    Lambda: (p,t) => 1.0,
                    Sigma: (p,t) => 1.0,
                    Xi: (p,t) => 0.0,
                    Source: (p,t) => 1.0
                )],
            InitialCondition: p => analyticSolution(p, 0),
            BoundaryConditions: [
                new BoundaryCondition2D.Dirichlet(Value: analyticSolution),
                new BoundaryCondition2D.Neumann(Flux: (p,t) => 0.0),
                new BoundaryCondition2D.Robin(Beta: (p,t) => 2.0, UBeta: (p,t) => t)
            ],
            mesh
        );

        var solver = new ParabolicSolver2D(
            [TimeSchemes.ExplicitTwoLayers],
            CsrMatrix.Factory,
            AnalyticIntegrator2D.Instance,
            new PCGSolver(m => IdentityPreconditioner.Instance)
        );
        double[] timeLayers = [0, 0.1, 0.2, 0.3, 0.4];

        var solutions = solver.Solve(problem, timeLayers, new ISolver.Params(1e-13, 10000));

        int timeId = 1;
        foreach (var solution in solutions)
        {
            double currentTime = timeLayers[timeId];
            for (int i = 0; i < mesh.VertexCount; i++)
            {
                var point = mesh[i];
                Assert.Equal(analyticSolution(point, currentTime), solution.Evaluate(point), 1e-12);
            }
            timeId++;
        }
    }
    [Fact]
    public void AllBCWithLinearTimeFuncImplicitThreeLayer()
    {
        static double analyticSolution(Vector2D p, double t) => t;

        var mesh = new StringReader(TriangleWithAllBC).ReadMesh2D(
            coordinateSystem: IdentityTransform<Vector2D>.Instance,
            FiniteElements.Triangle.Linear,
            FiniteElements.Segment.Linear
        );

        var problem = new HyperbolicProblem2D(
            Materials: [new(
                    Lambda: (p,t) => 1.0,
                    Sigma: (p,t) => 1.0,
                    Xi: (p,t) => 0.0,
                    Source: (p,t) => 1.0
                )],
            InitialCondition: p => analyticSolution(p, 0),
            BoundaryConditions: [
                new BoundaryCondition2D.Dirichlet(Value: (p, t) => analyticSolution(p,t)),
                new BoundaryCondition2D.Neumann(Flux: (p,t) => 0.0),
                new BoundaryCondition2D.Robin(Beta: (p,t) => 2.0,UBeta: (p,t) => t)
            ],
            mesh
        );

        var solver = new ParabolicSolver2D([TimeSchemes.ImplicitTwoLayers, TimeSchemes.ImplicitThreeLayer],
            CsrMatrix.Factory,
            AnalyticIntegrator2D.Instance,
            new PCGSolver(m => IdentityPreconditioner.Instance)
        );
        double[] timeLayers = [0, 0.1, 0.2, 0.3, 0.4];

        var solutions = solver.Solve(problem, timeLayers, new ISolver.Params(1e-13, 10000));

        int timeId = 1;
        foreach (var solution in solutions)
        {
            double currentTime = timeLayers[timeId];
            for (int i = 0; i < mesh.VertexCount; i++)
            {
                var point = mesh[i];
                Assert.Equal(analyticSolution(point, currentTime), solution.Evaluate(point), 1e-12);
            }
            timeId++;
        }
    }

    [Fact]
    public void DirichletWithLinearTimeFuncExplicitTwoLayer()
    {
        static double analyticSolution(Vector2D p, double t) => t;

        var mesh = new StringReader(TestMesh1).ReadMesh2D(
            coordinateSystem: IdentityTransform<Vector2D>.Instance,
            FiniteElements.Triangle.HierarchicalQuadratic,
            FiniteElements.Segment.HierarchicalQuadratic
        );

        var problem = new HyperbolicProblem2D(
            Materials: [new(
                    Lambda: (p,t) => 1.0,
                    Sigma: (p,t) => 1.0,
                    Xi: (p,t) => 0.0,
                    Source: (p,t) => 1.0
                )],
            InitialCondition: p => analyticSolution(p, 0),
            BoundaryConditions: [
                new BoundaryCondition2D.Dirichlet(Value: (p, t) => analyticSolution(p,t))
            ],
            mesh
        );

        var solver = new ParabolicSolver2D([TimeSchemes.ExplicitTwoLayers],
            CsrMatrix.Factory,
            AnalyticIntegrator2D.Instance,
            new PCGSolver(m => IdentityPreconditioner.Instance)
        );
        double[] timeLayers = [0, 1, 2, 3, 4, 5];


        var solutions = solver.Solve(problem, timeLayers, new ISolver.Params(1e-13, 10000));

        int timeId = 1;
        foreach (var solution in solutions)
        {
            double currentTime = timeLayers[timeId];
            for (int i = 0; i < mesh.VertexCount; i++)
            {
                var point = mesh[i];
                Assert.Equal(analyticSolution(point, currentTime), solution.Evaluate(point), 1e-11);
            }
            timeId++;
        }
    }
}
