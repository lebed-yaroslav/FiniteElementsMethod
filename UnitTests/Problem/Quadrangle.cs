using Model.Core.CoordinateSystem;
using Model.Core.Matrix;
using Model.Core.Solver;
using Model.Core.Solver.Precondition;
using Model.Fem.Elements;
using Model.Fem.Mesh;
using Telma;

namespace UnitTests.Problem;

public class ElipticProblemQuadrangleHermiteTests
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

    [Theory]
    [InlineData(квадратик_эталонный)]
    [InlineData(сетка_на_6_элементов)]
    [InlineData(ромб_без_выреза)]
    [InlineData(ромб_c_вырезом)]
    [InlineData(RefinedMesh)]
    [InlineData(ТестИзПрошлого)]
    [InlineData(одна_внутренняя_точка)]
    [InlineData(две_внутренние_точки)]
    [InlineData(четыре_внутренние_точки)]
    [InlineData(четыре_внутренние_точки_произвольный)]
    [InlineData(большая_четырехугольная_сетка)]
    public void HermiteQuadrangle_Vertices_IsErrorWithinTolerance(string meshText)
    {
        static double analyticSolution(Vector2D p) => p.X;

        var mesh = new StringReader(meshText).ReadMesh2D(
            coordinateSystem: IdentityTransform<Vector2D>.Instance,
            FiniteElements.Quadrangle.Hermite,
            FiniteElements.Segment.Hermite
        );

        var problem = new EllipticProblem2D(
            Materials: [
                new(Lambda: _ => 1.0, Gamma: _ => 0, Source: _ => 0)
            ],
            BoundaryConditions: [
                new BoundaryCondition2D.Dirichlet((p, _) => analyticSolution(p))
            ],
            mesh
        );

        var solver = new EllipticSolver2D(
            DenseMatrix.Factory,
            NumericItegrator2D.Instance,
            new PCGSolver(m => IdentityPreconditioner.Instance)
        );

        var solution = solver.Solve(problem, new ISolver.Params(1e-15, 10000));

        for (int i = 0; i < mesh.VertexCount; i++)
        {
            var p = mesh[i];
            Assert.Equal(analyticSolution(p), solution.Evaluate(p), Eps);
        }
    }
}

public class EllipticProblemQuadrangleTests
{
    [Fact]
    public void МастерЭлемент_ЛинейнаяФункция_БиЛинейныйБазис()
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


        //  * х
        var mesh = new StringReader(TestMesh1).ReadMesh2D(
            coordinateSystem: IdentityTransform<Vector2D>.Instance,
            FiniteElements.Quadrangle.Bilinear,
            FiniteElements.Segment.Linear
        );

        var problem = new EllipticProblem2D(
            Materials: [new(
                Lambda: _ => 1.0,
                Gamma: _ => 0.0,
                Source: p => 0.0
            )],
            BoundaryConditions: [
                new BoundaryCondition2D.Dirichlet(Value: (p, _) => p.X)
            ],
            mesh
        );

        var solver = new EllipticSolver2D(
            DenseMatrix.Factory,
            NumericItegrator2D.Instance,
            new PCGSolver(m => IdentityPreconditioner.Instance)
        );

        var solution = solver.Solve(problem, new ISolver.Params(1e-12, 10000)).Coefficients;

        Assert.Equal(0.0, solution[0], 1e-10);
        Assert.Equal(1.0, solution[1], 1e-10);
        Assert.Equal(1.0, solution[2], 1e-10);
        Assert.Equal(0.0, solution[3], 1e-10);
    }
    [Fact]
    public void ПроизвольныйЭлемент_ЛинейнаяФункция_БиЛинейныйБазис()
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
        // x
        var mesh = new StringReader(TestMesh1).ReadMesh2D(
            coordinateSystem: IdentityTransform<Vector2D>.Instance,
            FiniteElements.Quadrangle.Bilinear,
            FiniteElements.Segment.Linear
        );

        var problem = new EllipticProblem2D(
            Materials: [new(
                Lambda: _ => 1.0,
                Gamma: _ => 0.0,
                Source: p => 0.0
            )],
            BoundaryConditions: [
                new BoundaryCondition2D.Dirichlet(Value: (p, _) => p.X)
            ],
            mesh
        );

        var solver = new EllipticSolver2D(
            DenseMatrix.Factory,
            NumericItegrator2D.Instance,
            new PCGSolver(m => IdentityPreconditioner.Instance)
        );

        var solution = solver.Solve(problem, new ISolver.Params(1e-12, 10000)).Coefficients;

        Assert.Equal(-1.0, solution[0], 1e-10);
        Assert.Equal(3.0, solution[1], 1e-10);
        Assert.Equal(3.0, solution[2], 1e-10);
        Assert.Equal(0.0, solution[3], 1e-10);
    }
    [Fact]
    public void равномернаяСеткаСВнутреннимУзлом_ЛинейнаяФункция_БиЛинейныйБазис()
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


        // x
        var mesh = new StringReader(TestMesh1).ReadMesh2D(
            coordinateSystem: IdentityTransform<Vector2D>.Instance,
            FiniteElements.Quadrangle.Bilinear,
            FiniteElements.Segment.Linear
        );

        var problem = new EllipticProblem2D(
            Materials: [new(
                Lambda: _ => 1.0,
                Gamma: _ => 0.0,
                Source: p => 0.0
            )],
            BoundaryConditions: [
                new BoundaryCondition2D.Dirichlet(Value: (p, _) => p.X)
            ],
            mesh
        );

        var solver = new EllipticSolver2D(
            DenseMatrix.Factory,
            NumericItegrator2D.Instance,
            new PCGSolver(m => IdentityPreconditioner.Instance)
        );

        var solution = solver.Solve(problem, new ISolver.Params(1e-12, 10000)).Coefficients;

        Assert.Equal(1.0, solution[0], 1e-10);
        Assert.Equal(2.0, solution[1], 1e-10);
        Assert.Equal(1.0, solution[2], 1e-10);
        Assert.Equal(0.0, solution[3], 1e-10);
        Assert.Equal(2.0, solution[4], 1e-10);
        Assert.Equal(0.0, solution[5], 1e-10);
        Assert.Equal(2.0, solution[6], 1e-10);
        Assert.Equal(1.0, solution[7], 1e-10);
        Assert.Equal(0.0, solution[8], 1e-10);
    }




    [Fact]
    public void МастерЭлемент_КвадратичнаяФункция_БиКвадратичныйБазис()
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


        //  * х
        var mesh = new StringReader(TestMesh1).ReadMesh2D(
            coordinateSystem: IdentityTransform<Vector2D>.Instance,
            FiniteElements.Quadrangle.LagrangeQuadratic,
            FiniteElements.Segment.LagrangeQuadratic
        );

        var problem = new EllipticProblem2D(
            Materials: [new(
                Lambda: _ => 1.0,
                Gamma: _ => 0.0,
                Source: p => -2.0
            )],
            BoundaryConditions: [
                new BoundaryCondition2D.Dirichlet(Value: (p, _) => p.X * p.X)
            ],
            mesh
        );

        var solver = new EllipticSolver2D(
            DenseMatrix.Factory,
            NumericItegrator2D.Instance,
            new PCGSolver(m => IdentityPreconditioner.Instance)
        );

        var solution = solver.Solve(problem, new ISolver.Params(1e-12, 10000)).Coefficients;

        var real_solutin = new double[] { 0.25,
                                        0,
                                        0.25,
                                        1,
                                        0.25,
                                        0,
                                        1,
                                        1,
                                        0, };
        for (int i = 0; i < solution.Length; i++)
        {
            Assert.Equal(real_solutin[i], solution[i], 1e-10);
        }
    }
    [Fact]
    public void МастерЭлемент_ЛинейнаяФункция_БиКвадратичныйБазис()
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


        //  * х
        var mesh = new StringReader(TestMesh1).ReadMesh2D(
            coordinateSystem: IdentityTransform<Vector2D>.Instance,
            FiniteElements.Quadrangle.LagrangeQuadratic,
            FiniteElements.Segment.LagrangeQuadratic
        );

        var problem = new EllipticProblem2D(
            Materials: [new(
                Lambda: _ => 1.0,
                Gamma: _ => 0.0,
                Source: p => 0.0
            )],
            BoundaryConditions: [
                new BoundaryCondition2D.Dirichlet(Value: (p, _) => p.X)
            ],
            mesh
        );

        var solver = new EllipticSolver2D(
            DenseMatrix.Factory,
            NumericItegrator2D.Instance,
            new PCGSolver(m => IdentityPreconditioner.Instance)
        );

        var solution = solver.Solve(problem, new ISolver.Params(1e-12, 10000)).Coefficients;

        var real_solutin = new double[] { 0.5,
                                        0,
                                        0.5,
                                        1,
                                        0.5,
                                        0,
                                        1,
                                        1,
                                        0 };
        for (int i = 0; i < solution.Length; i++)
        {
            Assert.Equal(real_solutin[i], solution[i], 1e-10);
        }
    }
    [Fact]
    public void МастерЭлемент_ЛинейнаяФункция_БиКвадратичныйБазис_ВсеВидыКраевых()
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
        1 2 1
        2 3 3
        3 0 2
        """;



        //  * х
        var mesh = new StringReader(TestMesh1).ReadMesh2D(
            coordinateSystem: IdentityTransform<Vector2D>.Instance,
            FiniteElements.Quadrangle.LagrangeQuadratic,
            FiniteElements.Segment.LagrangeQuadratic
        );

        var problem = new EllipticProblem2D(
                    Materials: [new(
                Lambda: _ => 1.0,
                Gamma: _ => 0.0,
                Source: p => 0.0
            )],
                    BoundaryConditions: [
                        new BoundaryCondition2D.Dirichlet(Value: (p,_) => p.X + p.Y),
                new BoundaryCondition2D.Neumann(Flux: (p,_) => 1.0),
                new BoundaryCondition2D.Neumann(Flux: (p,_) => -1.0),
                new BoundaryCondition2D.Robin(Beta: (p,_) => 1.0,UBeta: (p,_) => p.X + 2)
                    ],
                    mesh
                );

        var solver = new EllipticSolver2D(
            DenseMatrix.Factory,
            NumericItegrator2D.Instance,
            new PCGSolver(m => IdentityPreconditioner.Instance)
        );

        var solution = solver.Solve(problem, new ISolver.Params(1e-12, 10000)).Coefficients;

        var real_solutin = new double[] { 2, 1, 1.5, 1.5, 0.5, 1, 0.5, 1, 0 };

        for (int i = 0; i < solution.Length; i++)
        {
            Assert.Equal(real_solutin[i], solution[i], 1e-10);
        }
    }



    [Fact]
    public void РавномернаяСеткаСВнутреннимУзлом_ЛинейнаяФункция_БиКвадратичныйБазис()
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


        //  * х
        var mesh = new StringReader(TestMesh1).ReadMesh2D(
        coordinateSystem: IdentityTransform<Vector2D>.Instance,
        FiniteElements.Quadrangle.LagrangeQuadratic,
        FiniteElements.Segment.LagrangeQuadratic
);

        var problem = new EllipticProblem2D(
            Materials: [new(
                Lambda: _ => 1.0,
                Gamma: _ => 0.0,
                Source: p => 0.0
            )],
            BoundaryConditions: [
                new BoundaryCondition2D.Dirichlet(Value: (p, _) => p.X)
            ],
            mesh
        );

        var solver = new EllipticSolver2D(
            DenseMatrix.Factory,
            NumericItegrator2D.Instance,
            new PCGSolver(m => IdentityPreconditioner.Instance)
        );

        var solution = solver.Solve(problem, new ISolver.Params(1e-12, 10000)).Coefficients;

        var real_solutin = new double[] { 0.5,
                                        0.5,
                                        0.25,
                                        0.75,
                                        0.5,
                                        0.25,
                                        0.75,
                                        0.25,
                                        0.75,
                                        0.75,
                                        1,
                                        0,
                                        0.25,
                                        1,
                                        0.75,
                                        0,
                                        0.25,
                                        1,
                                        0.5,
                                        0,
                                        1,
                                        0,
                                        1,
                                        0.5,
                                        0 };
        for (int i = 0; i < solution.Length; i++)
        {
            Assert.Equal(real_solutin[i], solution[i], 1e-10);
        }

    }
    [Fact]
    public void РавномернаяСеткаСВнутреннимУзлом_КвадратичнаяФункция_БиКвадратичныйБазис()
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


        //  * х
        var mesh = new StringReader(TestMesh1).ReadMesh2D(
        coordinateSystem: IdentityTransform<Vector2D>.Instance,
        FiniteElements.Quadrangle.LagrangeQuadratic,
        FiniteElements.Segment.LagrangeQuadratic
);

        var problem = new EllipticProblem2D(
            Materials: [new(
                Lambda: _ => 1.0,
                Gamma: _ => 0.0,
                Source: p => -2.0
            )],
            BoundaryConditions: [
                new BoundaryCondition2D.Dirichlet(Value: (p, _) => p.X * p.X)
            ],
            mesh
        );

        var solver = new EllipticSolver2D(
            DenseMatrix.Factory,
            NumericItegrator2D.Instance,
            new PCGSolver(m => IdentityPreconditioner.Instance)
        );

        var solution = solver.Solve(problem, new ISolver.Params(1e-12, 10000)).Coefficients;

        var real_solutin = new double[] { 0.250,
                                        0.250,
                                        0.0625,
                                        0.5625,
                                        0.250,
                                        0.0625,
                                        0.5625,
                                        0.0625,
                                        0.5625,
                                        0.5625,
                                        1,
                                        0,
                                        0.0625,
                                        1,
                                        0.5625,
                                        0,
                                        0.0625,
                                        1,
                                        0.25,
                                        0,
                                        1,
                                        0,
                                        1,
                                        0.25,
                                        0 };
        for (int i = 0; i < solution.Length; i++)
        {
            Assert.Equal(real_solutin[i], solution[i], 1e-10);
        }

    }
    [Fact]
    public void РавномернаяСеткаСМножественнымиВнутреннимиУзломи_КвадратичнаяФункция_БиКвадратичныйБазис()
    {
        string TestMesh1 =
               """
        25
        0 0
        1 0
        2 0
        3 0
        4 0
        0 1
        1 1
        2 1
        3 1
        4 1
        0 2
        1 2
        2 2
        3 2
        4 2
        0 3
        1 3
        2 3
        3 3
        4 3
        0 4
        1 4
        2 4
        3 4
        4 4
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
        23 24 0
        22 23 0
        21 22 0
        20 21 0
        15 20 0
        10 15 0
        5 10 0
        0 5 0
        """;


        //  * х
        var mesh = new StringReader(TestMesh1).ReadMesh2D(
        coordinateSystem: IdentityTransform<Vector2D>.Instance,
        FiniteElements.Quadrangle.LagrangeQuadratic,
        FiniteElements.Segment.LagrangeQuadratic
);

        var problem = new EllipticProblem2D(
                    Materials: [new(
                Lambda: _ => 1.0,
                Gamma: _ => 0.0,
                Source: p => -4.0
            )],
                    BoundaryConditions: [
                        new BoundaryCondition2D.Dirichlet(Value: (p, _) => p.X * p.X + p.Y * p.Y)
                    ],
                    mesh
                );

        var solver = new EllipticSolver2D(
            DenseMatrix.Factory,
            NumericItegrator2D.Instance,
            new PCGSolver(m => IdentityPreconditioner.Instance)
        );

        var solution = solver.Solve(problem, new ISolver.Params(1e-12, 10000)).Coefficients;



        var real_solution = new double[] { 2, 5, 10, 5, 8, 13, 10, 13, 18, 1.25, 1.25,
4.25, 3.25, 9.25, 7.25, 13.25, 3.25, 4.25, 6.25, 6.25, 11.25, 10.25, 16.25, 7.25, 9.25,
10.25, 11.25, 15.25, 15.25, 21.25, 13.25, 16.25, 21.25, 0.5, 2.5, 6.5, 12.5, 2.5, 4.5,
8.5, 14.5, 6.5, 8.5, 12.5, 18.5, 12.5, 14.5, 18.5, 24.5, 28.25,
28.25, 22.25, 18.25, 12.25, 16.25, 22.25, 6.25, 18.25,
2.25, 16.25, 12.25, 6.25, 2.25, 0.25, 0.25, 32,
25, 20, 17, 16, 25, 9, 20, 4,
17, 1, 16, 9, 4, 1, 0};

        for (int i = 0; i < solution.Length; i++)
        {
            Assert.Equal(real_solution[i], solution[i], 1e-10);
        }
    }



    //кубич б-с
    [Fact]
    public void МастерЭлемент_КубическаяФункция_БиКубическийБазис()
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


        //  * х
        var mesh = new StringReader(TestMesh1).ReadMesh2D(
            coordinateSystem: IdentityTransform<Vector2D>.Instance,
            FiniteElements.Quadrangle.LagrangeCubic,
            FiniteElements.Segment.LagrangeCubic
        );

        var problem = new EllipticProblem2D(
            Materials: [new(
                Lambda: _ => 1.0,
                Gamma: _ => 0.0,
                Source: p => -6.0 * p.X
            )],
            BoundaryConditions: [
                new BoundaryCondition2D.Dirichlet(Value: (p, _) => p.X * p.X * p.X)
            ],
            mesh
        );

        var solver = new EllipticSolver2D(
            DenseMatrix.Factory,
            NumericItegrator2D.Instance,
            new PCGSolver(m => IdentityPreconditioner.Instance)
        );

        var solution = solver.Solve(problem, new ISolver.Params(1e-12, 10000)).Coefficients;

        var real_solution = new double[] { 1.0 / 27.0,
                                        8.0 / 27.0,
                                        8.0 / 27.0,
                                        1.0 / 27.0,
                                        0,
                                        0,
                                        1.0 / 27.0,
                                        8.0 / 27.0,
                                        1,
                                        1,
                                        8.0 / 27.0,
                                        1.0 / 27.0,
                                        0,
                                        1,
                                        1,
                                        0, };
        for (int i = 0; i < solution.Length; i++)
        {
            Assert.Equal(real_solution[i], solution[i], 1e-10);
        }
    }
    [Fact]
    public void МастерЭлемент_КвадратичнаяФункция_БиКубическийБазис()
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


        //  * х
        var mesh = new StringReader(TestMesh1).ReadMesh2D(
            coordinateSystem: IdentityTransform<Vector2D>.Instance,
            FiniteElements.Quadrangle.LagrangeCubic,
            FiniteElements.Segment.LagrangeCubic
        );

        var problem = new EllipticProblem2D(
            Materials: [new(
                Lambda: _ => 1.0,
                Gamma: _ => 0.0,
                Source: p => -2.0
            )],
            BoundaryConditions: [
                new BoundaryCondition2D.Dirichlet(Value: (p, _) => p.X * p.X)
            ],
            mesh
        );

        var solver = new EllipticSolver2D(
            DenseMatrix.Factory,
            NumericItegrator2D.Instance,
            new PCGSolver(m => IdentityPreconditioner.Instance)
        );

        var solution = solver.Solve(problem, new ISolver.Params(1e-12, 10000)).Coefficients;

        var real_solution = new double[] { 1.0 / 9.0,
                                        4.0 / 9.0,
                                        4.0 / 9.0,
                                        1.0 / 9.0,
                                        0,
                                        0,
                                        1.0 / 9.0,
                                        4.0 / 9.0,
                                        1,
                                        1,
                                        4.0 / 9.0,
                                        1.0 / 9.0,
                                        0,
                                        1,
                                        1,
                                        0, };
        for (int i = 0; i < solution.Length; i++)
        {
            Assert.Equal(real_solution[i], solution[i], 1e-10);
        }
    }
    [Fact]
    public void МастерЭлемент_ЛинейнаяФункция_БиКубическийБазис_ВсеВидыКраевых()
    {
        string meshInput =
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
        1 2 1
        2 3 3
        3 0 2
        """;

        static double analyticalSolution(Vector2D p) => p.X + p.Y;

        var mesh = new StringReader(meshInput).ReadMesh2D(
            coordinateSystem: IdentityTransform<Vector2D>.Instance,
            FiniteElements.Quadrangle.LagrangeCubic,
            FiniteElements.Segment.LagrangeCubic
        );

        var problem = new EllipticProblem2D(
                    Materials: [new(
                Lambda: _ => 1.0,
                Gamma: _ => 0.0,
                Source: p => 0.0
            )],
            BoundaryConditions: [
                new BoundaryCondition2D.Dirichlet(Value: (p, _) => analyticalSolution(p)),
                new BoundaryCondition2D.Neumann(Flux: (p, _) => 1.0),
                new BoundaryCondition2D.Neumann(Flux: (p, _) => -1.0),
                new BoundaryCondition2D.Robin(Beta: (p, _) => 1.0, UBeta: (p, _) => p.X + 2)
            ],
            mesh
        );

        var solver = new EllipticSolver2D(
            DenseMatrix.Factory,
            NumericItegrator2D.Instance,
            new PCGSolver(m => IdentityPreconditioner.Instance)
        );

        var solution = solver.Solve(problem, new ISolver.Params(1e-12, 10000));

        var real_solutin = new double[] { 2, 1, 4 / 3.0, 5 / 3.0, 5 / 3.0, 4 / 3.0, 2 / 3.0,
                                                1 / 3.0, 2 / 3.0, 1, 4 / 3.0, 1, 2 / 3.0, 1 / 3.0, 1, 0};

        for (int i = 0; i < 4; ++i)
            for (int j = 0; j < 4; ++j)
            {
                var point = new Vector2D(i, j) / 3.0;
                Assert.Equal(analyticalSolution(point), solution.Evaluate(point), 1e-10);
            }
    }


    [Fact]
    public void РавномернаяСеткаСВнутреннимУзлом_КубическаяФункция_БиКубическийБазис()
    {
        string TestMesh1 =
               """
        9
        0 0
        3 0
        6 0
        0 3
        3 3
        6 3
        0 6
        3 6
        6 6
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

        static double analyticalSolution(Vector2D p) => p.X * p.X * p.X;

        //  * х
        var mesh = new StringReader(TestMesh1).ReadMesh2D(
        coordinateSystem: IdentityTransform<Vector2D>.Instance,
        FiniteElements.Quadrangle.LagrangeCubic,
        FiniteElements.Segment.LagrangeCubic
);

        var problem = new EllipticProblem2D(
            Materials: [new(
                Lambda: _ => 1.0,
                Gamma: _ => 0.0,
                Source: p => -6.0 * p.X
            )],
            BoundaryConditions: [
                new BoundaryCondition2D.Dirichlet(Value: (p, _) => analyticalSolution(p))
            ],
            mesh
        );

        var solver = new EllipticSolver2D(
            DenseMatrix.Factory,
            NumericItegrator2D.Instance,
            new PCGSolver(m => IdentityPreconditioner.Instance)
        );

        var solution = solver.Solve(problem, new ISolver.Params(1e-12, 10000));

        for (int i = 0; i < 7; ++i)
            for (int j = 0; j < 7; ++j)
            {
                var point = new Vector2D(i, j);
                Assert.Equal(analyticalSolution(point), solution.Evaluate(point), 1e-10);
            }
    }
    [Fact]
    public void РавномернаяСеткаСВнутреннимУзлом_КвадратичнаяФункция_БиКубическийБазис()
    {
        string TestMesh1 =
               """
        9
        0 0
        3 0
        6 0
        0 3
        3 3
        6 3
        0 6
        3 6
        6 6
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

        static double analyticalSolution(Vector2D p) => p.X * p.X;

        //  * х
        var mesh = new StringReader(TestMesh1).ReadMesh2D(
        coordinateSystem: IdentityTransform<Vector2D>.Instance,
        FiniteElements.Quadrangle.LagrangeQuadratic,
        FiniteElements.Segment.LagrangeQuadratic
);

        var problem = new EllipticProblem2D(
            Materials: [new(
                Lambda: _ => 1.0,
                Gamma: _ => 0.0,
                Source: p => -2.0
            )],
            BoundaryConditions: [
                new BoundaryCondition2D.Dirichlet(Value: (p, _) => analyticalSolution(p))
            ],
            mesh
        );

        var solver = new EllipticSolver2D(
            DenseMatrix.Factory,
            NumericItegrator2D.Instance,
            new PCGSolver(m => IdentityPreconditioner.Instance)
        );

        var solution = solver.Solve(problem, new ISolver.Params(1e-12, 10000));

        for (int i = 0; i < 7; ++i)
            for (int j = 0; j < 7; ++j)
            {
                var point = new Vector2D(i, j);
                Assert.Equal(analyticalSolution(point), solution.Evaluate(point), 1e-10);
            }

    }
    [Fact]
    public void РавномернаяСеткаСМножественнымиВнутреннимиУзломи_КвадратичнаяФункция_БиКубическийБазис()
    {
        string TestMesh1 =
               """
        25
        0 0
        3 0
        6 0
        9 0
        12 0
        0 3
        3 3
        6 3
        9 3
        12 3
        0 6
        3 6
        6 6
        9 6
        12 6
        0 9
        3 9
        6 9
        9 9
        12 9
        0 12
        3 12
        6 12
        9 12
        12 12
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
        23 24 0
        22 23 0
        21 22 0
        20 21 0
        15 20 0
        10 15 0
        5 10 0
        0 5 0
        """;

        static double analyticalSolution(Vector2D p) => p.X * p.X + p.Y * p.Y;

        //  * х
        var mesh = new StringReader(TestMesh1).ReadMesh2D(
        coordinateSystem: IdentityTransform<Vector2D>.Instance,
        FiniteElements.Quadrangle.LagrangeQuadratic,
        FiniteElements.Segment.LagrangeQuadratic
);

        var problem = new EllipticProblem2D(
                    Materials: [new(
                Lambda: _ => 1.0,
                Gamma: _ => 0.0,
                Source: p => -4.0
            )],
                    BoundaryConditions: [
                        new BoundaryCondition2D.Dirichlet(Value: (p, _) => analyticalSolution(p))
                    ],
                    mesh
                );

        var solver = new EllipticSolver2D(
            DenseMatrix.Factory,
            NumericItegrator2D.Instance,
            new PCGSolver(m => IdentityPreconditioner.Instance)
        );

        var solution = solver.Solve(problem, new ISolver.Params(1e-12, 10000));

        for (int i = 0; i < 13; ++i)
            for (int j = 0; j < 13; ++j)
            {
                var point = new Vector2D(i, j);
                Assert.Equal(analyticalSolution(point), solution.Evaluate(point), 1e-10);
            }
    }
}
