using System.Numerics;
using Model.Core.CoordinateSystem;
using Model.Core.Matrix;
using Model.Core.Solver;
using Model.Core.Solver.Precondition;
using Model.Model.Elements;
using Model.Model.Mesh;
using Telma;

namespace UnitTests.Problem;

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

        var real_solutin = new double[] { 2, 1, 4 / 3.0, 5 / 3.0, 5 / 3.0, 4 / 3.0, 2 / 3.0,
                                                1 / 3.0, 2 / 3.0, 1, 4 / 3.0, 1, 2 / 3.0, 1 / 3.0, 1, 0};

        for (int i = 0; i < solution.Length; i++)
        {
            Assert.Equal(real_solutin[i], solution[i], 1e-10);
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
