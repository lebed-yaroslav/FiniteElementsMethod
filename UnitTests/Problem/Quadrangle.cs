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
    public void МастерЭлемент_ЛинейнаяФункция()
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

        var solution = solver.Solve(problem, new ISolver.Params(1e-12, 10000));

        Assert.Equal(0.0, solution[0], 1e-10);
        Assert.Equal(1.0, solution[1], 1e-10);
        Assert.Equal(1.0, solution[2], 1e-10);
        Assert.Equal(0.0, solution[3], 1e-10);
    }
    [Fact]
    public void МастерЭлемент_ПолиномСтепенни4()
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


        // x * х * х * х
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
                new BoundaryCondition2D.Dirichlet(Value: (p, _) => p.X * p.X * p.X * p.X)
            ],
            mesh
        );

        var solver = new EllipticSolver2D(
            DenseMatrix.Factory,
            NumericItegrator2D.Instance,
            new PCGSolver(m => IdentityPreconditioner.Instance)
        );

        var solution = solver.Solve(problem, new ISolver.Params(1e-12, 10000));

        Assert.Equal(0.0, solution[0], 1e-10);
        Assert.Equal(1.0, solution[1], 1e-10);
        Assert.Equal(1.0, solution[2], 1e-10);
        Assert.Equal(0.0, solution[3], 1e-10);
    }
    [Fact]
    public void ПроизвольныйЭлемент_ЛинейнаяФункция()
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

        var solution = solver.Solve(problem, new ISolver.Params(1e-12, 10000));

        Assert.Equal(-1.0, solution[0], 1e-10);
        Assert.Equal(3.0, solution[1], 1e-10);
        Assert.Equal(3.0, solution[2], 1e-10);
        Assert.Equal(0.0, solution[3], 1e-10);
    }
    [Fact]
    public void ПроизвольныйЭлемент_ПолиномСтепенни4()
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
        // x * х * х * х
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
                new BoundaryCondition2D.Dirichlet(Value: (p, _) => p.X * p.X * p.X * p.X)
            ],
            mesh
        );

        var solver = new EllipticSolver2D(
            DenseMatrix.Factory,
            NumericItegrator2D.Instance,
            new PCGSolver(m => IdentityPreconditioner.Instance)
        );

        var solution = solver.Solve(problem, new ISolver.Params(1e-12, 10000));

        Assert.Equal(1.0, solution[0], 1e-10);
        Assert.Equal(81.0, solution[1], 1e-10);
        Assert.Equal(81.0, solution[2], 1e-10);
        Assert.Equal(0.0, solution[3], 1e-10);
    }
    [Fact]
    public void равномернаяСеткаСВнутреннимУзлом_ЛинейнаяФункция()
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

        var solution = solver.Solve(problem, new ISolver.Params(1e-12, 10000));

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
    public void равномернаяСеткаСВнутреннимУзлом_ПолиномСтепенни4()
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


        // x * х
        var mesh = new StringReader(TestMesh1).ReadMesh2D(
            coordinateSystem: IdentityTransform<Vector2D>.Instance,
            FiniteElements.Quadrangle.Bilinear,
            FiniteElements.Segment.Linear
        );

        var problem = new EllipticProblem2D(
            Materials: [new(
                Lambda: _ => 1.0,
                Gamma: _ => 0.0,
                Source: p => -12.0 * p.X * p.X
            )],
            BoundaryConditions: [
                new BoundaryCondition2D.Dirichlet(Value: (p, _) => p.X * p.X * p.X * p.X)
            ],
            mesh
        );

        var solver = new EllipticSolver2D(
            DenseMatrix.Factory,
            NumericItegrator2D.Instance,
            new PCGSolver(m => IdentityPreconditioner.Instance)
        );

        var solution = solver.Solve(problem, new ISolver.Params(1e-12, 10000));

        Assert.Equal(1.0, solution[0], 1e-10);
        Assert.Equal(16.0, solution[1], 1e-10);
        Assert.Equal(1.0, solution[2], 1e-10);
        Assert.Equal(0.0, solution[3], 1e-10);
        Assert.Equal(16.0, solution[4], 1e-10);
        Assert.Equal(0.0, solution[5], 1e-10);
        Assert.Equal(16.0, solution[6], 1e-10);
        Assert.Equal(1.0, solution[7], 1e-10);
        Assert.Equal(0.0, solution[8], 1e-10);
    }

}
