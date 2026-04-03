using System.Numerics;
using Model.Core.CoordinateSystem;
using Model.Core.Matrix;
using Model.Core.Solver;
using Model.Core.Solver.Precondition;
using Model.Model.Elements;
using Model.Model.Mesh;
using Telma;

namespace UnitTests.Problem;

public class EllipticProblemTriangleTests
{
    private static string TestMesh1 =
    """
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
    public void Test1() {
        // x * x
        var mesh = new StringReader(TestMesh1).ReadMesh2D(
            coordinateSystem: IdentityTransform<Vector2D>.Instance,
            FiniteElements.Triangle.Linear,
            FiniteElements.Segment.Linear
        );

        var problem = new EllipticProblem2D(
            Materials: [new(
                Lambda: _ => 1.0,
                Gamma: _ => 1.0,
                Source: p => -2 + p.X * p.X
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

        var solution = solver.Solve(problem, new ISolver.Params(1e-12, 10000));

        Assert.Equal(0.0, solution[0], 1e-10);
        Assert.Equal(9.0, solution[1], 1e-10);
        Assert.Equal(0.0, solution[2], 1e-10);
    }
}
