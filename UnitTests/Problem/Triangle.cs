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
    public class Linear
    {
        public const string TestMesh1 =
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
        public void Solve_LinearFn_DirichletBc_NoErrorOnVertices()
        {
            static double analyticSolution(Vector2D p) => p.X + p.Y;

            var mesh = new StringReader(TestMesh1).ReadMesh2D(
                coordinateSystem: IdentityTransform<Vector2D>.Instance,
                FiniteElements.Triangle.Linear,
                FiniteElements.Segment.Linear
            );

            var problem = new EllipticProblem2D(
                Materials: [new(
                    Lambda: _ => 1.0,
                    Gamma: _ => 1.0,
                    Source: p => analyticSolution(p)
                )],
                BoundaryConditions: [
                    new BoundaryCondition2D.Dirichlet(Value: (p, _) => analyticSolution(p))
                ],
                mesh
            );

            var solver = new EllipticSolver2D(
                DenseMatrix.Factory,
                NumericItegrator2D.Instance,
                new PCGSolver(m => IdentityPreconditioner.Instance)
            );

            var solution = solver.Solve(problem, new ISolver.Params(1e-12, 10000));
            for (int i = 0; i < mesh.VertexCount; ++i)
            {
                var point = mesh[i];
                Console.WriteLine(point);
                Assert.Equal(analyticSolution(point), solution.Evaluate(point), 1e-12);
            }
        }
    }

    private static string TriangleWithDirichletNeumann => """
        3
        0 0
        3 0
        0 1
        1
        0 1 2 0
        3
        0 1 0
        1 2 0
        0 2 1
        """;

    [Fact]
    public void LinearTriangleWithDirichletNeumannWithLinearFunc()
    {
        var mesh = new StringReader(TriangleWithDirichletNeumann).ReadMesh2D(
            coordinateSystem: IdentityTransform<Vector2D>.Instance,
            FiniteElements.Triangle.Linear,
            FiniteElements.Segment.Linear
        );
        static double u(Vector2D p) => p.X;
        var problem = new EllipticProblem2D(
            Materials: [new(
                Lambda: _ => 1.0,
                Gamma: _ => 1.0,
                Source: p => p.X
            )],
            BoundaryConditions: [
                new BoundaryCondition2D.Dirichlet(Value: (p,_) => u(p)),
                new BoundaryCondition2D.Neumann(Flux: (p,_) => -1.0)
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
        Assert.Equal(3.0, solution[1], 1e-10);
        Assert.Equal(0.0, solution[2], 1e-10);
    }

    private static string TriangleWithDirichletNeumann2 => """
        3
        0 0
        3 0
        0 1
        1
        0 1 2 0
        3
        0 1 0
        1 2 1
        0 2 0
        """;

    [Fact]
    public void LinearTriangleWithDirichletNeumannWithLinearFunc2()
    {
        var mesh = new StringReader(TriangleWithDirichletNeumann2).ReadMesh2D(
            coordinateSystem: IdentityTransform<Vector2D>.Instance,
            FiniteElements.Triangle.Linear,
            FiniteElements.Segment.Linear
        );
        static double u(Vector2D p) => p.X;
        var problem = new EllipticProblem2D(
            Materials: [new(
                Lambda: _ => 1.0,
                Gamma: _ => 1.0,
                Source: p => p.X
            )],
            BoundaryConditions: [
                new BoundaryCondition2D.Dirichlet(Value: (p,_) => u(p)),
                new BoundaryCondition2D.Neumann(Flux: (p,_) => 1.0/Math.Sqrt(10))
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
        Assert.Equal(3.0, solution[1], 1e-10);
        Assert.Equal(0.0, solution[2], 1e-10);
    }

    private static string TriangleWithDirichletNeumannRobin => """
        3
        0 0
        3 0
        0 1
        1
        0 1 2 0
        3
        0 1 0
        1 2 1
        0 2 2
        """;
    [Fact]
    public void LinearTriangleWithDirichletNeumannRobinWithLinearFunc()
    {
        var mesh = new StringReader(TriangleWithDirichletNeumannRobin).ReadMesh2D(
            coordinateSystem: IdentityTransform<Vector2D>.Instance,
            FiniteElements.Triangle.Linear,
            FiniteElements.Segment.Linear
        );
        static double u(Vector2D p) => p.X;
        var problem = new EllipticProblem2D(
            Materials: [new(
                Lambda: _ => 1.0,
                Gamma: _ => 1.0,
                Source: p => p.X
            )],
            BoundaryConditions: [
                new BoundaryCondition2D.Dirichlet(Value: (p,_) => u(p)),
                new BoundaryCondition2D.Neumann(Flux: (p,_) => 1.0/Math.Sqrt(10)),
                new BoundaryCondition2D.Robin(Beta: (p,_) => 2.0,UBeta: (p,_) => -0.5)
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
        Assert.Equal(3.0, solution[1], 1e-10);
        Assert.Equal(0.0, solution[2], 1e-10);
    }
    [Fact]
    public void LinearTriangleWithDirichletNeumannRobinWithLinearFunc2()
    {
        var mesh = new StringReader(TriangleWithDirichletNeumannRobin).ReadMesh2D(
            coordinateSystem: IdentityTransform<Vector2D>.Instance,
            FiniteElements.Triangle.Linear,
            FiniteElements.Segment.Linear
        );
        static double u(Vector2D p) => p.X;
        var problem = new EllipticProblem2D(
            Materials: [new(
                Lambda: _ => 1.0,
                Gamma: _ => 1.0,
                Source: p => p.X
            )],
            BoundaryConditions: [
                new BoundaryCondition2D.Dirichlet(Value: (p,_) => u(p)),
                new BoundaryCondition2D.Neumann(Flux: (p,_) => 1.0/Math.Sqrt(10)),
                new BoundaryCondition2D.Robin(Beta: (p,_) => 1.0,UBeta: (p,_) => -1.0)
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
        Assert.Equal(3.0, solution[1], 1e-10);
        Assert.Equal(0.0, solution[2], 1e-10);
    }
    [Fact]
    public void HierarchicalQuadraticWithDirichletWithLinearFunc()
    {
        var mesh = new StringReader(Linear.TestMesh1).ReadMesh2D(
            coordinateSystem: IdentityTransform<Vector2D>.Instance,
            FiniteElements.Triangle.HierarchicalQuadratic,
            FiniteElements.Segment.HierarchicalQuadratic
        );
        static double u(Vector2D p) => p.X;
        var problem = new EllipticProblem2D(
            Materials: [new(
                Lambda: _ => 1.0,
                Gamma: _ => 1.0,
                Source: p => p.X
            )],
            BoundaryConditions: [
                new BoundaryCondition2D.Dirichlet(Value: (p,_) => u(p)) 
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
        Assert.Equal(0.0, solution[1], 1e-10);
        Assert.Equal(0.0, solution[2], 1e-10);
        Assert.Equal(0.0, solution[3], 1e-10);
        Assert.Equal(3.0, solution[4], 1e-10);
        Assert.Equal(0.0, solution[5], 1e-10);
    }
    [Fact]
    public void HierarchicalQuadraticWithDirichletWithQuadraticFunc()
    {
        var mesh = new StringReader(Linear.TestMesh1).ReadMesh2D(
            coordinateSystem: IdentityTransform<Vector2D>.Instance,
            FiniteElements.Triangle.HierarchicalQuadratic,
            FiniteElements.Segment.HierarchicalQuadratic
        );
        static double u(Vector2D p) => p.X * p.X;
        var problem = new EllipticProblem2D(
            Materials: [new(
                Lambda: _ => 1.0,
                Gamma: _ => 1.0,
                Source: p => -2 + p.X * p.X
            )],
            BoundaryConditions: [
                new BoundaryCondition2D.Dirichlet(Value: (p,_) => u(p))
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
        Assert.Equal(-9.0, solution[1], 1e-10);
        Assert.Equal(-9.0, solution[2], 1e-10);
        Assert.Equal(0.0, solution[3], 1e-10);
        Assert.Equal(9.0, solution[4], 1e-10);
        Assert.Equal(0.0, solution[5], 1e-10);
    }
    [Fact]
    public void HierarchicalQuadraticWithDirichletNeumannWithLinearFunc()
    {
        var mesh = new StringReader(TriangleWithDirichletNeumann).ReadMesh2D(
            coordinateSystem: IdentityTransform<Vector2D>.Instance,
            FiniteElements.Triangle.HierarchicalQuadratic,
            FiniteElements.Segment.HierarchicalQuadratic
        );
        static double u(Vector2D p) => p.X;
        var problem = new EllipticProblem2D(
            Materials: [new(
                Lambda: _ => 1.0,
                Gamma: _ => 1.0,
                Source: p => p.X
            )],
            BoundaryConditions: [
                new BoundaryCondition2D.Dirichlet(Value: (p,_) => u(p)),
                new BoundaryCondition2D.Neumann(Flux: (p,_) => -1.0)
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
        Assert.Equal(0.0, solution[1], 1e-10);
        Assert.Equal(0.0, solution[2], 1e-10);
        Assert.Equal(0.0, solution[3], 1e-10);
        Assert.Equal(3.0, solution[4], 1e-10);
        Assert.Equal(0.0, solution[5], 1e-10);
    }
    [Fact]
    public void HierarchicalQuadraticWithDirichletNeumannWithQuadraticFunc()
    {
        var mesh = new StringReader(TriangleWithDirichletNeumann).ReadMesh2D(
            coordinateSystem: IdentityTransform<Vector2D>.Instance,
            FiniteElements.Triangle.HierarchicalQuadratic,
            FiniteElements.Segment.HierarchicalQuadratic
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
                new BoundaryCondition2D.Neumann(Flux: (p,_) => -2 * p.X)
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
        Assert.Equal(-9.0, solution[1], 1e-10);
        Assert.Equal(-9.0, solution[2], 1e-10);
        Assert.Equal(0.0, solution[3], 1e-10);
        Assert.Equal(9.0, solution[4], 1e-10);
        Assert.Equal(0.0, solution[5], 1e-10);
    }
}



