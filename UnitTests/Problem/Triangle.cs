using Model.Core.CoordinateSystem;
using Model.Core.Matrix;
using Model.Core.Solver;
using Model.Core.Solver.Precondition;
using Model.Fem.Elements;
using Model.Fem.Mesh;
using Model.Fem.Problem;
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
        public static string TriangleWithDirichletNeumann => """
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

        public static string TriangleWithDirichletNeumann2 => """
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

        public static string TriangleWithDirichletNeumannRobin => """
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
    }

    public class HierarchicalQuadratic
    {
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

            var solution = solver.Solve(problem, new ISolver.Params(1e-13, 10000));

            for (int i = 0; i < mesh.VertexCount; i++)
            {
                var point = mesh[i];
                Console.WriteLine(point);
                Assert.Equal(u(point), solution.Evaluate(point),1e-12);
            }
            
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

            var solution = solver.Solve(problem, new ISolver.Params(1e-13, 10000));

            for (int i = 0; i < mesh.VertexCount; i++)
            {
                var point = mesh[i];
                Console.WriteLine(point);
                Assert.Equal(u(point), solution.Evaluate(point), 1e-12);
            }
        }
        [Fact]
        public void HierarchicalQuadraticWithDirichletNeumannWithLinearFunc()
        {
            var mesh = new StringReader(Linear.TriangleWithDirichletNeumann).ReadMesh2D(
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

            var solution = solver.Solve(problem, new ISolver.Params(1e-13, 10000));

            for (int i = 0; i < mesh.VertexCount; i++)
            {
                var point = mesh[i];
                Console.WriteLine(point);
                Assert.Equal(u(point), solution.Evaluate(point), 1e-12);
            }
        }
        [Fact]
        public void HierarchicalQuadraticWithDirichletNeumannWithQuadraticFunc()
        {
            var mesh = new StringReader(Linear.TriangleWithDirichletNeumann).ReadMesh2D(
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

            var solution = solver.Solve(problem, new ISolver.Params(1e-13, 10000));
            for (int i = 0; i < mesh.VertexCount; i++)
            {
                var point = mesh[i];
                Console.WriteLine(point);
                Assert.Equal(u(point), solution.Evaluate(point), 1e-12);
            }
        }
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
        public void HierarchicalQuadraticWithAllBCWithQuadraticFunc()
        {
            var mesh = new StringReader(TriangleWithAllBC).ReadMesh2D(
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
                new BoundaryCondition2D.Neumann(Flux: (p,_) => 2.0 * p.X/Math.Sqrt(2.0)),
                new BoundaryCondition2D.Robin(Beta: (p,_) => 1.0,UBeta: (p,_) => -2.0 * p.X),
                ],
                mesh
            );

            var solver = new EllipticSolver2D(
                DenseMatrix.Factory,
                NumericItegrator2D.Instance,
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
        public void HierarchicalQuadraticWithDirichletRobinWithQuadraticFunc2()
        {
            var mesh = new StringReader(TriangleWithDirichletRobin).ReadMesh2D(
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
                new BoundaryCondition2D.Robin(Beta: (p,_) => 1.0,UBeta: (p,_) => 2.0*p.X/Math.Sqrt(2.0) + p.X*p.X),
                new BoundaryCondition2D.Robin(Beta: (p,_) => 1.0,UBeta: (p,_) => -2.0 * p.X),
                ],
                mesh
            );

            var solver = new EllipticSolver2D(
                DenseMatrix.Factory,
                NumericItegrator2D.Instance,
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
        public void HierarchicalQuadraticMeshWithAllBCWithQuadraticFunc()
        {
            var mesh = new StringReader(TwoNotMasterTriangles).ReadMesh2D(
                coordinateSystem: IdentityTransform<Vector2D>.Instance,
                FiniteElements.Triangle.HierarchicalQuadratic,
                FiniteElements.Segment.HierarchicalQuadratic
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
                NumericItegrator2D.Instance,
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
        public void CSRHierarchicalQuadraticMeshWithAllBCWithQuadraticFunc()
        {
            var mesh = new StringReader(TwoNotMasterTriangles).ReadMesh2D(
                coordinateSystem: IdentityTransform<Vector2D>.Instance,
                FiniteElements.Triangle.HierarchicalQuadratic,
                FiniteElements.Segment.HierarchicalQuadratic
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
                NumericItegrator2D.Instance,
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


    public class LagrangeCubic
    {
        [Fact]
        public void LagrangeCubicWithDirichletWithLinearFunc()
        {
            var mesh = new StringReader(Linear.TestMesh1).ReadMesh2D(
                coordinateSystem: IdentityTransform<Vector2D>.Instance,
                FiniteElements.Triangle.LagrangeCubic,
                FiniteElements.Segment.LagrangeCubic
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

            var solution = solver.Solve(problem, new ISolver.Params(1e-13, 10000));

            for (int i = 0; i < mesh.VertexCount; i++)
            {
                var point = mesh[i];
                Console.WriteLine(point);
                Assert.Equal(u(point), solution.Evaluate(point),1e-12);
            }
            
        }
        [Fact]
        public void LagrangeCubicWithDirichletWithQuadraticFunc()
        {
            var mesh = new StringReader(Linear.TestMesh1).ReadMesh2D(
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
                    new BoundaryCondition2D.Dirichlet(Value: (p,_) => u(p))
                ],
                mesh
            );

            var solver = new EllipticSolver2D(
                DenseMatrix.Factory,
                NumericItegrator2D.Instance,
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
        public void LagrangeCubicWithDirichletNeumannWithLinearFunc()
        {
            var mesh = new StringReader(Linear.TriangleWithDirichletNeumann).ReadMesh2D(
                coordinateSystem: IdentityTransform<Vector2D>.Instance,
                FiniteElements.Triangle.LagrangeCubic,
                FiniteElements.Segment.LagrangeCubic
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

            var solution = solver.Solve(problem, new ISolver.Params(1e-13, 10000));

            for (int i = 0; i < mesh.VertexCount; i++)
            {
                var point = mesh[i];
                Console.WriteLine(point);
                Assert.Equal(u(point), solution.Evaluate(point), 1e-12);
            }
        }
        [Fact]
        public void LagrangeCubicWithDirichletNeumannWithQuadraticFunc()
        {
            var mesh = new StringReader(Linear.TriangleWithDirichletNeumann).ReadMesh2D(
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
                new BoundaryCondition2D.Neumann(Flux: (p,_) => -2 * p.X)
                ],
                mesh
            );

            var solver = new EllipticSolver2D(
                DenseMatrix.Factory,
                NumericItegrator2D.Instance,
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
                NumericItegrator2D.Instance,
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
                NumericItegrator2D.Instance,
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
                NumericItegrator2D.Instance,
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
                NumericItegrator2D.Instance,
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

        private static string TwoNotMasterTrianglesFlipped => """
        4
        0 0 
        2 0
        0 1
        2 1
        2
        0 2 1 0
        1 2 3 0
        4
        0 1 0 
        3 2 2 
        2 0 0 
        3 1 1 
        """;

        [Fact]
        public void LagrangeCubicMeshWithAllBCWithQuadraticFuncFlipped()
        {
            var mesh = new StringReader(TwoNotMasterTrianglesFlipped).ReadMesh2D(
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
                NumericItegrator2D.Instance,
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
        public void CSRLagrangeCubicMeshWithAllBCWithQuadraticFuncFlipped()
        {
            var mesh = new StringReader(TwoNotMasterTrianglesFlipped).ReadMesh2D(
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
                NumericItegrator2D.Instance,
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
}




