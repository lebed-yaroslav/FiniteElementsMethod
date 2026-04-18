using System;
using System.Collections.Generic;
using System.Text;
using Model.Core.CoordinateSystem;
using Model.Core.Matrix;
using Model.Core.Solver;
using Model.Core.Solver.Precondition;
using Model.Fem.Elements;
using Model.Fem.Mesh;
using Model.Fem.Problem;
using Telma;

namespace UnitTests.Problem;

public class ParabolicProblemTriangleTests
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
        public void TriangleWithDirichletWithLinearTimeFunc()
        {
            static double analyticSolution(Vector2D p, double t) => t;

            var mesh = new StringReader(TestMesh1).ReadMesh2D(
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
                    new BoundaryCondition2D.Dirichlet(Value: (p, t) => analyticSolution(p,t))
                ],
                mesh
            );

            var solver = new ParabolicSolver2D(
                [TimeSchemes.ForwardEuler],
                CsrMatrix.Factory,
                NumericItegrator2D.Instance,
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
                    Assert.Equal(analyticSolution(point, currentTime), solution.Evaluate(point), 1e-12);
                }
                timeId++;
            }
        }
        public const string TriangleWithAllBC = """
            3
            0 0
            3 0
            0 1
            1
            0 1 2 0
            3
            0 1 0
            1 2 1
            2 0 1
            """;
        [Fact]
        public void TriangleWithAllBCWithLinearTimeFunc()
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
                    //new BoundaryCondition2D.Robin(Beta: (p,t) => 2.0,UBeta: (p,t) => t)
                ],
                mesh
            );

            var solver = new ParabolicSolver2D(
                [TimeSchemes.ForwardEuler],
                CsrMatrix.Factory,
                NumericItegrator2D.Instance,
                new PCGSolver(m => IdentityPreconditioner.Instance)
            );
            double[] timeLayers = [0, 0.1, 0.2, 0.3, 0.4];
     
            var solutions = solver.Solve(problem, timeLayers,new ISolver.Params(1e-13, 10000));

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
    }
}
