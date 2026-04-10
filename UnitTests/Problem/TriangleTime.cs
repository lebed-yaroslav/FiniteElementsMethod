using System;
using System.Collections.Generic;
using System.Text;
using Model.Core.CoordinateSystem;
using Model.Core.Matrix;
using Model.Core.Solver;
using Model.Core.Solver.Precondition;
using Model.Model.Elements;
using Model.Model.Mesh;
using Model.Model.Problem;
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
        public void Test1()
        {
            static double analyticSolution(Vector2D p,double t) => t;

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
                BoundaryConditions: [
                    new BoundaryCondition2D.Dirichlet(Value: (p, t) => analyticSolution(p,t))
                ],
                mesh
            );
            
            var solver = new ParabolicSolver2D(
                DenseMatrix.Factory,
                NumericItegrator2D.Instance,
                new PCGSolver(m => IdentityPreconditioner.Instance)
            );
            double[] timeLayers = [0,1,2,3,4,5];
            var solution = solver.Solve2Layer(problem,timeLayers,false, new ISolver.Params(1e-12, 10000));
            for (int i = 0; i < timeLayers.Length; i++)
            {
                var solutionForT = solution[i];
                for (int j = 0; j < mesh.VertexCount; ++j)
                {
                    var point = mesh[j];
                    Assert.Equal(analyticSolution(point, timeLayers[i]), solutionForT.Evaluate(point), 1e-12);
                }
            }
            
        }
    }
}
   
   

