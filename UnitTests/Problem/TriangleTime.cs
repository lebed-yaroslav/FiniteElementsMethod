using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Text;
using Model.Core.CoordinateSystem;
using Model.Core.Matrix;
using Model.Core.Solver;
using Model.Core.Solver.Precondition;
using Model.Fem.Elements;
using Model.Fem.Mesh;
using Model.Fem.Problem;
using Telma;
using Xunit.Abstractions;

namespace UnitTests.Problem;

public class ParabolicProblemTriangleTests
{
    #region Mesh

    private const string TestMesh1 =
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
    private const string TwoNotMasterTriangles = """
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
    private const string TriangleMeshWithAllBC = """
        9
        0 0
        2 0
        0 2
        2 2
        4 0
        4 2
        0 4
        2 4
        4 4
        8
        0 1 2 0
        1 2 3 0
        1 4 3 0
        3 4 5 0
        2 3 6 0
        3 6 7 0
        3 5 7 0
        5 7 8 0
        8
        0 2 0
        2 6 0
        4 5 0
        5 8 0
        0 1 1
        1 4 1
        6 7 2
        7 8 2
        """;
    private const string TriangleMeshWithDirichlet = """
        9
        0 0
        2 0
        0 2
        2 2
        4 0
        4 2
        0 4
        2 4
        4 4
        8
        0 1 2 0
        1 2 3 0
        1 4 3 0
        3 4 5 0
        2 3 6 0
        3 6 7 0
        3 5 7 0
        5 7 8 0
        8
        0 2 0
        2 6 0
        4 5 0
        5 8 0
        0 1 0
        1 4 0
        6 7 0
        7 8 0
        """;

    #endregion
    public class Linear
    {
       
        [Fact]
        public void DirichletWithLinearTimeFuncExplicitTwoLayer()
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
                    new BoundaryCondition2D.Dirichlet(Value: (p, t) => analyticSolution(p,t)),
                    new BoundaryCondition2D.Neumann(Flux: (p,t) => 0.0),
                    new BoundaryCondition2D.Robin(Beta: (p,t) => 2.0,UBeta: (p,t) => t)
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

            var solver = new ParabolicSolver2D([TimeSchemes.BackwardEuler,TimeSchemes.ImplicitThreeLayer],
                CsrMatrix.Factory,
                NumericItegrator2D.Instance,
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
    }

    public class HierarchicalQuadratic
    {
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

            var solver = new ParabolicSolver2D([TimeSchemes.ForwardEuler],
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
        [Fact]
        public void TwoTriangleAllBCWithLinearTimeFuncExplicitTwoLayerXY()
        {
            static double analyticSolution(Vector2D p, double t) => p.X*p.X + p.Y*p.Y + t;

            var mesh = new StringReader(TwoNotMasterTriangles).ReadMesh2D(
                coordinateSystem: IdentityTransform<Vector2D>.Instance,
                FiniteElements.Triangle.HierarchicalQuadratic,
                FiniteElements.Segment.HierarchicalQuadratic
            );

            var problem = new HyperbolicProblem2D(
                Materials: [new(
                    Lambda: (p,t) => 1.0,
                    Sigma: (p,t) => 1.0,
                    Xi: (p,t) => 0.0,
                    Source: (p,t) => -3.0
                )],
                InitialCondition: p => analyticSolution(p, 0),
                BoundaryConditions: [
                    new BoundaryCondition2D.Dirichlet(Value: (p, t) => analyticSolution(p,t)),
                    new BoundaryCondition2D.Neumann(Flux: (p,t) => 2.0 * p.X),
                    new BoundaryCondition2D.Robin(Beta: (p,t) => 1.0,UBeta: (p,t) => p.X*p.X + 3.0 + t)
                ],
                mesh
            );

            var solver = new ParabolicSolver2D([TimeSchemes.ForwardEuler],
                CsrMatrix.Factory,
                NumericItegrator2D.Instance,
                new PCGSolver(m => IdentityPreconditioner.Instance)
            );
            double[] timeLayers = [0, 0.3, 0.6, 0.9, 1.2, 1.5, 1.8];


            var solutions = solver.Solve(problem, timeLayers, new ISolver.Params(1e-14, 10000));

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
        [Fact]
        public void TwoTriangleAllBCWithLinearTimeFuncExplicitTwoLayer()
        {
            static double analyticSolution(Vector2D p, double t) => t;

            var mesh = new StringReader(TwoNotMasterTriangles).ReadMesh2D(
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
                    new BoundaryCondition2D.Dirichlet(Value: (p, t) => analyticSolution(p,t)),
                    new BoundaryCondition2D.Neumann(Flux: (p,t) => 0.0),
                    new BoundaryCondition2D.Robin(Beta: (p,t) => 2.0,UBeta: (p,t) => t)
                ],
                mesh
            );

            var solver = new ParabolicSolver2D([TimeSchemes.ForwardEuler],
                CsrMatrix.Factory,
                NumericItegrator2D.Instance,
                new PCGSolver(m => IdentityPreconditioner.Instance)
            );
            double[] timeLayers = [0, 0.3, 0.6, 0.9, 1.2, 1.5, 1.8];


            var solutions = solver.Solve(problem, timeLayers, new ISolver.Params(1e-14, 10000));

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

        [Fact]
        public void AllBCWithLinearTimeFuncExplicitTwoLayer()
        {
            static double analyticSolution(Vector2D p, double t) => t;

            var mesh = new StringReader(TriangleWithAllBC).ReadMesh2D(
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
                    new BoundaryCondition2D.Dirichlet(Value: (p, t) => analyticSolution(p,t)),
                    new BoundaryCondition2D.Neumann(Flux: (p,t) => 0.0),
                    new BoundaryCondition2D.Robin(Beta: (p,t) => 2.0,UBeta: (p,t) => t)
                ],
                mesh
            );

            var solver = new ParabolicSolver2D([TimeSchemes.ForwardEuler],
                CsrMatrix.Factory,
                NumericItegrator2D.Instance,
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
        public void ApproximationAllBCWithLinearTimeFuncExplicitTwoLayer()
        {
            static double analyticSolution(Vector2D p, double t) => t;
            string fileOut = "output.txt";
            using FileStream fs = new(fileOut, FileMode.Create);
            using StreamWriter writer = new(fs);

            var mesh = new StringReader(TriangleMeshWithAllBC).ReadMesh2D(
                coordinateSystem: IdentityTransform<Vector2D>.Instance,
                FiniteElements.Triangle.HierarchicalQuadratic,
                FiniteElements.Segment.HierarchicalQuadratic
            );

            var problem = new HyperbolicProblem2D(
                Materials: [new(
                    Lambda: (p,t) => 1.0,
                    Sigma: (p,t) => 2.0,
                    Xi: (p,t) => 0.0,
                    Source: (p,t) => 2.0
                )],
                InitialCondition: p => analyticSolution(p, 0),
                BoundaryConditions: [
                    new BoundaryCondition2D.Dirichlet(Value: (p, t) => analyticSolution(p,t)),
                    new BoundaryCondition2D.Neumann(Flux: (p,t) => 0.0),
                    new BoundaryCondition2D.Robin(Beta: (p,t) => 1.0,UBeta: (p,t) => t)
                ],
                mesh
            );

            var solver = new ParabolicSolver2D([TimeSchemes.ForwardEuler],
                CsrMatrix.Factory,
                NumericItegrator2D.Instance,
                new PCGSolver(m => IdentityPreconditioner.Instance)
            );
            double[] timeLayers = [0, 0.1, 0.2, 0.3, 0.4];

            var solutions = solver.Solve(problem, timeLayers, new ISolver.Params(1e-13, 10000));
            writer.WriteLine("t; (x,y); u_analytic; u_fem; error");
            int timeId = 1;
            foreach (var solution in solutions)
            {
                double currentTime = timeLayers[timeId];
                
                for (int i = 0; i < mesh.VertexCount; i++)
                {
                    writer.Write($"{currentTime};");
                    var point = mesh[i];
                    writer.Write($"[{point.X},{point.Y}];");
                    double u_analytic = analyticSolution(point, currentTime);
                    double u_fem = solution.Evaluate(point);
                    writer.Write($"{u_analytic};");
                    writer.Write($"{u_fem};");
                    writer.WriteLine($"{Math.Abs(u_analytic - u_fem):E14}");
                    Assert.Equal(u_analytic, u_fem, 1e-11);
                }
                timeId++;
            }
        }
        [Fact]
        public void ApproximationDirichletWithLinearTimeFuncImplicitThreeLayer()
        {
            static double analyticSolution(Vector2D p, double t) => t;
            string fileOut = "output.txt";
            using FileStream fs = new(fileOut, FileMode.Create);
            using StreamWriter writer = new(fs);

            var mesh = new StringReader(TriangleMeshWithDirichlet).ReadMesh2D(
                coordinateSystem: IdentityTransform<Vector2D>.Instance,
                FiniteElements.Triangle.HierarchicalQuadratic,
                FiniteElements.Segment.HierarchicalQuadratic
            );

            var problem = new HyperbolicProblem2D(
                Materials: [new(
                    Lambda: (p,t) => 1.0,
                    Sigma: (p,t) => 2.0,
                    Xi: (p,t) => 0.0,
                    Source: (p,t) => 2.0
                )],
                InitialCondition: p => analyticSolution(p, 0),
                BoundaryConditions: [
                    new BoundaryCondition2D.Dirichlet(Value: (p, t) => analyticSolution(p,t)),
                ],
                mesh
            );

            var solver = new ParabolicSolver2D([TimeSchemes.BackwardEuler,TimeSchemes.ImplicitThreeLayer],
                CsrMatrix.Factory,
                NumericItegrator2D.Instance,
                new PCGSolver(m => IdentityPreconditioner.Instance)
            );
            double[] timeLayers = [0, 0.1, 0.2, 0.3, 0.4];

            var solutions = solver.Solve(problem, timeLayers, new ISolver.Params(1e-13, 10000));
            writer.WriteLine("t; (x,y); u_analytic; u_fem; error");
            int timeId = 1;
            foreach (var solution in solutions)
            {
                double currentTime = timeLayers[timeId];

                for (int i = 0; i < mesh.VertexCount; i++)
                {
                    writer.Write($"{currentTime};");
                    var point = mesh[i];
                    writer.Write($"[{point.X},{point.Y}];");
                    double u_analytic = analyticSolution(point, currentTime);
                    double u_fem = solution.Evaluate(point);
                    writer.Write($"{u_analytic};");
                    writer.Write($"{u_fem};");
                    writer.WriteLine($"{Math.Abs(u_analytic - u_fem):E14}");
                    Assert.Equal(u_analytic, u_fem, 1e-11);
                }
                timeId++;
            }
        }

        [Fact]
        public void ApproximationAllBCWithQuadraticTimeFuncExplicitTwoLayer()
        {
            string fileOut = "output.txt";
            using FileStream fs = new(fileOut, FileMode.Create);
            using StreamWriter writer = new(fs);
            static double analyticSolution(Vector2D p,double t) => t * t;
            var mesh = new StringReader(TriangleMeshWithAllBC).ReadMesh2D(
                coordinateSystem: IdentityTransform<Vector2D>.Instance,
                FiniteElements.Triangle.HierarchicalQuadratic,
                FiniteElements.Segment.HierarchicalQuadratic
            );

            var problem = new HyperbolicProblem2D(
                Materials: [new(
                    Lambda: (p,t) => 1.0,
                    Sigma: (p,t) => 1.0,
                    Xi: (p,t) => 0.0,
                    Source: (p,t) => 2.0 * t
                )],
                InitialCondition: p => analyticSolution(p, 0),
                BoundaryConditions: [
                    new BoundaryCondition2D.Dirichlet(Value: (p, t) => analyticSolution(p,t)),
                    new BoundaryCondition2D.Neumann(Flux: (p,t) => 0.0),
                    new BoundaryCondition2D.Robin(Beta: (p,t) => 1.0,UBeta: (p,t) => t * t)
                ],
                mesh
            );

            var solver = new ParabolicSolver2D([TimeSchemes.ForwardEuler],
                CsrMatrix.Factory,
                NumericItegrator2D.Instance,
                new PCGSolver(m => IdentityPreconditioner.Instance)
            );
             double[] timeLayers = [0, 0.1, 0.2, 0.3, 0.4];
            //double[] timeLayers = [0, 0.05, 0.1, 0.15, 0.2, 0.25, 0.3, 0.35, 0.4];
            var solutions = solver.Solve(problem, timeLayers, new ISolver.Params(1e-13, 10000));

            
            writer.WriteLine("t; (x,y); u_analytic; u_fem; error");
            int timeId = 1;
            foreach (var solution in solutions)
            {
                double currentTime = timeLayers[timeId];

                for (int i = 0; i < mesh.VertexCount; i++)
                {
                    writer.Write($"{currentTime};");
                    var point = mesh[i];
                    writer.Write($"[{point.X},{point.Y}];");
                    double u_analytic = analyticSolution(point, currentTime);
                    double u_fem = solution.Evaluate(point);
                    writer.Write($"{u_analytic};");
                    writer.Write($"{u_fem};");
                    writer.WriteLine($"{Math.Abs(u_analytic - u_fem):E14}");
                    //Assert.NotEqual(u_analytic, u_fem, 1e-11);
                }
                timeId++;
            }
            
        }
        [Fact]
        public void ApproximationDirichletWithQuadraticTimeFuncImplicitThreeLayer()
        {
            string fileOut = "output.txt";
            using FileStream fs = new(fileOut, FileMode.Create);
            using StreamWriter writer = new(fs);
            static double analyticSolution(Vector2D p, double t) => t * t;
            var mesh = new StringReader(TriangleMeshWithDirichlet).ReadMesh2D(
                coordinateSystem: IdentityTransform<Vector2D>.Instance,
                FiniteElements.Triangle.HierarchicalQuadratic,
                FiniteElements.Segment.HierarchicalQuadratic
            );

            var problem = new HyperbolicProblem2D(
                Materials: [new(
                    Lambda: (p,t) => 1.0,
                    Sigma: (p,t) => 1.0,
                    Xi: (p,t) => 0.0,
                    Source: (p,t) => 2.0 * t
                )],
                InitialCondition: p => analyticSolution(p, 0),
                BoundaryConditions: [
                    new BoundaryCondition2D.Dirichlet(Value: (p, t) => analyticSolution(p,t)),
                 
                ],
                mesh
            );

            var solver = new ParabolicSolver2D([TimeSchemes.BackwardEuler,TimeSchemes.ImplicitThreeLayer],
                CsrMatrix.Factory,
                NumericItegrator2D.Instance,
                new PCGSolver(m => IdentityPreconditioner.Instance)
            );
            double[] timeLayers = [0, 0.1, 0.2, 0.3, 0.4];
            //double[] timeLayers = [0, 0.0001, 0.0002, 0.0003, 0.0004, 0.0005, 0.0006, 0.0007, 0.0008,0.0009,0.001];
            var solutions = solver.Solve(problem, timeLayers, new ISolver.Params(1e-13, 10000));


            writer.WriteLine("t; (x,y); u_analytic; u_fem; error");
            int timeId = 1;
            foreach (var solution in solutions)
            {
                double currentTime = timeLayers[timeId];

                for (int i = 0; i < mesh.VertexCount; i++)
                {
                    writer.Write($"{currentTime};");
                    var point = mesh[i];
                    writer.Write($"[{point.X},{point.Y}];");
                    double u_analytic = analyticSolution(point, currentTime);
                    double u_fem = solution.Evaluate(point);
                    writer.Write($"{u_analytic};");
                    writer.Write($"{u_fem};");
                    writer.WriteLine($"{Math.Abs(u_analytic - u_fem):E14}");
                    Assert.Equal(u_analytic, u_fem, 1e-11);
                }
                timeId++;
            }

        }
        [Fact]
        public void ApproximationDirichletWithCubicTimeFuncImplicitThreeLayer()
        {
            string fileOut = "output.txt";
            using FileStream fs = new(fileOut, FileMode.Create);
            using StreamWriter writer = new(fs);
            static double analyticSolution(Vector2D p, double t) => t * t * t;
            var mesh = new StringReader(TriangleMeshWithDirichlet).ReadMesh2D(
                coordinateSystem: IdentityTransform<Vector2D>.Instance,
                FiniteElements.Triangle.HierarchicalQuadratic,
                FiniteElements.Segment.HierarchicalQuadratic
            );

            var problem = new HyperbolicProblem2D(
                Materials: [new(
                    Lambda: (p,t) => 1.0,
                    Sigma: (p,t) => 0.0,
                    Xi: (p,t) => 0.0,
                    Source: (p,t) => 0.0
                )],
                InitialCondition: p => analyticSolution(p, 0),
                BoundaryConditions: [
                    new BoundaryCondition2D.Dirichlet(Value: (p, t) => analyticSolution(p,t)),

                ],
                mesh
            );

            var solver = new ParabolicSolver2D([TimeSchemes.BackwardEuler, TimeSchemes.ImplicitThreeLayer],
                CsrMatrix.Factory,
                NumericItegrator2D.Instance,
                new PCGSolver(m => IdentityPreconditioner.Instance)
            );
            double[] timeLayers = [0, 0.1, 0.2, 0.3, 0.4];
            //double[] timeLayers = [0, 0.0001, 0.0002, 0.0003, 0.0004, 0.0005, 0.0006, 0.0007, 0.0008,0.0009,0.001];
            var solutions = solver.Solve(problem, timeLayers, new ISolver.Params(1e-13, 10000));


            writer.WriteLine("t; (x,y); u_analytic; u_fem; error");
            int timeId = 1;
            foreach (var solution in solutions)
            {
                double currentTime = timeLayers[timeId];

                for (int i = 0; i < mesh.VertexCount; i++)
                {
                    writer.Write($"{currentTime};");
                    var point = mesh[i];
                    writer.Write($"[{point.X},{point.Y}];");
                    double u_analytic = analyticSolution(point, currentTime);
                    double u_fem = solution.Evaluate(point);
                    writer.Write($"{u_analytic};");
                    writer.Write($"{u_fem};");
                    writer.WriteLine($"{Math.Abs(u_analytic - u_fem):E14}");
                    //Assert.NotEqual(u_analytic, u_fem, 1e-11);
                }
                timeId++;
            }

        }
        [Fact]
        public void ConvergenceAllBCWithSineAndExplicitTwoLayer()
        {
            string fileOut = "output.txt";
            using FileStream fs = new(fileOut, FileMode.Create);
            using StreamWriter writer = new(fs);
            
            static double analyticSolution(Vector2D p, double t) => Math.Sin(t);
            var mesh = new StringReader(TriangleMeshWithAllBC).ReadMesh2D(
                coordinateSystem: IdentityTransform<Vector2D>.Instance,
                FiniteElements.Triangle.HierarchicalQuadratic,
                FiniteElements.Segment.HierarchicalQuadratic
            );

            var problem = new HyperbolicProblem2D(
                Materials: [new(
                    Lambda: (p,t) => 1.0,
                    Sigma: (p,t) => 3.0,
                    Xi: (p,t) => 0.0,
                    Source: (p,t) => 3.0 * Math.Cos(t)
                )],
                InitialCondition: p => analyticSolution(p, 0),
                BoundaryConditions: [
                    new BoundaryCondition2D.Dirichlet(Value: (p, t) => analyticSolution(p,t)),
                    new BoundaryCondition2D.Neumann(Flux: (p,t) => 0.0),
                    new BoundaryCondition2D.Robin(Beta: (p,t) => 1.0,UBeta: (p,t) => Math.Sin(t))
                ],
                mesh
            );

            var solver = new ParabolicSolver2D([TimeSchemes.ForwardEuler],
                CsrMatrix.Factory,
                NumericItegrator2D.Instance,
                new PCGSolver(m => IdentityPreconditioner.Instance)
            );
            //double[] timeLayers = [0, 0.1, 0.2, 0.3, 0.4];
            double[] timeLayers = [0, 0.05, 0.1, 0.15, 0.2, 0.25, 0.3, 0.35, 0.4];
           
            var solutions = solver.Solve(problem, timeLayers, new ISolver.Params(1e-13, 10000));
            writer.WriteLine("t; difference_L2");
            int timeId = 1;
            double averageDifference = 0.0;
            foreach (var solution in solutions)
            {
                double currentTime = timeLayers[timeId];
                writer.Write($"{currentTime};");
                var difference = 0.0;
                difference = solution.Difference(p => analyticSolution(p, currentTime));
                writer.WriteLine($"{difference:E14}");
                averageDifference += difference;
                timeId++;
            }
            averageDifference /= timeLayers.Length;
            writer.WriteLine($"Average difference: {averageDifference:E14}");
           
        
    }
        [Fact]
        public void TestingTriangleMeshWithLinearTimeFuncExplicitTwoLayerXY()
        {
            static double analyticSolution(Vector2D p, double t) => p.X * p.X + p.Y * p.Y + t;
            string fileOut = "output.txt";
            using FileStream fs = new(fileOut, FileMode.Create);
            using StreamWriter writer = new(fs);
            var mesh = new StringReader(TriangleMeshWithDirichlet).ReadMesh2D(
                coordinateSystem: IdentityTransform<Vector2D>.Instance,
                FiniteElements.Triangle.HierarchicalQuadratic,
                FiniteElements.Segment.HierarchicalQuadratic
            );

            var problem = new HyperbolicProblem2D(
                Materials: [new(
                    Lambda: (p,t) => 3.0,
                    Sigma: (p,t) => 6.0,
                    Xi: (p,t) => 0.0,
                    Source: (p,t) => -6.0
                )],
                InitialCondition: p => analyticSolution(p, 0),
                BoundaryConditions: [
                    new BoundaryCondition2D.Dirichlet(Value: (p, t) => analyticSolution(p,t)),
                ],
                mesh
            );

            var solver = new ParabolicSolver2D([TimeSchemes.ForwardEuler],
                CsrMatrix.Factory,
                NumericItegrator2D.Instance,
                new PCGSolver(m => IdentityPreconditioner.Instance)
            );
            double[] timeLayers = [0, 0.25, 0.5, 0.75, 1, 1.25, 1.5];

            var solutions = solver.Solve(problem, timeLayers, new ISolver.Params(1e-13, 10000));

            writer.WriteLine("t; (x,y); u_analytic; u_fem; error");
            int timeId = 1;
            foreach (var solution in solutions)
            {
                double currentTime = timeLayers[timeId];

                for (int i = 0; i < mesh.VertexCount; i++)
                {
                    writer.Write($"{currentTime};");
                    var point = mesh[i];
                    writer.Write($"[{point.X},{point.Y}];");
                    double u_analytic = analyticSolution(point, currentTime);
                    double u_fem = solution.Evaluate(point);
                    writer.Write($"{u_analytic};");
                    writer.Write($"{u_fem};");
                    writer.WriteLine($"{Math.Abs(u_analytic - u_fem):E14}");
                    Assert.Equal(u_analytic, u_fem, 1e-11);
                }
                timeId++;
            }
        }
        [Fact]
        public void TestingTriangleMeshWithLinearTimeFuncImplicitThreeLayerXY()
        {
            static double analyticSolution(Vector2D p, double t) => p.X * p.X + p.Y * p.Y + t;
            string fileOut = "output.txt";
            using FileStream fs = new(fileOut, FileMode.Create);
            using StreamWriter writer = new(fs);
            var mesh = new StringReader(TriangleMeshWithDirichlet).ReadMesh2D(
                coordinateSystem: IdentityTransform<Vector2D>.Instance,
                FiniteElements.Triangle.HierarchicalQuadratic,
                FiniteElements.Segment.HierarchicalQuadratic
            );

            var problem = new HyperbolicProblem2D(
                Materials: [new(
                    Lambda: (p,t) => 3.0,
                    Sigma: (p,t) => 6.0,
                    Xi: (p,t) => 0.0,
                    Source: (p,t) => -6.0
                )],
                InitialCondition: p => analyticSolution(p, 0),
                BoundaryConditions: [
                    new BoundaryCondition2D.Dirichlet(Value: (p, t) => analyticSolution(p,t)),
                    
                ],
                mesh
            );

            var solver = new ParabolicSolver2D([TimeSchemes.BackwardEuler,TimeSchemes.ImplicitThreeLayer],
                CsrMatrix.Factory,
                NumericItegrator2D.Instance,
                new PCGSolver(m => IdentityPreconditioner.Instance)
            );
            double[] timeLayers = [0, 0.25,0.5,0.75, 1, 1.25, 1.5];

            var solutions = solver.Solve(problem, timeLayers, new ISolver.Params(1e-13, 10000));

            writer.WriteLine("t; (x,y); u_analytic; u_fem; error");
            int timeId = 1;
            foreach (var solution in solutions)
            {
                double currentTime = timeLayers[timeId];

                for (int i = 0; i < mesh.VertexCount; i++)
                {
                    writer.Write($"{currentTime};");
                    var point = mesh[i];
                    writer.Write($"[{point.X},{point.Y}];");
                    double u_analytic = analyticSolution(point, currentTime);
                    double u_fem = solution.Evaluate(point);
                    writer.Write($"{u_analytic};");
                    writer.Write($"{u_fem};");
                    writer.WriteLine($"{Math.Abs(u_analytic - u_fem):E14}");
                    Assert.Equal(u_analytic, u_fem, 1e-11);
                }
                timeId++;
            }
        }
        [Fact]
        public void TestingTriangleMeshWithQuadraticTimeFuncImplicitThreeLayerXY()
        {
            static double analyticSolution(Vector2D p, double t) => p.X * p.X + p.Y * p.Y + t*t;

            var mesh = new StringReader(TriangleMeshWithDirichlet).ReadMesh2D(
                coordinateSystem: IdentityTransform<Vector2D>.Instance,
                FiniteElements.Triangle.HierarchicalQuadratic,
                FiniteElements.Segment.HierarchicalQuadratic
            );

            var problem = new HyperbolicProblem2D(
                Materials: [new(
                    Lambda: (p,t) => 1.0,
                    Sigma: (p,t) => 0.0,
                    Xi: (p,t) => 0.0,
                    Source: (p,t) => -4.0
                )],
                InitialCondition: p => analyticSolution(p, 0),
                BoundaryConditions: [
                    new BoundaryCondition2D.Dirichlet(Value: (p, t) => analyticSolution(p,t)),

                ],
                mesh
            );

            var solver = new ParabolicSolver2D([TimeSchemes.BackwardEuler, TimeSchemes.ImplicitThreeLayer],
                CsrMatrix.Factory,
                NumericItegrator2D.Instance,
                new PCGSolver(m => IdentityPreconditioner.Instance)
            );
            double[] timeLayers = [0, 0.25, 0.5, 0.75, 1, 1.25, 1.5];

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
        [Fact]
        public void DirichletWithLinearTimeFuncImplicitThreeLayer()
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
                    new BoundaryCondition2D.Dirichlet(Value: (p, t) => analyticSolution(p,t)),
                    
                ],
                mesh
            );

            var solver = new ParabolicSolver2D([TimeSchemes.BackwardEuler,TimeSchemes.ImplicitThreeLayer],
                CsrMatrix.Factory,
                NumericItegrator2D.Instance,
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
        public void DirichletWithQuadraticTimeFuncImplicitThreeLayer()
        {
            static double analyticSolution(Vector2D p, double t) => t * t;

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
                    Source: (p,t) => 2 * t
                )],
                InitialCondition: p => analyticSolution(p, 0),
                BoundaryConditions: [
                    new BoundaryCondition2D.Dirichlet(Value: (p, t) => analyticSolution(p,t)),

                ],
                mesh
            );

            var solver = new ParabolicSolver2D([TimeSchemes.BackwardEuler, TimeSchemes.ImplicitThreeLayer],
                CsrMatrix.Factory,
                NumericItegrator2D.Instance,
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
                    new BoundaryCondition2D.Dirichlet(Value: (p, t) => analyticSolution(p,t)),
                    new BoundaryCondition2D.Neumann(Flux: (p,t) => 0.0),
                    new BoundaryCondition2D.Robin(Beta: (p,t) => 2.0,UBeta: (p,t) => t)
                ],
                mesh
            );

            var solver = new ParabolicSolver2D([TimeSchemes.BackwardEuler, TimeSchemes.ImplicitThreeLayer],
                CsrMatrix.Factory,
                NumericItegrator2D.Instance,
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
    }

    #region MeshesCubic

    private const string TestMesh1Cubic = """
        3
        1 1
        4 1
        1 2
        1
        0 1 2 0
        3
        0 1 0
        1 2 0
        2 0 0
        """;

    private const string TriangleWithAllBCCubic = """
        3
        1 1
        4 1
        1 2
        1
        0 1 2 0
        3
        0 1 0
        1 2 1
        2 0 2
        """;

    private const string TwoNotMasterTrianglesCubic = """
        4
        1 1 
        3 1
        1 2
        3 2
        2
        0 1 2 0
        1 2 3 0
        4
        0 1 0 
        2 3 2 
        0 2 0 
        1 3 1 
        """;

    private const string TriangleMeshWithAllBCCubic = """
        9
        1 1
        3 1
        1 3
        3 3
        5 1
        5 3
        1 5
        3 5
        5 5
        8
        0 1 2 0
        1 2 3 0
        1 4 3 0
        3 4 5 0
        2 3 6 0
        3 6 7 0
        3 5 7 0
        5 7 8 0
        8
        0 2 0
        2 6 0
        4 5 0
        5 8 0
        0 1 1
        1 4 1
        6 7 2
        7 8 2
        """;

    private const string TriangleMeshWithDirichletCubic = """
        9
        1 1
        3 1
        1 3
        3 3
        5 1
        5 3
        1 5
        3 5
        5 5
        8
        0 1 2 0
        1 2 3 0
        1 4 3 0
        3 4 5 0
        2 3 6 0
        3 6 7 0
        3 5 7 0
        5 7 8 0
        8
        0 2 0
        2 6 0
        4 5 0
        5 8 0
        0 1 0
        1 4 0
        6 7 0
        7 8 0
        """;

    #endregion

    public class HiararchicalCubicCartezian
    {
        [Fact]
        public void DirichletWithLinearTimeFuncExplicitThreeLayer()
        {
            static double analyticSolution(Vector2D p, double t) => t;

            var mesh = new StringReader(TestMesh1Cubic).ReadMesh2D(
                coordinateSystem: IdentityTransform<Vector2D>.Instance,
                FiniteElements.Triangle.HierarchicalCubic,
                FiniteElements.Segment.HierarchicalCubic
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

            var solver = new ParabolicSolver2D([TimeSchemes.BackwardEuler,TimeSchemes.ExplicitThreeLayer],
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
        [Fact]
        public void TwoTriangleAllBCWithLinearTimeFuncExplicitThreeLayerXY()
        {
            static double analyticSolution(Vector2D p, double t) => p.X * p.X + p.Y * p.Y + t;

            var mesh = new StringReader(TwoNotMasterTrianglesCubic).ReadMesh2D(
                coordinateSystem: IdentityTransform<Vector2D>.Instance,
                FiniteElements.Triangle.HierarchicalCubic,
                FiniteElements.Segment.HierarchicalCubic
            );

            var problem = new HyperbolicProblem2D(
                Materials: [new(
                    Lambda: (p,t) => 1.0,
                    Sigma: (p,t) => 1.0,
                    Xi: (p,t) => 0.0,
                    Source: (p,t) => -3.0
                )],
                InitialCondition: p => analyticSolution(p, 0),
                BoundaryConditions: [
                    new BoundaryCondition2D.Dirichlet(Value: (p, t) => analyticSolution(p,t)),
                    new BoundaryCondition2D.Neumann(Flux: (p,t) => 2.0 * p.X),
                    new BoundaryCondition2D.Robin(Beta: (p,t) => 1.0,UBeta: (p,t) => p.X*p.X  + t)
                ],
                mesh
            );

            var solver = new ParabolicSolver2D([TimeSchemes.BackwardEuler,TimeSchemes.ImplicitThreeLayer],
                CsrMatrix.Factory,
                NumericItegrator2D.Instance,
                new PCGSolver(m => IdentityPreconditioner.Instance)
            );
            double[] timeLayers = [0, 0.3, 0.6, 0.9, 1.2, 1.5, 1.8];


            var solutions = solver.Solve(problem, timeLayers, new ISolver.Params(1e-14, 10000));

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
        [Fact]
        public void TwoTriangleAllBCWithLinearTimeFuncExplicitThreeLayer()
        {
            static double analyticSolution(Vector2D p, double t) => t;

            var mesh = new StringReader(TwoNotMasterTrianglesCubic).ReadMesh2D(
                coordinateSystem: IdentityTransform<Vector2D>.Instance,
                FiniteElements.Triangle.HierarchicalCubic,
                FiniteElements.Segment.HierarchicalCubic
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

            var solver = new ParabolicSolver2D([TimeSchemes.ForwardEuler, TimeSchemes.ImplicitThreeLayer],
                CsrMatrix.Factory,
                NumericItegrator2D.Instance,
                new PCGSolver(m => IdentityPreconditioner.Instance)
            );
            double[] timeLayers = [0, 0.3, 0.6, 0.9, 1.2, 1.5, 1.8];


            var solutions = solver.Solve(problem, timeLayers, new ISolver.Params(1e-14, 10000));

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
        public void AllBCWithLinearTimeFuncExplicitThreeLayer()
        {
            static double analyticSolution(Vector2D p, double t) => t;

            var mesh = new StringReader(TriangleWithAllBCCubic).ReadMesh2D(
                coordinateSystem: IdentityTransform<Vector2D>.Instance,
                FiniteElements.Triangle.HierarchicalCubic,
                FiniteElements.Segment.HierarchicalCubic
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

            var solver = new ParabolicSolver2D([TimeSchemes.BackwardEuler, TimeSchemes.ExplicitThreeLayer],
                CsrMatrix.Factory,
                NumericItegrator2D.Instance,
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
        public void ApproximationAllBCWithLinearTimeFuncExplicitThreeLayer()
        {
            static double analyticSolution(Vector2D p, double t) => t;
            string fileOut = "output.txt";
            using FileStream fs = new(fileOut, FileMode.Create);
            using StreamWriter writer = new(fs);

            var mesh = new StringReader(TriangleMeshWithAllBCCubic).ReadMesh2D(
                coordinateSystem: IdentityTransform<Vector2D>.Instance,
                FiniteElements.Triangle.HierarchicalCubic,
                FiniteElements.Segment.HierarchicalCubic
            );

            var problem = new HyperbolicProblem2D(
                Materials: [new(
                    Lambda: (p,t) => 1.0,
                    Sigma: (p,t) => 2.0,
                    Xi: (p,t) => 0.0,
                    Source: (p,t) => 2.0
                )],
                InitialCondition: p => analyticSolution(p, 0),
                BoundaryConditions: [
                    new BoundaryCondition2D.Dirichlet(Value: (p, t) => analyticSolution(p,t)),
                    new BoundaryCondition2D.Neumann(Flux: (p,t) => 0.0),
                    new BoundaryCondition2D.Robin(Beta: (p,t) => 1.0,UBeta: (p,t) => t)
                ],
                mesh
            );

            var solver = new ParabolicSolver2D([TimeSchemes.BackwardEuler, TimeSchemes.ExplicitThreeLayer],
                CsrMatrix.Factory,
                NumericItegrator2D.Instance,
                new PCGSolver(m => IdentityPreconditioner.Instance)
            );
            double[] timeLayers = [0, 0.1, 0.2, 0.3, 0.4];

            var solutions = solver.Solve(problem, timeLayers, new ISolver.Params(1e-13, 10000));
            writer.WriteLine("t; (x,y); u_analytic; u_fem; error");
            int timeId = 1;
            foreach (var solution in solutions)
            {
                double currentTime = timeLayers[timeId];

                for (int i = 0; i < mesh.VertexCount; i++)
                {
                    writer.Write($"{currentTime};");
                    var point = mesh[i];
                    writer.Write($"[{point.X},{point.Y}];");
                    double u_analytic = analyticSolution(point, currentTime);
                    double u_fem = solution.Evaluate(point);
                    writer.Write($"{u_analytic};");
                    writer.Write($"{u_fem};");
                    writer.WriteLine($"{Math.Abs(u_analytic - u_fem):E14}");
                    Assert.Equal(u_analytic, u_fem, 1e-11);
                }
                timeId++;
            }
        }
        [Fact]
        public void ApproximationDirichletWithLinearTimeFuncImplicitThreeLayer()
        {
            static double analyticSolution(Vector2D p, double t) => t;
            string fileOut = "output.txt";
            using FileStream fs = new(fileOut, FileMode.Create);
            using StreamWriter writer = new(fs);

            var mesh = new StringReader(TriangleMeshWithDirichletCubic).ReadMesh2D(
                coordinateSystem: IdentityTransform<Vector2D>.Instance,
                FiniteElements.Triangle.HierarchicalCubic,
                FiniteElements.Segment.HierarchicalCubic
            );

            var problem = new HyperbolicProblem2D(
                Materials: [new(
                    Lambda: (p,t) => 1.0,
                    Sigma: (p,t) => 2.0,
                    Xi: (p,t) => 0.0,
                    Source: (p,t) => 2.0
                )],
                InitialCondition: p => analyticSolution(p, 0),
                BoundaryConditions: [
                    new BoundaryCondition2D.Dirichlet(Value: (p, t) => analyticSolution(p,t)),
                ],
                mesh
            );

            var solver = new ParabolicSolver2D([TimeSchemes.BackwardEuler, TimeSchemes.ImplicitThreeLayer],
                CsrMatrix.Factory,
                NumericItegrator2D.Instance,
                new PCGSolver(m => IdentityPreconditioner.Instance)
            );
            double[] timeLayers = [0, 0.1, 0.2, 0.3, 0.4];

            var solutions = solver.Solve(problem, timeLayers, new ISolver.Params(1e-13, 10000));
            writer.WriteLine("t; (x,y); u_analytic; u_fem; error");
            int timeId = 1;
            foreach (var solution in solutions)
            {
                double currentTime = timeLayers[timeId];

                for (int i = 0; i < mesh.VertexCount; i++)
                {
                    writer.Write($"{currentTime};");
                    var point = mesh[i];
                    writer.Write($"[{point.X},{point.Y}];");
                    double u_analytic = analyticSolution(point, currentTime);
                    double u_fem = solution.Evaluate(point);
                    writer.Write($"{u_analytic};");
                    writer.Write($"{u_fem};");
                    writer.WriteLine($"{Math.Abs(u_analytic - u_fem):E14}");
                    Assert.Equal(u_analytic, u_fem, 1e-11);
                }
                timeId++;
            }
        }

        [Fact]
        public void ApproximationAllBCWithQuadraticTimeFuncExplicitThreeLayer()
        {
            string fileOut = "output.txt";
            using FileStream fs = new(fileOut, FileMode.Create);
            using StreamWriter writer = new(fs);
            static double analyticSolution(Vector2D p, double t) => t * t;
            var mesh = new StringReader(TriangleMeshWithAllBCCubic).ReadMesh2D(
                coordinateSystem: IdentityTransform<Vector2D>.Instance,
                FiniteElements.Triangle.HierarchicalCubic,
                FiniteElements.Segment.HierarchicalCubic
            );

            var problem = new HyperbolicProblem2D(
                Materials: [new(
                    Lambda: (p,t) => 1.0,
                    Sigma: (p,t) => 1.0,
                    Xi: (p,t) => 0.0,
                    Source: (p,t) => 2.0 * t
                )],
                InitialCondition: p => analyticSolution(p, 0),
                BoundaryConditions: [
                    new BoundaryCondition2D.Dirichlet(Value: (p, t) => analyticSolution(p,t)),
                    new BoundaryCondition2D.Neumann(Flux: (p,t) => 0.0),
                    new BoundaryCondition2D.Robin(Beta: (p,t) => 1.0,UBeta: (p,t) => t * t)
                ],
                mesh
            );

            var solver = new ParabolicSolver2D([TimeSchemes.BackwardEuler, TimeSchemes.ExplicitThreeLayer],
                CsrMatrix.Factory,
                NumericItegrator2D.Instance,
                new PCGSolver(m => IdentityPreconditioner.Instance)
            );
            double[] timeLayers = [0, 0.1, 0.2, 0.3, 0.4];
            //double[] timeLayers = [0, 0.05, 0.1, 0.15, 0.2, 0.25, 0.3, 0.35, 0.4];
            var solutions = solver.Solve(problem, timeLayers, new ISolver.Params(1e-13, 10000));


            writer.WriteLine("t; (x,y); u_analytic; u_fem; error");
            int timeId = 1;
            foreach (var solution in solutions)
            {
                double currentTime = timeLayers[timeId];

                for (int i = 0; i < mesh.VertexCount; i++)
                {
                    writer.Write($"{currentTime};");
                    var point = mesh[i];
                    writer.Write($"[{point.X},{point.Y}];");
                    double u_analytic = analyticSolution(point, currentTime);
                    double u_fem = solution.Evaluate(point);
                    writer.Write($"{u_analytic};");
                    writer.Write($"{u_fem};");
                    writer.WriteLine($"{Math.Abs(u_analytic - u_fem):E14}");
                    //Assert.NotEqual(u_analytic, u_fem, 1e-11);
                }
                timeId++;
            }

        }
        [Fact]
        public void ApproximationDirichletWithQuadraticTimeFuncImplicitThreeLayer()
        {
            string fileOut = "output.txt";
            using FileStream fs = new(fileOut, FileMode.Create);
            using StreamWriter writer = new(fs);
            static double analyticSolution(Vector2D p, double t) => t * t;
            var mesh = new StringReader(TriangleMeshWithDirichletCubic).ReadMesh2D(
                coordinateSystem: IdentityTransform<Vector2D>.Instance,
                FiniteElements.Triangle.HierarchicalCubic,
                FiniteElements.Segment.HierarchicalCubic
            );

            var problem = new HyperbolicProblem2D(
                Materials: [new(
                    Lambda: (p,t) => 1.0,
                    Sigma: (p,t) => 1.0,
                    Xi: (p,t) => 0.0,
                    Source: (p,t) => 2.0 * t
                )],
                InitialCondition: p => analyticSolution(p, 0),
                BoundaryConditions: [
                    new BoundaryCondition2D.Dirichlet(Value: (p, t) => analyticSolution(p,t)),

                ],
                mesh
            );

            var solver = new ParabolicSolver2D([TimeSchemes.BackwardEuler, TimeSchemes.ImplicitThreeLayer],
                CsrMatrix.Factory,
                NumericItegrator2D.Instance,
                new PCGSolver(m => IdentityPreconditioner.Instance)
            );
            double[] timeLayers = [0, 0.1, 0.2, 0.3, 0.4];
            //double[] timeLayers = [0, 0.0001, 0.0002, 0.0003, 0.0004, 0.0005, 0.0006, 0.0007, 0.0008,0.0009,0.001];
            var solutions = solver.Solve(problem, timeLayers, new ISolver.Params(1e-13, 10000));


            writer.WriteLine("t; (x,y); u_analytic; u_fem; error");
            int timeId = 1;
            foreach (var solution in solutions)
            {
                double currentTime = timeLayers[timeId];

                for (int i = 0; i < mesh.VertexCount; i++)
                {
                    writer.Write($"{currentTime};");
                    var point = mesh[i];
                    writer.Write($"[{point.X},{point.Y}];");
                    double u_analytic = analyticSolution(point, currentTime);
                    double u_fem = solution.Evaluate(point);
                    writer.Write($"{u_analytic};");
                    writer.Write($"{u_fem};");
                    writer.WriteLine($"{Math.Abs(u_analytic - u_fem):E14}");
                    Assert.Equal(u_analytic, u_fem, 1e-11);
                }
                timeId++;
            }

        }
        [Fact]
        public void ApproximationDirichletWithCubicTimeFuncImplicitThreeLayer()
        {
            string fileOut = "output.txt";
            using FileStream fs = new(fileOut, FileMode.Create);
            using StreamWriter writer = new(fs);
            static double analyticSolution(Vector2D p, double t) => t * t * t;
            var mesh = new StringReader(TriangleMeshWithDirichletCubic).ReadMesh2D(
                coordinateSystem: IdentityTransform<Vector2D>.Instance,
                FiniteElements.Triangle.HierarchicalCubic,
                FiniteElements.Segment.HierarchicalCubic
            );

            var problem = new HyperbolicProblem2D(
                Materials: [new(
                    Lambda: (p,t) => 1.0,
                    Sigma: (p,t) => 0.0,
                    Xi: (p,t) => 0.0,
                    Source: (p,t) => 0.0
                )],
                InitialCondition: p => analyticSolution(p, 0),
                BoundaryConditions: [
                    new BoundaryCondition2D.Dirichlet(Value: (p, t) => analyticSolution(p,t)),

                ],
                mesh
            );

            var solver = new ParabolicSolver2D([TimeSchemes.BackwardEuler, TimeSchemes.ImplicitThreeLayer],
                CsrMatrix.Factory,
                NumericItegrator2D.Instance,
                new PCGSolver(m => IdentityPreconditioner.Instance)
            );
            double[] timeLayers = [0, 0.1, 0.2, 0.3, 0.4];
            //double[] timeLayers = [0, 0.0001, 0.0002, 0.0003, 0.0004, 0.0005, 0.0006, 0.0007, 0.0008,0.0009,0.001];
            var solutions = solver.Solve(problem, timeLayers, new ISolver.Params(1e-13, 10000));


            writer.WriteLine("t; (x,y); u_analytic; u_fem; error");
            int timeId = 1;
            foreach (var solution in solutions)
            {
                double currentTime = timeLayers[timeId];

                for (int i = 0; i < mesh.VertexCount; i++)
                {
                    writer.Write($"{currentTime};");
                    var point = mesh[i];
                    writer.Write($"[{point.X},{point.Y}];");
                    double u_analytic = analyticSolution(point, currentTime);
                    double u_fem = solution.Evaluate(point);
                    writer.Write($"{u_analytic};");
                    writer.Write($"{u_fem};");
                    writer.WriteLine($"{Math.Abs(u_analytic - u_fem):E14}");
                    //Assert.NotEqual(u_analytic, u_fem, 1e-11);
                }
                timeId++;
            }

        }
        [Fact]
        public void ConvergenceAllBCWithSineAndExplicitThreeLayer()
        {
            string fileOut = "output.txt";
            using FileStream fs = new(fileOut, FileMode.Create);
            using StreamWriter writer = new(fs);

            static double analyticSolution(Vector2D p, double t) => Math.Sin(t);
            var mesh = new StringReader(TriangleMeshWithAllBCCubic).ReadMesh2D(
                coordinateSystem: IdentityTransform<Vector2D>.Instance,
                FiniteElements.Triangle.HierarchicalCubic,
                FiniteElements.Segment.HierarchicalCubic
            );

            var problem = new HyperbolicProblem2D(
                Materials: [new(
                    Lambda: (p,t) => 1.0,
                    Sigma: (p,t) => 3.0,
                    Xi: (p,t) => 0.0,
                    Source: (p,t) => 3.0 * Math.Cos(t)
                )],
                InitialCondition: p => analyticSolution(p, 0),
                BoundaryConditions: [
                    new BoundaryCondition2D.Dirichlet(Value: (p, t) => analyticSolution(p,t)),
                    new BoundaryCondition2D.Neumann(Flux: (p,t) => 0.0),
                    new BoundaryCondition2D.Robin(Beta: (p,t) => 1.0,UBeta: (p,t) => Math.Sin(t))
                ],
                mesh
            );

            var solver = new ParabolicSolver2D([TimeSchemes.BackwardEuler, TimeSchemes.ExplicitThreeLayer],
                CsrMatrix.Factory,
                NumericItegrator2D.Instance,
                new PCGSolver(m => IdentityPreconditioner.Instance)
            );
            //double[] timeLayers = [0, 0.1, 0.2, 0.3, 0.4];
            double[] timeLayers = [0, 0.05, 0.1, 0.15, 0.2, 0.25, 0.3, 0.35, 0.4];

            var solutions = solver.Solve(problem, timeLayers, new ISolver.Params(1e-13, 10000));
            writer.WriteLine("t; difference_L2");
            int timeId = 1;
            double averageDifference = 0.0;
            foreach (var solution in solutions)
            {
                double currentTime = timeLayers[timeId];
                writer.Write($"{currentTime};");
                var difference = 0.0;
                difference = solution.Difference(p => analyticSolution(p, currentTime));
                writer.WriteLine($"{difference:E14}");
                averageDifference += difference;
                timeId++;
            }
            averageDifference /= timeLayers.Length;
            writer.WriteLine($"Average difference: {averageDifference:E14}");


        }
        [Fact]
        public void TestingTriangleMeshWithLinearTimeFuncExplicitThreeLayerXY()
        {
            static double analyticSolution(Vector2D p, double t) => p.X * p.X + p.Y * p.Y + t;
            string fileOut = "output.txt";
            using FileStream fs = new(fileOut, FileMode.Create);
            using StreamWriter writer = new(fs);
            var mesh = new StringReader(TriangleMeshWithDirichletCubic).ReadMesh2D(
                coordinateSystem: IdentityTransform<Vector2D>.Instance,
                FiniteElements.Triangle.HierarchicalCubic,
                FiniteElements.Segment.HierarchicalCubic
            );

            var problem = new HyperbolicProblem2D(
                Materials: [new(
                    Lambda: (p,t) => 3.0,
                    Sigma: (p,t) => 6.0,
                    Xi: (p,t) => 0.0,
                    Source: (p,t) => -6.0
                )],
                InitialCondition: p => analyticSolution(p, 0),
                BoundaryConditions: [
                    new BoundaryCondition2D.Dirichlet(Value: (p, t) => analyticSolution(p,t)),
                ],
                mesh
            );

            var solver = new ParabolicSolver2D([TimeSchemes.BackwardEuler, TimeSchemes.ExplicitThreeLayer],
                CsrMatrix.Factory,
                NumericItegrator2D.Instance,
                new PCGSolver(m => IdentityPreconditioner.Instance)
            );
            double[] timeLayers = [0, 0.25, 0.5, 0.75, 1, 1.25, 1.5];

            var solutions = solver.Solve(problem, timeLayers, new ISolver.Params(1e-13, 10000));

            writer.WriteLine("t; (x,y); u_analytic; u_fem; error");
            int timeId = 1;
            foreach (var solution in solutions)
            {
                double currentTime = timeLayers[timeId];

                for (int i = 0; i < mesh.VertexCount; i++)
                {
                    writer.Write($"{currentTime};");
                    var point = mesh[i];
                    writer.Write($"[{point.X},{point.Y}];");
                    double u_analytic = analyticSolution(point, currentTime);
                    double u_fem = solution.Evaluate(point);
                    writer.Write($"{u_analytic};");
                    writer.Write($"{u_fem};");
                    writer.WriteLine($"{Math.Abs(u_analytic - u_fem):E14}");
                    Assert.Equal(u_analytic, u_fem, 1e-6);
                }
                timeId++;
            }
        }
        [Fact]
        public void TestingTriangleMeshWithLinearTimeFuncImplicitThreeLayerXY()
        {
            static double analyticSolution(Vector2D p, double t) => p.X * p.X + p.Y * p.Y + t;
            string fileOut = "output.txt";
            using FileStream fs = new(fileOut, FileMode.Create);
            using StreamWriter writer = new(fs);
            var mesh = new StringReader(TriangleMeshWithDirichletCubic).ReadMesh2D(
                coordinateSystem: IdentityTransform<Vector2D>.Instance,
                FiniteElements.Triangle.HierarchicalCubic,
                FiniteElements.Segment.HierarchicalCubic
            );

            var problem = new HyperbolicProblem2D(
                Materials: [new(
                    Lambda: (p,t) => 3.0,
                    Sigma: (p,t) => 6.0,
                    Xi: (p,t) => 0.0,
                    Source: (p,t) => -6.0
                )],
                InitialCondition: p => analyticSolution(p, 0),
                BoundaryConditions: [
                    new BoundaryCondition2D.Dirichlet(Value: (p, t) => analyticSolution(p,t)),

                ],
                mesh
            );

            var solver = new ParabolicSolver2D([TimeSchemes.BackwardEuler, TimeSchemes.ImplicitThreeLayer],
                CsrMatrix.Factory,
                NumericItegrator2D.Instance,
                new PCGSolver(m => IdentityPreconditioner.Instance)
            );
            double[] timeLayers = [0, 0.25, 0.5, 0.75, 1, 1.25, 1.5];

            var solutions = solver.Solve(problem, timeLayers, new ISolver.Params(1e-13, 10000));

            writer.WriteLine("t; (x,y); u_analytic; u_fem; error");
            int timeId = 1;
            foreach (var solution in solutions)
            {
                double currentTime = timeLayers[timeId];

                for (int i = 0; i < mesh.VertexCount; i++)
                {
                    writer.Write($"{currentTime};");
                    var point = mesh[i];
                    writer.Write($"[{point.X},{point.Y}];");
                    double u_analytic = analyticSolution(point, currentTime);
                    double u_fem = solution.Evaluate(point);
                    writer.Write($"{u_analytic};");
                    writer.Write($"{u_fem};");
                    writer.WriteLine($"{Math.Abs(u_analytic - u_fem):E14}");
                    Assert.Equal(u_analytic, u_fem, 1e-11);
                }
                timeId++;
            }
        }
        [Fact]
        public void TestingTriangleMeshWithQuadraticTimeFuncImplicitThreeLayerXY()
        {
            static double analyticSolution(Vector2D p, double t) => p.X * p.X + p.Y * p.Y + t * t;

            var mesh = new StringReader(TriangleMeshWithDirichletCubic).ReadMesh2D(
                coordinateSystem: IdentityTransform<Vector2D>.Instance,
                FiniteElements.Triangle.HierarchicalCubic,
                FiniteElements.Segment.HierarchicalCubic
            );

            var problem = new HyperbolicProblem2D(
                Materials: [new(
                    Lambda: (p,t) => 1.0,
                    Sigma: (p,t) => 0.0,
                    Xi: (p,t) => 0.0,
                    Source: (p,t) => -4.0
                )],
                InitialCondition: p => analyticSolution(p, 0),
                BoundaryConditions: [
                    new BoundaryCondition2D.Dirichlet(Value: (p, t) => analyticSolution(p,t)),

                ],
                mesh
            );

            var solver = new ParabolicSolver2D([TimeSchemes.BackwardEuler, TimeSchemes.ImplicitThreeLayer],
                CsrMatrix.Factory,
                NumericItegrator2D.Instance,
                new PCGSolver(m => IdentityPreconditioner.Instance)
            );
            double[] timeLayers = [0, 0.25, 0.5, 0.75, 1, 1.25, 1.5];

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
        [Fact]
        public void DirichletWithLinearTimeFuncImplicitThreeLayer()
        {
            static double analyticSolution(Vector2D p, double t) => t;

            var mesh = new StringReader(TestMesh1Cubic).ReadMesh2D(
                coordinateSystem: IdentityTransform<Vector2D>.Instance,
                FiniteElements.Triangle.HierarchicalCubic,
                FiniteElements.Segment.HierarchicalCubic
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

                ],
                mesh
            );

            var solver = new ParabolicSolver2D([TimeSchemes.BackwardEuler, TimeSchemes.ImplicitThreeLayer],
                CsrMatrix.Factory,
                NumericItegrator2D.Instance,
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
        public void DirichletWithQuadraticTimeFuncImplicitThreeLayer()
        {
            static double analyticSolution(Vector2D p, double t) => t * t;

            var mesh = new StringReader(TestMesh1Cubic).ReadMesh2D(
                coordinateSystem: IdentityTransform<Vector2D>.Instance,
                FiniteElements.Triangle.HierarchicalCubic,
                FiniteElements.Segment.HierarchicalCubic
            );

            var problem = new HyperbolicProblem2D(
                Materials: [new(
                    Lambda: (p,t) => 1.0,
                    Sigma: (p,t) => 1.0,
                    Xi: (p,t) => 0.0,
                    Source: (p,t) => 2 * t
                )],
                InitialCondition: p => analyticSolution(p, 0),
                BoundaryConditions: [
                    new BoundaryCondition2D.Dirichlet(Value: (p, t) => analyticSolution(p,t)),

                ],
                mesh
            );

            var solver = new ParabolicSolver2D([TimeSchemes.BackwardEuler, TimeSchemes.ImplicitThreeLayer],
                CsrMatrix.Factory,
                NumericItegrator2D.Instance,
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

            var mesh = new StringReader(TriangleWithAllBCCubic).ReadMesh2D(
                coordinateSystem: IdentityTransform<Vector2D>.Instance,
                FiniteElements.Triangle.HierarchicalCubic,
                FiniteElements.Segment.HierarchicalCubic
            );

            var problem = new HyperbolicProblem2D(
                Materials: [new(
                    Lambda: (p,t) => 1.0,
                    Sigma:  (p,t) => 1.0,
                    Xi:     (p,t) => 0.0,
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

            var solver = new ParabolicSolver2D([TimeSchemes.BackwardEuler, TimeSchemes.ImplicitThreeLayer],
                CsrMatrix.Factory,
                NumericItegrator2D.Instance,
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
    }

    public class HiararchicalCubicCylindric
    {
        [Fact]
        public void DirichletWithLinearTimeFuncExplicitThreeLayer()
        {
            static double analyticSolution(Vector2D p, double t) => t;

            var mesh = new StringReader(TestMesh1Cubic).ReadMesh2D(
                coordinateSystem: CylindricCoordinateSystem.Instance,
                FiniteElements.Triangle.HierarchicalCubic,
                FiniteElements.Segment.HierarchicalCubic
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

            var solver = new ParabolicSolver2D([TimeSchemes.BackwardEuler, TimeSchemes.ExplicitThreeLayer],
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
        [Fact]
        public void TwoTriangleAllBCWithLinearTimeFuncExplicitThreeLayerXY()
        {
            static double analyticSolution(Vector2D p, double t) => p.X * p.X + p.Y * p.Y + t;

            var mesh = new StringReader(TwoNotMasterTrianglesCubic).ReadMesh2D(
                coordinateSystem: CylindricCoordinateSystem.Instance,
                FiniteElements.Triangle.HierarchicalCubic,
                FiniteElements.Segment.HierarchicalCubic
            );

            var problem = new HyperbolicProblem2D(
                Materials: [new(
                    Lambda: (p,t) => 1.0,
                    Sigma: (p,t) => 1.0,
                    Xi: (p,t) => 0.0,
                    Source: (p,t) => -3.0
                )],
                InitialCondition: p => analyticSolution(p, 0),
                BoundaryConditions: [
                    new BoundaryCondition2D.Dirichlet(Value: (p, t) => analyticSolution(p,t)),
                    new BoundaryCondition2D.Neumann(Flux: (p,t) => 2.0 * p.X),
                    new BoundaryCondition2D.Robin(Beta: (p,t) => 1.0,UBeta: (p,t) => p.X*p.X + 8.0 + t)
                ],
                mesh
            );

            var solver = new ParabolicSolver2D([TimeSchemes.BackwardEuler, TimeSchemes.ImplicitThreeLayer],
                CsrMatrix.Factory,
                NumericItegrator2D.Instance,
                new PCGSolver(m => IdentityPreconditioner.Instance)
            );
            double[] timeLayers = [0, 0.3, 0.6, 0.9, 1.2, 1.5, 1.8];


            var solutions = solver.Solve(problem, timeLayers, new ISolver.Params(1e-14, 10000));

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
        [Fact]
        public void TwoTriangleAllBCWithLinearTimeFuncExplicitThreeLayer()
        {
            static double analyticSolution(Vector2D p, double t) => t;

            var mesh = new StringReader(TwoNotMasterTrianglesCubic).ReadMesh2D(
                coordinateSystem: CylindricCoordinateSystem.Instance,
                FiniteElements.Triangle.HierarchicalCubic,
                FiniteElements.Segment.HierarchicalCubic
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

            var solver = new ParabolicSolver2D([TimeSchemes.ForwardEuler, TimeSchemes.ImplicitThreeLayer],
                CsrMatrix.Factory,
                NumericItegrator2D.Instance,
                new PCGSolver(m => IdentityPreconditioner.Instance)
            );
            double[] timeLayers = [0, 0.3, 0.6, 0.9, 1.2, 1.5, 1.8];


            var solutions = solver.Solve(problem, timeLayers, new ISolver.Params(1e-14, 10000));

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
        public void AllBCWithLinearTimeFuncExplicitThreeLayer()
        {
            static double analyticSolution(Vector2D p, double t) => t;

            var mesh = new StringReader(TriangleWithAllBCCubic).ReadMesh2D(
                coordinateSystem: CylindricCoordinateSystem.Instance,
                FiniteElements.Triangle.HierarchicalCubic,
                FiniteElements.Segment.HierarchicalCubic
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

            var solver = new ParabolicSolver2D([TimeSchemes.BackwardEuler, TimeSchemes.ExplicitThreeLayer],
                CsrMatrix.Factory,
                NumericItegrator2D.Instance,
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
        public void ApproximationAllBCWithLinearTimeFuncExplicitThreeLayer()
        {
            static double analyticSolution(Vector2D p, double t) => t;
            string fileOut = "output.txt";
            using FileStream fs = new(fileOut, FileMode.Create);
            using StreamWriter writer = new(fs);

            var mesh = new StringReader(TriangleMeshWithAllBCCubic).ReadMesh2D(
                coordinateSystem: CylindricCoordinateSystem.Instance,
                FiniteElements.Triangle.HierarchicalCubic,
                FiniteElements.Segment.HierarchicalCubic
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
                    new BoundaryCondition2D.Robin(Beta: (p,t) => 1.0,UBeta: (p,t) => t)
                ],
                mesh
            );

            var solver = new ParabolicSolver2D([TimeSchemes.BackwardEuler, TimeSchemes.ExplicitThreeLayer],
                CsrMatrix.Factory,
                NumericItegrator2D.Instance,
                new PCGSolver(m => IdentityPreconditioner.Instance)
            );
            double[] timeLayers = [0, 0.05, 0.1, 0.15, 0.2, 0.25, 0.3, 0.35, 0.4];
            //double[] timeLayers = [0, 0.1, 0.2, 0.3, 0.4];

            var solutions = solver.Solve(problem, timeLayers, new ISolver.Params(1e-14, 10000));
            writer.WriteLine("t; (x,y); Analytic; Fem; error");
            int timeId = 1;
            foreach (var solution in solutions)
            {
                double currentTime = timeLayers[timeId];

                for (int i = 0; i < mesh.VertexCount; i++)
                {
                    writer.Write($"{currentTime};");
                    var point = mesh[i];
                    writer.Write($"[{point.X}:{point.Y}];");
                    double u_analytic = analyticSolution(point, currentTime);
                    double u_fem = solution.Evaluate(point);
                    writer.Write($"{u_analytic:E16};");
                    writer.Write($"{u_fem:E16};");
                    writer.WriteLine($"{Math.Abs(u_analytic - u_fem):E14}");
                    Assert.Equal(u_analytic, u_fem, 1e-8);
                }
                timeId++;
            }
        }


        [Fact]
        public void ApproximationAllBCWithLinearTimeFuncEmplicitThreeLayer()
        {
            static double analyticSolution(Vector2D p, double t) => t;
            string fileOut = "output.txt";
            using FileStream fs = new(fileOut, FileMode.Create);
            using StreamWriter writer = new(fs);

            var mesh = new StringReader(TriangleMeshWithAllBCCubic).ReadMesh2D(
                coordinateSystem: CylindricCoordinateSystem.Instance,
                FiniteElements.Triangle.HierarchicalCubic,
                FiniteElements.Segment.HierarchicalCubic
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
                    new BoundaryCondition2D.Robin(Beta: (p,t) => 1.0,UBeta: (p,t) => t)
                ],
                mesh
            );

            var solver = new ParabolicSolver2D([TimeSchemes.BackwardEuler, TimeSchemes.ImplicitThreeLayer],
                CsrMatrix.Factory,
                NumericItegrator2D.Instance,
                new PCGSolver(m => IdentityPreconditioner.Instance)
            );
            double[] timeLayers = [0, 0.05, 0.1, 0.15, 0.2, 0.25, 0.3, 0.35, 0.4];
            //double[] timeLayers = [0, 0.1, 0.2, 0.3, 0.4];

            var solutions = solver.Solve(problem, timeLayers, new ISolver.Params(1e-14, 10000));
            writer.WriteLine("t; (x,y); Analytic; Fem; error");
            int timeId = 1;
            foreach (var solution in solutions)
            {
                double currentTime = timeLayers[timeId];

                for (int i = 0; i < mesh.VertexCount; i++)
                {
                    writer.Write($"{currentTime};");
                    var point = mesh[i];
                    writer.Write($"[{point.X}:{point.Y}];");
                    double u_analytic = analyticSolution(point, currentTime);
                    double u_fem = solution.Evaluate(point);
                    writer.Write($"{u_analytic:E16};");
                    writer.Write($"{u_fem:E16};");
                    writer.WriteLine($"{Math.Abs(u_analytic - u_fem):E14}");
                    Assert.Equal(u_analytic, u_fem, 1e-8);
                }
                timeId++;
            }
        }


        [Fact]
        public void ApproximationDirichletWithLinearTimeFuncImplicitThreeLayer()
        {
            static double analyticSolution(Vector2D p, double t) => t;
            string fileOut = "output.txt";
            using FileStream fs = new(fileOut, FileMode.Create);
            using StreamWriter writer = new(fs);

            var mesh = new StringReader(TriangleMeshWithDirichletCubic).ReadMesh2D(
                coordinateSystem: CylindricCoordinateSystem.Instance,
                FiniteElements.Triangle.HierarchicalCubic,
                FiniteElements.Segment.HierarchicalCubic
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
                ],
                mesh
            );

            var solver = new ParabolicSolver2D([TimeSchemes.BackwardEuler, TimeSchemes.ImplicitThreeLayer],
                CsrMatrix.Factory,
                NumericItegrator2D.Instance,
                new PCGSolver(m => IdentityPreconditioner.Instance)
            );
            double[] timeLayers = [0, 0.05, 0.1, 0.15, 0.2, 0.25, 0.3, 0.35, 0.4];

            var solutions = solver.Solve(problem, timeLayers, new ISolver.Params(1e-13, 10000));
            writer.WriteLine("t; (x,y); u_analytic; u_fem; error");
            int timeId = 1;
            foreach (var solution in solutions)
            {
                double currentTime = timeLayers[timeId];

                for (int i = 0; i < mesh.VertexCount; i++)
                {
                    writer.Write($"{currentTime};");
                    var point = mesh[i];
                    writer.Write($"[{point.X},{point.Y}];");
                    double u_analytic = analyticSolution(point, currentTime);
                    double u_fem = solution.Evaluate(point);
                    writer.Write($"{u_analytic};");
                    writer.Write($"{u_fem};");
                    writer.WriteLine($"{Math.Abs(u_analytic - u_fem):E14}");
                    Assert.Equal(u_analytic, u_fem, 1e-10);
                }
                timeId++;
            }
        }



        [Fact]
        public void ApproximationAllBCWithQuadraticTimeFuncExplicitThreeLayer()
        {
            string fileOut = "output.txt";
            using FileStream fs = new(fileOut, FileMode.Create);
            using StreamWriter writer = new(fs);
            static double analyticSolution(Vector2D p, double t) =>  t * t;
            var mesh = new StringReader(TriangleMeshWithDirichletCubic).ReadMesh2D(
                coordinateSystem: CylindricCoordinateSystem.Instance,
                FiniteElements.Triangle.HierarchicalCubic,
                FiniteElements.Segment.HierarchicalCubic
            );

            var problem = new HyperbolicProblem2D(
                Materials: [new(
                    Lambda: (p,t) => 1.0,
                    Sigma: (p,t) => 1.0,
                    Xi: (p,t) => 0.0,
                    Source: (p,t) =>2*t
                )],
                InitialCondition: p => analyticSolution(p, 0),
                BoundaryConditions: [
                    new BoundaryCondition2D.Dirichlet(Value: (p, t) => analyticSolution(p,t)),
                ],
                mesh
            );

            var solver = new ParabolicSolver2D([TimeSchemes.BackwardEuler, TimeSchemes.ExplicitThreeLayer],
                CsrMatrix.Factory,
                NumericItegrator2D.Instance,
                new PCGSolver(m => IdentityPreconditioner.Instance)
            );



            double[] timeLayers = [0, 0.1, 0.2, 0.3, 0.4];
            //double[] timeLayers = [0, 0.05, 0.1, 0.15, 0.2, 0.25, 0.3, 0.35, 0.4];
            //double[] timeLayers = [
            //    0,0.005,0.01,0.015,0.02,0.025,0.03,0.035,0.04,
            //    0.045,0.05,0.055,0.06,0.065,0.07,0.075,0.08,
            //    0.085,0.09,0.095,0.1,0.105,0.11,0.115,0.12,
            //    0.125,0.13,0.135,0.14,0.145,0.15,0.155,0.16,
            //    0.165,0.17,0.175,0.18,0.185,0.19,0.195,0.2,
            //    0.205,0.21,0.215,0.22,0.225,0.23,0.235,0.24,
            //    0.245,0.25,0.255,0.26,0.265,0.27,0.275,0.28,
            //    0.285,0.29,0.295,0.3,0.305,0.31,0.315,0.32,
            //    0.325,0.33,0.335,0.34,0.345,0.35,0.355,0.36,
            //    0.365, 0.37,0.375,0.38,0.385,0.39,0.395,0.4];
            var solutions = solver.Solve(problem, timeLayers, new ISolver.Params(1e-13, 10000));


            writer.WriteLine("t; (x,y); Analytic; Fem; error");
            int timeId = 1;
            foreach (var solution in solutions)
            {
                double currentTime = timeLayers[timeId];

                for (int i = 0; i < mesh.VertexCount; i++)
                {
                    writer.Write($"{currentTime};");
                    var point = mesh[i];
                    writer.Write($"[{point.X},{point.Y}];");
                    double u_analytic = analyticSolution(point, currentTime);
                    double u_fem = solution.Evaluate(point);
                    writer.Write($"{u_analytic};");
                    writer.Write($"{u_fem};");
                    writer.WriteLine($"{Math.Abs(u_analytic - u_fem):E14}");
                    //Assert.NotEqual(u_analytic, u_fem, 1e-11);
                }
                timeId++;
            }

        }

        [Fact]
        public void ApproximationAllBCWithQuadraticTimeFuncImplicitThreeLayer2()
        {
            string fileOut = "output.txt";
            using FileStream fs = new(fileOut, FileMode.Create);
            using StreamWriter writer = new(fs);
            static double analyticSolution(Vector2D p, double t) =>t * t*p.X;
            var mesh = new StringReader(TriangleMeshWithDirichletCubic).ReadMesh2D(
                coordinateSystem: CylindricCoordinateSystem.Instance,
                FiniteElements.Triangle.HierarchicalCubic,
                FiniteElements.Segment.HierarchicalCubic
            );

            var problem = new HyperbolicProblem2D(
                Materials: [new(
                    Lambda: (p,t) => 1.0,
                    Sigma: (p,t) => 1.0,
                    Xi: (p,t) => 0.0,
                    Source: (p,t) =>2*t*p.X
                )],
                InitialCondition: p => analyticSolution(p, 0),
                BoundaryConditions: [
                    new BoundaryCondition2D.Dirichlet(Value: (p, t) => analyticSolution(p,t)),
                ],
                mesh
            );

            var solver = new ParabolicSolver2D([TimeSchemes.BackwardEuler, TimeSchemes.ImplicitThreeLayer],
                CsrMatrix.Factory,
                NumericItegrator2D.Instance,
                new PCGSolver(m => IdentityPreconditioner.Instance)
            );



            double[] timeLayers = [0, 0.1, 0.2, 0.3, 0.4];
            //double[] timeLayers = [0, 0.05, 0.1, 0.15, 0.2, 0.25, 0.3, 0.35, 0.4];
            //double[] timeLayers = [
            //    0,0.005,0.01,0.015,0.02,0.025,0.03,0.035,0.04,
            //    0.045,0.05,0.055,0.06,0.065,0.07,0.075,0.08,
            //    0.085,0.09,0.095,0.1,0.105,0.11,0.115,0.12,
            //    0.125,0.13,0.135,0.14,0.145,0.15,0.155,0.16,
            //    0.165,0.17,0.175,0.18,0.185,0.19,0.195,0.2,
            //    0.205,0.21,0.215,0.22,0.225,0.23,0.235,0.24,
            //    0.245,0.25,0.255,0.26,0.265,0.27,0.275,0.28,
            //    0.285,0.29,0.295,0.3,0.305,0.31,0.315,0.32,
            //    0.325,0.33,0.335,0.34,0.345,0.35,0.355,0.36,
            //    0.365, 0.37,0.375,0.38,0.385,0.39,0.395,0.4];
            var solutions = solver.Solve(problem, timeLayers, new ISolver.Params(1e-13, 10000));


            writer.WriteLine("t; (x,y); Analytic; Fem; error");
            int timeId = 1;
            foreach (var solution in solutions)
            {
                double currentTime = timeLayers[timeId];

                for (int i = 0; i < mesh.VertexCount; i++)
                {
                    writer.Write($"{currentTime};");
                    var point = mesh[i];
                    writer.Write($"[{point.X},{point.Y}];");
                    double u_analytic = analyticSolution(point, currentTime);
                    double u_fem = solution.Evaluate(point);
                    writer.Write($"{u_analytic};");
                    writer.Write($"{u_fem};");
                    writer.WriteLine($"{Math.Abs(u_analytic - u_fem):E14}");
                    //Assert.NotEqual(u_analytic, u_fem, 1e-11);
                }
                timeId++;
            }

        }
        [Fact]
        public void ApproximationDirichletWithQuadraticTimeFuncImplicitThreeLayer()
        {
            string fileOut = "output.txt";
            using FileStream fs = new(fileOut, FileMode.Create);
            using StreamWriter writer = new(fs);
            static double analyticSolution(Vector2D p, double t) => t * t;
            var mesh = new StringReader(TriangleMeshWithDirichletCubic).ReadMesh2D(
                coordinateSystem: CylindricCoordinateSystem.Instance,
                FiniteElements.Triangle.HierarchicalCubic,
                FiniteElements.Segment.HierarchicalCubic
            );

            var problem = new HyperbolicProblem2D(
                Materials: [new(
                    Lambda: (p,t) => 1.0,
                    Sigma: (p,t) => 1.0,
                    Xi: (p,t) => 0.0,
                    Source: (p,t) => 2.0 * t
                )],
                InitialCondition: p => analyticSolution(p, 0),
                BoundaryConditions: [
                    new BoundaryCondition2D.Dirichlet(Value: (p, t) => analyticSolution(p,t)),

                ],
                mesh
            );

            var solver = new ParabolicSolver2D([TimeSchemes.BackwardEuler, TimeSchemes.ImplicitThreeLayer],
                CsrMatrix.Factory,
                NumericItegrator2D.Instance,
                new PCGSolver(m => IdentityPreconditioner.Instance)
            );
            double[] timeLayers = [0, 0.1, 0.2, 0.3, 0.4];
            //double[] timeLayers = [0, 0.0001, 0.0002, 0.0003, 0.0004, 0.0005, 0.0006, 0.0007, 0.0008,0.0009,0.001];
            var solutions = solver.Solve(problem, timeLayers, new ISolver.Params(1e-13, 10000));


            writer.WriteLine("t; (x,y); u_analytic; u_fem; error");
            int timeId = 1;
            foreach (var solution in solutions)
            {
                double currentTime = timeLayers[timeId];

                for (int i = 0; i < mesh.VertexCount; i++)
                {
                    writer.Write($"{currentTime};");
                    var point = mesh[i];
                    writer.Write($"[{point.X},{point.Y}];");
                    double u_analytic = analyticSolution(point, currentTime);
                    double u_fem = solution.Evaluate(point);
                    writer.Write($"{u_analytic};");
                    writer.Write($"{u_fem};");
                    writer.WriteLine($"{Math.Abs(u_analytic - u_fem):E14}");
                    Assert.Equal(u_analytic, u_fem, 1e-11);
                }
                timeId++;
            }

        }
        [Fact]
        public void ApproximationDirichletWithCubicTimeFuncImplicitThreeLayer()
        {
            string fileOut = "output.txt";
            using FileStream fs = new(fileOut, FileMode.Create);
            using StreamWriter writer = new(fs);
            static double analyticSolution(Vector2D p, double t) =>p.X *p.X *p.X* t * t * t;
            var mesh = new StringReader(TriangleMeshWithAllBCCubic).ReadMesh2D(
                coordinateSystem: CylindricCoordinateSystem.Instance,
                FiniteElements.Triangle.HierarchicalCubic,
                FiniteElements.Segment.HierarchicalCubic
            );

            var problem = new HyperbolicProblem2D(
                Materials: [new(
                    Lambda: (p,t) => 1.0,
                    Sigma: (p,t) => 1.0,
                    Xi: (p,t) => 0.0,
                    Source: (p,t) => -9*p.X*t*t*t + 3*p.X*p.X*t*t
                )],
                InitialCondition: p => analyticSolution(p, 0),
                BoundaryConditions: [
                    new BoundaryCondition2D.Dirichlet(Value: (p, t) => analyticSolution(p,t)),
                    new BoundaryCondition2D.Neumann(Flux: (p,t) => 0.0),
                    new BoundaryCondition2D.Robin(Beta: (p,t) => 1.0,UBeta: (p,t) => p.X * p.X * p.X * t * t * t)
                ],
                mesh
            );

            var solver = new ParabolicSolver2D([TimeSchemes.BackwardEuler, TimeSchemes.ImplicitThreeLayer],
                CsrMatrix.Factory,
                NumericItegrator2D.Instance,
                new PCGSolver(m => IdentityPreconditioner.Instance)
            );
            double[] timeLayers = [0, 0.1, 0.2, 0.3, 0.4];
            //double[] timeLayers = [0, 0.0001, 0.0002, 0.0003, 0.0004, 0.0005, 0.0006, 0.0007, 0.0008,0.0009,0.001];
            var solutions = solver.Solve(problem, timeLayers, new ISolver.Params(1e-13, 10000));


            writer.WriteLine("t; (x,y); u_analytic; u_fem; error");
            int timeId = 1;
            foreach (var solution in solutions)
            {
                double currentTime = timeLayers[timeId];

                for (int i = 0; i < mesh.VertexCount; i++)
                {
                    writer.Write($"{currentTime};");
                    var point = mesh[i];
                    writer.Write($"[{point.X},{point.Y}];");
                    double u_analytic = analyticSolution(point, currentTime);
                    double u_fem = solution.Evaluate(point);
                    writer.Write($"{u_analytic};");
                    writer.Write($"{u_fem};");
                    writer.WriteLine($"{Math.Abs(u_analytic - u_fem):E14}");
                    //Assert.NotEqual(u_analytic, u_fem, 1e-11);
                }
                timeId++;
            }

        }
        [Fact]
        public void ApproximationDirichletWithCubicTimeFuncExplicitThreeLayer()
        {
            string fileOut = "output.txt";
            using FileStream fs = new(fileOut, FileMode.Create);
            using StreamWriter writer = new(fs);
            static double analyticSolution(Vector2D p, double t) => p.X * p.X * p.X * t * t * t;
            var mesh = new StringReader(TriangleMeshWithAllBCCubic).ReadMesh2D(
                coordinateSystem: CylindricCoordinateSystem.Instance,
                FiniteElements.Triangle.HierarchicalCubic,
                FiniteElements.Segment.HierarchicalCubic
            );

            var problem = new HyperbolicProblem2D(
                Materials: [new(
                    Lambda: (p,t) => 1.0,
                    Sigma: (p,t) => 1.0,
                    Xi: (p,t) => 0.0,
                    Source: (p,t) => -9*p.X*t*t*t + 3*p.X*p.X*t*t
                )],
                InitialCondition: p => analyticSolution(p, 0),
                BoundaryConditions: [
                    new BoundaryCondition2D.Dirichlet(Value: (p, t) => analyticSolution(p,t)),
                    new BoundaryCondition2D.Neumann(Flux: (p,t) => 0.0),
                    new BoundaryCondition2D.Robin(Beta: (p,t) => 1.0,UBeta: (p,t) => p.X * p.X * p.X * t * t * t)
                ],
                mesh
            );

            var solver = new ParabolicSolver2D([TimeSchemes.BackwardEuler, TimeSchemes.ExplicitThreeLayer],
                CsrMatrix.Factory,
                NumericItegrator2D.Instance,
                new PCGSolver(m => IdentityPreconditioner.Instance)
            );
            double[] timeLayers = [0, 0.1, 0.2, 0.3, 0.4];
            //double[] timeLayers = [0, 0.0001, 0.0002, 0.0003, 0.0004, 0.0005, 0.0006, 0.0007, 0.0008,0.0009,0.001];
            var solutions = solver.Solve(problem, timeLayers, new ISolver.Params(1e-13, 10000));


            writer.WriteLine("t; (x,y); u_analytic; u_fem; error");
            int timeId = 1;
            foreach (var solution in solutions)
            {
                double currentTime = timeLayers[timeId];

                for (int i = 0; i < mesh.VertexCount; i++)
                {
                    writer.Write($"{currentTime};");
                    var point = mesh[i];
                    writer.Write($"[{point.X},{point.Y}];");
                    double u_analytic = analyticSolution(point, currentTime);
                    double u_fem = solution.Evaluate(point);
                    writer.Write($"{u_analytic};");
                    writer.Write($"{u_fem};");
                    writer.WriteLine($"{Math.Abs(u_analytic - u_fem):E14}");
                    //Assert.NotEqual(u_analytic, u_fem, 1e-11);
                }
                timeId++;
            }

        }
        [Fact]
        public void ConvergenceAllBCWithSineAndExplicitThreeLayer()
        {
            string fileOut = "output.txt";
            using FileStream fs = new(fileOut, FileMode.Create);
            using StreamWriter writer = new(fs);

            static double analyticSolution(Vector2D p, double t) => Math.Cos(t);
            var mesh = new StringReader(TriangleMeshWithAllBCCubic).ReadMesh2D(
                coordinateSystem: CylindricCoordinateSystem.Instance,
                FiniteElements.Triangle.HierarchicalCubic,
                FiniteElements.Segment.HierarchicalCubic
            );

            var problem = new HyperbolicProblem2D(
                Materials: [new(
                    Lambda: (p,t) => 1.0,
                    Sigma: (p,t) => 1.0,
                    Xi: (p,t) => 0.0,
                    Source: (p,t) => -Math.Sin(t)
                )],
                InitialCondition: p => analyticSolution(p, 0),
                BoundaryConditions: [
                    new BoundaryCondition2D.Dirichlet(Value: (p, t) => analyticSolution(p,t)),
                    new BoundaryCondition2D.Neumann(Flux: (p,t) => 0.0),
                    new BoundaryCondition2D.Robin(Beta: (p,t) => 1.0,UBeta: (p,t) => Math.Cos(t))
                ],
                mesh
            );

            var solver = new ParabolicSolver2D([TimeSchemes.BackwardEuler, TimeSchemes.ExplicitThreeLayer],
                CsrMatrix.Factory,
                NumericItegrator2D.Instance,
                new PCGSolver(m => IdentityPreconditioner.Instance)
            );

            //double[] timeLayers = [0, 0.1, 0.2, 0.3, 0.4];
            //double[] timeLayers = [0, 0.02, 0.04, 0.06, 0.08];
            double[] timeLayers = [0, 0.01, 0.02, 0.03, 0.04, 0.05, 0.06, 0.07, 0.08];
            //double[] timeLayers = [0, 0.05, 0.1, 0.15, 0.2, 0.25, 0.3, 0.35, 0.4];

            var solutions = solver.Solve(problem, timeLayers, new ISolver.Params(1e-13, 10000));
            writer.WriteLine("t; NormL2");
            int timeId = 1;
            double averageDifference = 0.0;
            foreach (var solution in solutions)
            {
                double currentTime = timeLayers[timeId];
                writer.Write($"{currentTime};");
                var difference = 0.0;
                difference = solution.Difference(p => analyticSolution(p, currentTime));
                writer.WriteLine($"{difference:E14}");
                averageDifference += difference;
                timeId++;
            }
            //averageDifference /= timeLayers.Length;
            //writer.WriteLine($"Average difference: {averageDifference:E14}");


        }
        [Fact]
        public void ConvergenceAllBCWithSineAndImplicitThreeLayer()
        {
            string fileOut = "output.txt";
            using FileStream fs = new(fileOut, FileMode.Create);
            using StreamWriter writer = new(fs);

            static double analyticSolution(Vector2D p, double t) => Math.Cos(t);
            var mesh = new StringReader(TriangleMeshWithAllBCCubic).ReadMesh2D(
                coordinateSystem: CylindricCoordinateSystem.Instance,
                FiniteElements.Triangle.HierarchicalCubic,
                FiniteElements.Segment.HierarchicalCubic
            );

            var problem = new HyperbolicProblem2D(
                Materials: [new(
                    Lambda: (p,t) => 1.0,
                    Sigma: (p,t) => 1.0,
                    Xi: (p,t) => 0.0,
                    Source: (p,t) => -Math.Sin(t)
                )],
                InitialCondition: p => analyticSolution(p, 0),
                BoundaryConditions: [
                    new BoundaryCondition2D.Dirichlet(Value: (p, t) => analyticSolution(p,t)),
                    new BoundaryCondition2D.Neumann(Flux: (p,t) => 0.0),
                    new BoundaryCondition2D.Robin(Beta: (p,t) => 1.0,UBeta: (p,t) => Math.Cos(t))
                ],
                mesh
            );

            var solver = new ParabolicSolver2D([TimeSchemes.BackwardEuler, TimeSchemes.ImplicitThreeLayer],
                CsrMatrix.Factory,
                NumericItegrator2D.Instance,
                new PCGSolver(m => IdentityPreconditioner.Instance)
            );

            //double[] timeLayers = [0, 0.1, 0.2, 0.3, 0.4];
            //double[] timeLayers = [0, 0.02, 0.04, 0.06, 0.08];
            double[] timeLayers = [0, 0.01, 0.02, 0.03, 0.04, 0.05,0.06,0.07,0.08];
            //double[] timeLayers = [0, 0.05, 0.1, 0.15, 0.2, 0.25, 0.3, 0.35, 0.4];

            var solutions = solver.Solve(problem, timeLayers, new ISolver.Params(1e-13, 10000));
            writer.WriteLine("t; NormL2");
            int timeId = 1;
            double averageDifference = 0.0;
            foreach (var solution in solutions)
            {
                double currentTime = timeLayers[timeId];
                writer.Write($"{currentTime};");
                var difference = 0.0;
                difference = solution.Difference(p => analyticSolution(p, currentTime));
                writer.WriteLine($"{difference:E14}");
                averageDifference += difference;
                timeId++;
            }
            //averageDifference /= timeLayers.Length;
            //writer.WriteLine($"Average difference: {averageDifference:E14}");


        }
        [Fact]
        public void TestingTriangleMeshWithLinearTimeFuncExplicitThreeLayerXY()
        {
            static double analyticSolution(Vector2D p, double t) => p.X * p.X + p.Y * p.Y + t;
            string fileOut = "output.txt";
            using FileStream fs = new(fileOut, FileMode.Create);
            using StreamWriter writer = new(fs);
            var mesh = new StringReader(TriangleMeshWithDirichletCubic).ReadMesh2D(
                coordinateSystem: CylindricCoordinateSystem.Instance,
                FiniteElements.Triangle.HierarchicalCubic,
                FiniteElements.Segment.HierarchicalCubic
            );

            var problem = new HyperbolicProblem2D(
                Materials: [new(
                    Lambda: (p,t) => 3.0,
                    Sigma: (p,t) => 6.0,
                    Xi: (p,t) => 0.0,
                    Source: (p,t) => -6.0
                )],
                InitialCondition: p => analyticSolution(p, 0),
                BoundaryConditions: [
                    new BoundaryCondition2D.Dirichlet(Value: (p, t) => analyticSolution(p,t)),
                ],
                mesh
            );

            var solver = new ParabolicSolver2D([TimeSchemes.BackwardEuler, TimeSchemes.ExplicitThreeLayer],
                CsrMatrix.Factory,
                NumericItegrator2D.Instance,
                new PCGSolver(m => IdentityPreconditioner.Instance)
            );
            double[] timeLayers = [0, 0.25, 0.5, 0.75, 1, 1.25, 1.5];

            var solutions = solver.Solve(problem, timeLayers, new ISolver.Params(1e-13, 10000));

            writer.WriteLine("t; (x,y); u_analytic; u_fem; error");
            int timeId = 1;
            foreach (var solution in solutions)
            {
                double currentTime = timeLayers[timeId];

                for (int i = 0; i < mesh.VertexCount; i++)
                {
                    writer.Write($"{currentTime};");
                    var point = mesh[i];
                    writer.Write($"[{point.X},{point.Y}];");
                    double u_analytic = analyticSolution(point, currentTime);
                    double u_fem = solution.Evaluate(point);
                    writer.Write($"{u_analytic};");
                    writer.Write($"{u_fem};");
                    writer.WriteLine($"{Math.Abs(u_analytic - u_fem):E14}");
                    Assert.Equal(u_analytic, u_fem, 1e-6);
                }
                timeId++;
            }
        }
        [Fact]
        public void TestingTriangleMeshWithLinearTimeFuncImplicitThreeLayerXY()
        {
            static double analyticSolution(Vector2D p, double t) => p.X * p.X + p.Y * p.Y + t;
            string fileOut = "output.txt";
            using FileStream fs = new(fileOut, FileMode.Create);
            using StreamWriter writer = new(fs);
            var mesh = new StringReader(TriangleMeshWithDirichletCubic).ReadMesh2D(
                coordinateSystem: CylindricCoordinateSystem.Instance,
                FiniteElements.Triangle.HierarchicalCubic,
                FiniteElements.Segment.HierarchicalCubic
            );

            var problem = new HyperbolicProblem2D(
                Materials: [new(
                    Lambda: (p,t) => 3.0,
                    Sigma: (p,t) => 6.0,
                    Xi: (p,t) => 0.0,
                    Source: (p,t) => -6.0
                )],
                InitialCondition: p => analyticSolution(p, 0),
                BoundaryConditions: [
                    new BoundaryCondition2D.Dirichlet(Value: (p, t) => analyticSolution(p,t)),

                ],
                mesh
            );

            var solver = new ParabolicSolver2D([TimeSchemes.BackwardEuler, TimeSchemes.ImplicitThreeLayer],
                CsrMatrix.Factory,
                NumericItegrator2D.Instance,
                new PCGSolver(m => IdentityPreconditioner.Instance)
            );
            double[] timeLayers = [0, 0.25, 0.5, 0.75, 1, 1.25, 1.5];

            var solutions = solver.Solve(problem, timeLayers, new ISolver.Params(1e-13, 10000));

            writer.WriteLine("t; (x,y); u_analytic; u_fem; error");
            int timeId = 1;
            foreach (var solution in solutions)
            {
                double currentTime = timeLayers[timeId];

                for (int i = 0; i < mesh.VertexCount; i++)
                {
                    writer.Write($"{currentTime};");
                    var point = mesh[i];
                    writer.Write($"[{point.X},{point.Y}];");
                    double u_analytic = analyticSolution(point, currentTime);
                    double u_fem = solution.Evaluate(point);
                    writer.Write($"{u_analytic};");
                    writer.Write($"{u_fem};");
                    writer.WriteLine($"{Math.Abs(u_analytic - u_fem):E14}");
                    Assert.Equal(u_analytic, u_fem, 1e-11);
                }
                timeId++;
            }
        }
        [Fact]
        public void TestingTriangleMeshWithQuadraticTimeFuncImplicitThreeLayerXY()
        {
            static double analyticSolution(Vector2D p, double t) => p.X * p.X + p.Y * p.Y + t * t;

            var mesh = new StringReader(TriangleMeshWithDirichletCubic).ReadMesh2D(
                coordinateSystem: CylindricCoordinateSystem.Instance,
                FiniteElements.Triangle.HierarchicalCubic,
                FiniteElements.Segment.HierarchicalCubic
            );

            var problem = new HyperbolicProblem2D(
                Materials: [new(
                    Lambda: (p,t) => 1.0,
                    Sigma: (p,t) => 0.0,
                    Xi: (p,t) => 0.0,
                    Source: (p,t) => -4.0
                )],
                InitialCondition: p => analyticSolution(p, 0),
                BoundaryConditions: [
                    new BoundaryCondition2D.Dirichlet(Value: (p, t) => analyticSolution(p,t)),

                ],
                mesh
            );

            var solver = new ParabolicSolver2D([TimeSchemes.BackwardEuler, TimeSchemes.ImplicitThreeLayer],
                CsrMatrix.Factory,
                NumericItegrator2D.Instance,
                new PCGSolver(m => IdentityPreconditioner.Instance)
            );
            double[] timeLayers = [0, 0.25, 0.5, 0.75, 1, 1.25, 1.5];

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
        [Fact]
        public void DirichletWithLinearTimeFuncImplicitThreeLayer()
        {
            static double analyticSolution(Vector2D p, double t) => t;

            var mesh = new StringReader(TestMesh1Cubic).ReadMesh2D(
                coordinateSystem: CylindricCoordinateSystem.Instance,
                FiniteElements.Triangle.HierarchicalCubic,
                FiniteElements.Segment.HierarchicalCubic
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

                ],
                mesh
            );

            var solver = new ParabolicSolver2D([TimeSchemes.BackwardEuler, TimeSchemes.ImplicitThreeLayer],
                CsrMatrix.Factory,
                NumericItegrator2D.Instance,
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
        public void DirichletWithQuadraticTimeFuncImplicitThreeLayer()
        {
            static double analyticSolution(Vector2D p, double t) => t * t;

            var mesh = new StringReader(TestMesh1Cubic).ReadMesh2D(
                coordinateSystem: CylindricCoordinateSystem.Instance,
                FiniteElements.Triangle.HierarchicalCubic,
                FiniteElements.Segment.HierarchicalCubic
            );

            var problem = new HyperbolicProblem2D(
                Materials: [new(
                    Lambda: (p,t) => 1.0,
                    Sigma: (p,t) => 1.0,
                    Xi: (p,t) => 0.0,
                    Source: (p,t) => 2 * t
                )],
                InitialCondition: p => analyticSolution(p, 0),
                BoundaryConditions: [
                    new BoundaryCondition2D.Dirichlet(Value: (p, t) => analyticSolution(p,t)),

                ],
                mesh
            );

            var solver = new ParabolicSolver2D([TimeSchemes.BackwardEuler, TimeSchemes.ImplicitThreeLayer],
                CsrMatrix.Factory,
                NumericItegrator2D.Instance,
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

            var mesh = new StringReader(TriangleWithAllBCCubic).ReadMesh2D(
                coordinateSystem: CylindricCoordinateSystem.Instance,
                FiniteElements.Triangle.HierarchicalCubic,
                FiniteElements.Segment.HierarchicalCubic
            );

            var problem = new HyperbolicProblem2D(
                Materials: [new(
                    Lambda: (p,t) => 1.0,
                    Sigma:  (p,t) => 1.0,
                    Xi:     (p,t) => 0.0,
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

            var solver = new ParabolicSolver2D([TimeSchemes.BackwardEuler, TimeSchemes.ImplicitThreeLayer],
                CsrMatrix.Factory,
                NumericItegrator2D.Instance,
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
    }

}
