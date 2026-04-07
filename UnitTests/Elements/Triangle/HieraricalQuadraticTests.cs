using System.Diagnostics;
using Model.Core.CoordinateSystem;
using Model.Model;
using Model.Model.Elements;
using Model.Model.Elements.Segment;
using Model.Model.Mesh;
using Model.Model.Problem;
using Telma;
using DofManager = Model.Model.Assembly.DofManager;
namespace UnitTests;


public class HierarichalQuadraticTests
{
    [Fact]
    public void LocalStiffnessMatrix_IsSymmetric()
    {
        static double U(Vector2D p) => p.X;
        var mesh = BuildUnitMesh();
        var integrator = NumericItegrator2D.Instance;
        var problem = new HyperbolicProblem2D(
           Materials:
           [
               new (
                    Lambda: static (_, _) => 1.0,
                    Xi: static (_, _) => 0.0,
                    Sigma: static (_, _) => 1.0, //gamma
                    Source: (p, _) => U(p)
                )
           ],
           BoundaryConditions:
           [
               // [0,1]:
                new BoundaryCondition2D.Dirichlet((p, _) => U(p)),
                // [0,2]
                new BoundaryCondition2D.Dirichlet((p, _) => U(p)),
                //[2,3]
                new BoundaryCondition2D.Dirichlet((p,_) => U(p)),
               
                ////[1,3]: 
                //new BoundaryCondition2D.Dirichlet((p, _) => U(p))
           ],
           Mesh: mesh
       );
        var dofManager = DofManager.NumerateDof(mesh, problem.BoundaryConditions);
        foreach (var element in mesh.FiniteElements)
        {
            var G = integrator.CalculateLocalStiffness(element, _ => 1.0);
            for (int i = 0; i < G.Size; i++)
            {
                for (int j = 0; j < G.Size; j++)
                    Assert.Equal(G[i, j], G[j, i],1e-13);
            }
        }
    }
    [Fact]
    public void LocalStiffnessMultipliedByConstantIsZero()
    {
        
        static double U(Vector2D p) => 5.0;
        var mesh = BuildUnitMesh();
        var integrator = NumericItegrator2D.Instance;
        var problem = new HyperbolicProblem2D(
           Materials:
           [
               new (
                    Lambda: static (_, _) => 1.0,
                    Xi: static (_, _) => 0.0,
                    Sigma: static (_, _) => 1.0, //gamma
                    Source: (p, _) => U(p)
                )
           ],
           BoundaryConditions:
           [
               // [0,1]:
                new BoundaryCondition2D.Dirichlet((p, _) => U(p)),
                // [0,2]
                new BoundaryCondition2D.Dirichlet((p, _) => U(p)),
                //[2,3]
                new BoundaryCondition2D.Dirichlet((p,_) => U(p)),
               
                ////[1,3]: 
                //new BoundaryCondition2D.Dirichlet((p, _) => U(p))
           ],
           Mesh: mesh
       );
        var dofManager = DofManager.NumerateDof(mesh, problem.BoundaryConditions);
        double[] u_const = [5,5,5,0,0,0]; //u=5
        foreach (var element in mesh.FiniteElements)
        {
            var G = integrator.CalculateLocalStiffness(element, _ => 1.0);
            var result = new double[G.Size];
            for (int i = 0;i < G.Size; i++)
            {
                for (int j = 0;j < G.Size; j++)
                {
                    result[i] += G[i, j] * u_const[i];
                }
                
            }
            for (int i = 0; i < result.Length; i++)
                Assert.Equal(0.0, result[i], 1e-13);
        }
    }
    private static Mesh2D BuildUnitMesh()
    {

        var mesh = new Mesh2D(IdentityTransform<Vector2D>.Instance);
        var elementfactory = FiniteElements.Triangle.HierarchicalQuadratic;
        var boundaryFactory = new HierarchicalQuadraticSegmentFactory();
        mesh.AddVertex(new Vector2D(0.0, 0.0)); // 0
        mesh.AddVertex(new Vector2D(1.0, 0.0)); // 1
        mesh.AddVertex(new Vector2D(0.0, 1.0)); // 2

        mesh.AddElement(elementfactory, vertices: [0, 1, 2], materialIndex: 0);
        mesh.AddBoundary(boundaryFactory, vertices: [0, 1], boundaryIndex: 0);
        mesh.AddBoundary(boundaryFactory, vertices: [0, 2], boundaryIndex: 1);
        mesh.AddBoundary(boundaryFactory, vertices: [1, 2], boundaryIndex: 2);

        return mesh;
    }
    [Fact]
    public void NonMasterLocalStiffnessMatrix_IsSymmetric()
    {
        var mesh = BuildNonMasterUnitMesh();
        static double U(Vector2D p) => p.X;
        var integrator = NumericItegrator2D.Instance;
        var problem = new HyperbolicProblem2D(
           Materials:
           [
               new (
                    Lambda: static (_, _) => 1.0,
                    Xi: static (_, _) => 0.0,
                    Sigma: static (_, _) => 1.0, //gamma
                    Source: (p, _) => U(p)
                )
           ],
           BoundaryConditions:
           [
               // [0,1]:
                new BoundaryCondition2D.Dirichlet((p, _) => U(p)),
                // [0,2]
                new BoundaryCondition2D.Dirichlet((p, _) => U(p)),
                //[2,3]
                new BoundaryCondition2D.Dirichlet((p,_) => U(p)),
               
                ////[1,3]: 
                //new BoundaryCondition2D.Dirichlet((p, _) => U(p))
           ],
           Mesh: mesh
       );
        var dofManager = DofManager.NumerateDof(mesh, problem.BoundaryConditions);
        foreach (var element in mesh.FiniteElements)
        {
            var G = integrator.CalculateLocalStiffness(element, _ => 1.0);
            for (int i = 0; i < G.Size; i++)
            {
                for (int j = 0; j < G.Size; j++)
                    Assert.Equal(G[i, j], G[j, i], 1e-13);
            }
        }
    }
    [Fact]
    public void NonMasterLocalStiffnessMultipliedByConstantIsZero()
    {
        var mesh = BuildNonMasterUnitMesh();
        static double U(Vector2D p) => 3.0;
        var integrator = NumericItegrator2D.Instance;
        var problem = new HyperbolicProblem2D(
           Materials:
           [
               new (
                    Lambda: static (_, _) => 1.0,
                    Xi: static (_, _) => 0.0,
                    Sigma: static (_, _) => 1.0, //gamma
                    Source: (p, _) => U(p)
                )
           ],
           BoundaryConditions:
           [
               // [0,1]:
                new BoundaryCondition2D.Dirichlet((p, _) => U(p)),
                // [0,2]
                new BoundaryCondition2D.Dirichlet((p, _) => U(p)),
                //[2,3]
                new BoundaryCondition2D.Dirichlet((p,_) => U(p)),
               
                ////[1,3]: 
                //new BoundaryCondition2D.Dirichlet((p, _) => U(p))
           ],
           Mesh: mesh
       );
        var dofManager = DofManager.NumerateDof(mesh, problem.BoundaryConditions);
        double[] u_const = [3, 3, 3, 0, 0, 0]; //u=3
        foreach (var element in mesh.FiniteElements)
        {
            var G = integrator.CalculateLocalStiffness(element, _ => 1.0);
            var result = new double[G.Size];
            for (int i = 0; i < G.Size; i++)
            {
                for (int j = 0; j < G.Size; j++)
                {
                    result[i] += G[i, j] * u_const[i];
                }

            }
            for (int i = 0; i < result.Length; i++)
                Assert.Equal(0.0, result[i], 1e-13);
        }
    }
    private static Mesh2D BuildNonMasterUnitMesh()
    {
        var mesh = new Mesh2D(IdentityTransform<Vector2D>.Instance);
        var elementfactory = FiniteElements.Triangle.HierarchicalQuadratic;
        var boundaryFactory = new HierarchicalQuadraticSegmentFactory();
        mesh.AddVertex(new Vector2D(0.0, 0.0)); // 0
        mesh.AddVertex(new Vector2D(1.0, 2.0)); // 1
        mesh.AddVertex(new Vector2D(3.0, 1.0)); // 2

        mesh.AddElement(elementfactory, vertices: [0, 1, 2], materialIndex: 0);
        mesh.AddBoundary(boundaryFactory, vertices: [0, 1], boundaryIndex: 0);
        mesh.AddBoundary(boundaryFactory, vertices: [0, 2], boundaryIndex: 1);
        mesh.AddBoundary(boundaryFactory, vertices: [1, 2], boundaryIndex: 2);
        return mesh;
    }

    [Fact]
    public void LocalMassMatrixIsCorrect()
    {
        static double U(Vector2D p) => p.X;
        var mesh = BuildUnitMesh();
        var integrator = NumericItegrator2D.Instance;
        var problem = new HyperbolicProblem2D(
           Materials:
           [
               new (
                    Lambda: static (_, _) => 1.0,
                    Xi: static (_, _) => 0.0,
                    Sigma: static (_, _) => 1.0, //gamma
                    Source: (p, _) => U(p)
                )
           ],
           BoundaryConditions:
           [
               // [0,1]:
                new BoundaryCondition2D.Dirichlet((p, _) => U(p)),
                // [0,2]
                new BoundaryCondition2D.Dirichlet((p, _) => U(p)),
                //[2,3]
                new BoundaryCondition2D.Dirichlet((p,_) => U(p)),

                ////[1,3]: 
                //new BoundaryCondition2D.Dirichlet((p, _) => U(p))
           ],
           Mesh: mesh
       );
        var dofManager = DofManager.NumerateDof(mesh, problem.BoundaryConditions);
        double[,] ExpectedMass = new double[6,6]
        {
        { 0.08333333333331604, 0.04166666666665918, 0.04166666666665918, 0.016666666666664262, 0.008333333333331944, 0.016666666666664262 },
        { 0.04166666666665918, 0.08333333333330661, 0.04166666666665448, 0.01666666666666297, 0.01666666666666127, 0.008333333333331942 },
        { 0.04166666666665918, 0.04166666666665448, 0.08333333333330661, 0.008333333333331944, 0.016666666666661265, 0.016666666666662975 },
        { 0.016666666666664262, 0.01666666666666297, 0.008333333333331944, 0.005555555555554573, 0.0027777777777771946, 0.002777777777777553 },
        { 0.008333333333331944, 0.01666666666666127, 0.016666666666661265, 0.0027777777777771946, 0.005555555555553519, 0.0027777777777771946 },
        { 0.016666666666664262, 0.008333333333331942, 0.016666666666662975, 0.002777777777777553, 0.0027777777777771946, 0.005555555555554575 }
        };
        foreach (var element in mesh.FiniteElements)
        {
            var M = integrator.CalculateLocalMass(element, _ => 1.0);
            for (int i = 0; i < M.Size; i++)
                for (int j = 0; j < M.Size; j++)
                    Assert.Equal(ExpectedMass[i, j], M[i, j], 1e-12);
        }
    }
    [Fact]
    public void LocalStiffnessMatrixIsCorrect()
    {
        static double U(Vector2D p) => p.X;
        var mesh = BuildUnitMesh();
        var integrator = NumericItegrator2D.Instance;
        var problem = new HyperbolicProblem2D(
           Materials:
           [
               new (
                    Lambda: static (_, _) => 1.0,
                    Xi: static (_, _) => 0.0,
                    Sigma: static (_, _) => 1.0, //gamma
                    Source: (p, _) => U(p)
                )
           ],
           BoundaryConditions:
           [
               // [0,1]:
                new BoundaryCondition2D.Dirichlet((p, _) => U(p)),
                // [0,2]
                new BoundaryCondition2D.Dirichlet((p, _) => U(p)),
                //[2,3]
                new BoundaryCondition2D.Dirichlet((p,_) => U(p)),

                ////[1,3]: 
                //new BoundaryCondition2D.Dirichlet((p, _) => U(p))
           ],
           Mesh: mesh
       );
        var dofManager = DofManager.NumerateDof(mesh, problem.BoundaryConditions);
        double[,] ExpectedStiffness = new double[6, 6]
        {
        {0.9999999999997502, -0.4999999999998751, -0.4999999999998751, 0.16666666666660618, -0.33333333333324056, 0.16666666666660618 },
        {-0.4999999999998751, 0.4999999999998751, 0, 1.4108506030119372E-14, 0.16666666666662028, -0.16666666666662028 },
        {-0.4999999999998751, 0, 0.4999999999998751, -0.1666666666666203, 0.1666666666666203, 1.410548545360155E-14 },
        {0.16666666666660618, 1.4108506030119372E-14, -0.1666666666666203, 0.166666666666611, -0.08333333333330192, -9.405670686746248E-15 },
        {-0.33333333333324056, 0.16666666666662028, 0.1666666666666203, -0.08333333333330192, 0.16666666666661323, -0.08333333333330192 },
        {0.16666666666660618, -0.16666666666662028, 1.410548545360155E-14, -9.405670686746248E-15, -0.08333333333330192, 0.16666666666661095 }
        };
        foreach (var element in mesh.FiniteElements)
        {
            var G = integrator.CalculateLocalStiffness(element, _ => 1.0);
            for (int i = 0; i < G.Size; i++)
                for (int j = 0; j < G.Size; j++)
                    Assert.Equal(ExpectedStiffness[i, j], G[i, j], 1e-12);
        }
    }
}


