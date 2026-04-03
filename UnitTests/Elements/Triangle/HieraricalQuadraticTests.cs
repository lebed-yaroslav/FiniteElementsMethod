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
                    Source: (p, _) => p.X
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
        double[] u_expected = [5,5,5,0,0,0]; //u=5
        foreach (var element in mesh.FiniteElements)
        {
            var G = integrator.CalculateLocalStiffness(element, _ => 1.0);
            var result = new double[G.Size];
            for (int i = 0;i < G.Size; i++)
            {
                for (int j = 0;j < G.Size; j++)
                {
                    result[i] += G[i, j] * u_expected[i];
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
        static double U(Vector2D p) => 5.0;
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
        double[] u_expected = [3, 3, 3, 0, 0, 0]; //u=3
        foreach (var element in mesh.FiniteElements)
        {
            var G = integrator.CalculateLocalStiffness(element, _ => 1.0);
            var result = new double[G.Size];
            for (int i = 0; i < G.Size; i++)
            {
                for (int j = 0; j < G.Size; j++)
                {
                    result[i] += G[i, j] * u_expected[i];
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
}


