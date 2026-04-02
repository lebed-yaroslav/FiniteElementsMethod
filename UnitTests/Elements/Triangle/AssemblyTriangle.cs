
using System.Diagnostics;
using Model.Core.CoordinateSystem;
using Model.Core.Matrix;
using Model.Core.Solver;
using Model.Core.Solver.Precondition;
using Model.Model;
using Model.Model.Assembly;
using Model.Model.Elements;
using Model.Model.Elements.Segment;
using Model.Model.Integrator;
using Model.Model.Mesh;
using Model.Model.Problem;
using Telma;
using DofManager = Model.Model.Assembly.DofManager;

namespace UnitTests.Elements.Triangle;


public class AssemblyTriangleTest
{
    [Fact]
    public void CorrectAssembly()
    {
        static double U(Vector2D p) => 5.0;
        //var mesh = Build2UnitsMesh();
        var cartesianCS = IdentityTransform<Vector2D>.Instance;
        var reader = new StringReader(TwoTrianglesWithDirchletNeumann);
        var elementfactory = FiniteElements.Triangle.HierarchicalQuadratic;
        var boundaryFactory = new HierarchicalQuadraticSegmentFactory();

        var mesh = MeshInput.ReadMesh(reader, cartesianCS, elementfactory, boundaryFactory);
       
        var matrixFactory = new CsrMatrixFactory();
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
                new BoundaryCondition2D.Neumann(static (_, _) => 0.0),
                //[2,3]
                new BoundaryCondition2D.Dirichlet((p,_) => U(p)),
               
                //[1,3]: 
                new BoundaryCondition2D.Neumann(static (_, _) => 0.0)
           ],
           Mesh: mesh
       );
        
        var dofManager = DofManager.NumerateDof((IMeshWithBoundaries<Vector2D,Vector1D>)problem.Mesh, problem.BoundaryConditions);
        var assembler = new Assembler2D(mesh, dofManager, matrixFactory, integrator);
        var algebraicSolver = new PCGSolver(
        PreconditionerCreator: matrix => CsrILUFactorization.Create((CsrMatrix)matrix)
    );
        var ellipticSolver = new EllipticSolver2D(assembler, algebraicSolver);
        double[] solution = ellipticSolver.Solve(problem);
        foreach (var val in solution)
        {
            Assert.Equal(5.0, val, 1e-10);
        }
    }

    private static string TwoTrianglesWithDirchletNeumann => """
        4
        0 0
        1 0 
        0 1
        1 1
        2
        0 1 2 10
        1 2 3 10
        4
        0 1 1 
        0 2 2 
        2 3 1 
        1 3 2 
        """;
    private static Mesh2D Build2UnitsMesh()
    {
        var cartesianCS = IdentityTransform<Vector2D>.Instance;
        var mesh = new Mesh2D(cartesianCS);

        mesh.AddVertex(new Vector2D(0,0));
        mesh.AddVertex(new Vector2D(1,0));
        mesh.AddVertex(new Vector2D(0,1));
        mesh.AddVertex(new Vector2D(1,1));

        var elementfactory = FiniteElements.Triangle.HierarchicalQuadratic;
        var boundaryFactory = new HierarchicalQuadraticSegmentFactory();

        mesh.AddElement(elementfactory, vertices: [0, 1, 2], materialIndex: 0);
        mesh.AddElement(elementfactory, vertices: [1, 2, 3], materialIndex: 0);

        mesh.AddBoundary(boundaryFactory, vertices: [0, 1], boundaryIndex: 0);
        mesh.AddBoundary(boundaryFactory, vertices: [0, 2], boundaryIndex: 1);
        mesh.AddBoundary(boundaryFactory, vertices: [2, 3], boundaryIndex: 2);
        mesh.AddBoundary(boundaryFactory, vertices: [1, 3], boundaryIndex: 3);

        return mesh;
    }
}
   
//var assembler = new Assembler2D(mesh, dofManager, matrixFactory, integrator);

//    Func<int, Func<Vector2D, double>> lambdaByMaterial = materialIndex => (point => 1.0);
//    Func<int, Func<Vector2D, double>> sourceByMaterial = materialIndex => (point => 5.0);

//        assembler.ResetSystemMatrix();
//        assembler.ResetLoadVector();
//        assembler.CalculateStiffness(lambdaByMaterial);
     
       
//        assembler.CalculateLoad(sourceByMaterial);

//       
//        Assert.NotNull(assembler.Matrix);

//        
//        Assert.Equal(dofManager.FreeDofCount, assembler.RhsVector.Length);

//        
//        Assert.Equal(dofManager.FixedDofCount, assembler.FixedSolution.Length);




