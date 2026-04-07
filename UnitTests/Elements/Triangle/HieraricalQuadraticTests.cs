using System.ComponentModel.DataAnnotations;
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
    private static string NotStandardTriangle => """
        3
        0 0 
        2 0
        0 1
        1
        0 1 2 10
        3
        1 2 0
        0 1 0
        0 2 0
        """;
    [Fact]
    public void LocalStiffnessAndMassIsCorrect()
    {
        var mesh = new StringReader(NotStandardTriangle).ReadMesh2D(
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
        var integrator = NumericItegrator2D.Instance;
        var dofManager = DofManager.NumerateDof(mesh, problem.BoundaryConditions);
        var ExpectedStiffness = new double[6, 6]
             {
            {1.2499999999996874, -0.24999999999993755, -0.9999999999997502, 0.33333333333323356, -0.41666666666655067, 0.08333333333328193 },
            {-0.24999999999993755, 0.24999999999993755, 0, 7.054253015059686E-15, 0.08333333333331014, -0.08333333333331014 },
            {-0.9999999999997502, 0, 0.9999999999997502, -0.3333333333332406, 0.3333333333332406, 2.82109709072031E-14 },
            {0.33333333333323356, 7.054253015059686E-15, -0.3333333333332406, 0.20833333333326545, -0.16666666666661092, -1.175708835843281E-14 },
            {-0.41666666666655067, 0.08333333333331014, 0.3333333333332406, -0.16666666666661092, 0.2083333333332666, -0.04166666666664391 },
            {0.08333333333328193, -0.08333333333331014, 2.82109709072031E-14, -1.175708835843281E-14, -0.04166666666664391, 0.20833333333326193 }
            };
        foreach (var element in  mesh.FiniteElements)
        {
            var G = integrator.CalculateLocalStiffness(element, _ => 1.0);
            for (int i = 0; i < G.Size; i++)
            {
                for (int j = 0; j < G.Size; j++)
                    Assert.Equal(ExpectedStiffness[i,j], G[i, j],1e-12);
            }
        }
        var ExpectedMass = new double[6, 6]
            {
            {0.16666666666663207, 0.08333333333331835, 0.08333333333331835, 0.033333333333328524, 0.016666666666663887, 0.033333333333328524 },
            {0.08333333333331835, 0.16666666666661323, 0.08333333333330896, 0.03333333333332594, 0.03333333333332254, 0.016666666666663884 },
            {0.08333333333331835, 0.08333333333330896, 0.16666666666661323, 0.016666666666663887, 0.03333333333332253, 0.03333333333332595 },
            {0.033333333333328524, 0.03333333333332594, 0.016666666666663887, 0.011111111111109146, 0.005555555555554389, 0.005555555555555106 },
            {0.016666666666663887, 0.03333333333332254, 0.03333333333332253, 0.005555555555554389, 0.011111111111107038, 0.005555555555554389 },
            {0.033333333333328524, 0.016666666666663884, 0.03333333333332595, 0.005555555555555106, 0.005555555555554389, 0.01111111111110915 }
            };
        foreach (var element in mesh.FiniteElements)
        {
            var M = integrator.CalculateLocalMass(element, _ => 1.0);
            for (int i = 0; i < M.Size; i++)
            {
                for (int j = 0; j < M.Size; j++)
                    Assert.Equal(ExpectedMass[i, j], M[i, j],1e-12);
            }
        }
    }
}

