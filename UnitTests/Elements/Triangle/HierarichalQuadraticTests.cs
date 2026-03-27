using Model.Core.CoordinateSystem;
using Model.Model;
using Model.Model.Elements;
using Model.Model.Elements.Segment;
using Model.Model.Mesh;
using Telma;

namespace UnitTests;


public class HierarichalQuadraticTests
{
    [Fact]
    public void LocalStiffnessMatrix_IsSymmetric()
    {
        var mesh = BuildUnitMesh();
        foreach (var element in mesh.FiniteElements)
        {
            var G = NumericIntegrator.CalculateLocalStiffness(element, _ => 1.0);
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
        var mesh = BuildUnitMesh();

        double[] u_expected = [5,5,5,0,0,0]; //u=5
        foreach (var element in mesh.FiniteElements)
        {
            var G = NumericIntegrator.CalculateLocalStiffness(element, _ => 1.0);
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

        mesh.AddVertex(new Vector2D(0.0, 0.0)); // 0
        mesh.AddVertex(new Vector2D(1.0, 0.0)); // 1
        mesh.AddVertex(new Vector2D(0.0, 1.0)); // 2

        mesh.AddElement(FiniteElements.Triangle.HierarchicalQuadratic, vertices: [0, 1, 2], materialIndex: 0);

        return mesh;
    }

}


