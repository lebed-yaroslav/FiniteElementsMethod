using Model.Model.Elements;
using Telma;

namespace UnitTests.Elements.Quadrangle;

public class QuadrangleGeometryTests
{
    private static IFiniteElement2D CreateMeshWithQuadrangle(Vector2D p00, Vector2D p01, Vector2D p11, Vector2D p10)
    {
        var mesh = new Mesh2D();
        mesh.AddVertex(p00);
        mesh.AddVertex(p01);
        mesh.AddVertex(p11);
        mesh.AddVertex(p10);
        return mesh.AddElement(FiniteElements.Quadrangle.Bilinear, [0, 1, 2, 3], 0);
    }

    // TODO: near-edge tests

    [Fact]
    public void ContainsPoint_IsCorrectForCw()
    {
        var quadrangle = CreateMeshWithQuadrangle(new(2, 2), new(5, 3), new(4, 6), new(1, 4)).Geometry;
        Assert.False(quadrangle.ContainsPoint(new(0, 0)));
        Assert.False(quadrangle.ContainsPoint(new(3, 2)));
        Assert.True(quadrangle.ContainsPoint(new(3, 2.5)));
    }

    [Fact]
    public void ContainsPoint_IsCorrectForCcw()
    {
        var quadrangle = CreateMeshWithQuadrangle(new(1, 4), new(4, 6), new(5, 3), new(2, 2)).Geometry;
        Assert.False(quadrangle.ContainsPoint(new(0, 0)));
        Assert.False(quadrangle.ContainsPoint(new(3, 2)));
        Assert.True(quadrangle.ContainsPoint(new(3, 2.5)));
    }
}

