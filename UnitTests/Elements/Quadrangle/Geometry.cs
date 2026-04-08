using Model.Model.Elements;
using Model.Model.Mesh;
using Telma;

namespace UnitTests.Elements.Quadrangle;

public class QuadrangleGeometryTests
{
    private static IFiniteElement2D CreateMeshWithQuadrangle(Vector2D p00, Vector2D p10, Vector2D p11, Vector2D p01)
    {
        var mesh = new Mesh2D();
        mesh.AddVertices(p00, p10, p11, p01);
        return mesh.AddElement(FiniteElements.Quadrangle.Bilinear, [0, 1, 2, 3], 0);
    }

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

    [Fact]
    public void ContainsPoint_IsCorrectForEdgePoints()
    {
        var quadrangle = CreateMeshWithQuadrangle(new(1, 4), new(4, 6), new(5, 3), new(2, 2)).Geometry;
        Assert.True(quadrangle.ContainsPoint(new(2.5, 5)));
        Assert.True(quadrangle.ContainsPoint(new(4.5, 4.5)));
        Assert.True(quadrangle.ContainsPoint(new(3.5, 2.5)));
        Assert.True(quadrangle.ContainsPoint(new(1.5, 3)));
    }
}

