using Model.Fem.Elements;
using Model.Fem.Mesh;
using Telma;

namespace UnitTests.Elements.Triangle;

public class TriangleGeometryTests
{
    private static IFiniteElement2D CreateMeshWithTriangle(Vector2D a, Vector2D b, Vector2D c)
    {
        var mesh = new Mesh2D();
        mesh.AddVertices(a, b, c);
        return mesh.AddElement(FiniteElements.Triangle.Linear, [0, 1, 2], 0);
    }

    [Fact]
    public void ContainsPoint_IsCorrectForCw()
    {
        var triangle = CreateMeshWithTriangle(new(2, 2), new(5, 3), new(4, 6)).Geometry;
        Assert.False(triangle.ContainsPoint(new(0, 0)));
        Assert.False(triangle.ContainsPoint(new(3, 2)));
        Assert.True(triangle.ContainsPoint(new(3, 2.5)));
    }

    [Fact]
    public void ContainsPoint_IsCorrectForCcw()
    {
        var triangle = CreateMeshWithTriangle(new(4, 6), new(2, 2), new(5, 3)).Geometry;
        Assert.False(triangle.ContainsPoint(new(0, 0)));
        Assert.False(triangle.ContainsPoint(new(3, 2)));
        Assert.True(triangle.ContainsPoint(new(3, 2.5)));
    }

    [Fact]
    public void ContainsPoint_IsCorrectForEdgePoints()
    {
        var triangle = CreateMeshWithTriangle(new(0, 0), new(2, 2), new(1, 3)).Geometry;
        Assert.True(triangle.ContainsPoint(new(1, 1)));
        Assert.True(triangle.ContainsPoint(new(1.5, 2.5)));
        Assert.True(triangle.ContainsPoint(new(0.5, 1.5)));
    }
}
