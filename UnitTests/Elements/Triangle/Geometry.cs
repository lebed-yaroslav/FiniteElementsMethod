using Model.Model.Elements;
using Telma;

namespace UnitTests.Elements.Triangle;

public class TriangleGeometryTests
{
    private static IFiniteElement2D CreateMeshWithTriangle(Vector2D a, Vector2D b, Vector2D c)
    {
        var mesh = new Mesh2D();
        mesh.AddVertex(a);
        mesh.AddVertex(b);
        mesh.AddVertex(c);
        return mesh.AddElement(FiniteElements.Triangle.Linear, [0, 1, 2], 0);
    }

    // TODO: near-edge tests

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
}
