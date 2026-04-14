using Model.Fem.Elements;
using Model.Fem.Mesh;
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

    public static TheoryData<Vector2D, Vector2D, Vector2D, Vector2D> Quadrangles { get; } = new()
    {
        { new(0, 0), new(1, 0), new(1, 1), new(0, 1) }
    };

    [Theory]
    [MemberData(nameof(Quadrangles))]
    public void ContainsPoint_Vertices(Vector2D p00, Vector2D p10, Vector2D p11, Vector2D p01)
    {
        var quadrangle = CreateMeshWithQuadrangle(p00, p10, p11, p01);
        Assert.All(
            [p00, p10, p11, p01],
            vertex => Assert.True(quadrangle.ContainsPoint(vertex))
        );
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

