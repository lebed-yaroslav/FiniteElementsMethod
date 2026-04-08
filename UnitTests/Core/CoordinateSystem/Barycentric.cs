using Model.Core.CoordinateSystem;
using Telma;

namespace UnitTests.Core.CoordinateSystem;


public class BarycentricCoordinateSystemTests
{
    private const double Eps = 1e-10;

    [Fact]
    public void InverseTransform_Vertices_MapsToUnit()
    {
        var a = new Vector2D(1, 2);
        var b = new Vector2D(5, 2);
        var c = new Vector2D(2, 6);

        var cs = new BarycentricCoordinateSystem(a, b, c);

        Assert.Equal(a, cs.InverseTransform(new(0, 0)), Eps);
        Assert.Equal(b, cs.InverseTransform(new(1, 0)), Eps);
        Assert.Equal(c, cs.InverseTransform(new(0, 1)), Eps);
    }

    [Fact]
    public void Transform_Unit_MapsToVertices()
    {
        var a = new Vector2D(1, 2);
        var b = new Vector2D(5, 2);
        var c = new Vector2D(2, 6);

        var cs = new BarycentricCoordinateSystem(a, b, c);

        Assert.Equal(new(0, 0), cs.Transform(a), Eps);
        Assert.Equal(new(1, 0), cs.Transform(b), Eps);
        Assert.Equal(new(0, 1), cs.Transform(c), Eps);
    }

    [Fact]
    public void Transform_InverseTransform_PreservesPoint()
    {
        var a = new Vector2D(1, 2);
        var b = new Vector2D(5, 2);
        var c = new Vector2D(2, 6);

        var cs = new BarycentricCoordinateSystem(a, b, c);

        var localPoint = new Vector2D(0.2, 0.3);
        var physicalPoint = cs.InverseTransform(localPoint);
        var restoredPoint = cs.Transform(physicalPoint);

        Assert.Equal(localPoint, restoredPoint, Eps);
    }

    [Fact]
    public void InverseTransform_MapsCentroidCorrectly()
    {
        var a = new Vector2D(1, 2);
        var b = new Vector2D(5, 2);
        var c = new Vector2D(2, 6);

        var cs = new BarycentricCoordinateSystem(a, b, c);

        var localCentroid = new Vector2D(1.0 / 3.0, 1.0 / 3.0);
        var expected = new Vector2D(
            (a.X + b.X + c.X) / 3.0,
            (a.Y + b.Y + c.Y) / 3.0
        );

        var actual = cs.InverseTransform(localCentroid);

        Assert.Equal(expected, actual, Eps);
    }

    [Fact]
    public void Jacobian_IsConstant()
    {
        var a = new Vector2D(1, 2);
        var b = new Vector2D(5, 2);
        var c = new Vector2D(2, 6);

        var cs = new BarycentricCoordinateSystem(a, b, c);

        var j1 = cs.Jacobian(new Vector2D(0.1, 0.1));
        var j2 = cs.Jacobian(new Vector2D(0.2, 0.3));
        var j3 = cs.Jacobian(new Vector2D(0.4, 0.1));

        Assert.Equal(j1, j2, Eps);
        Assert.Equal(j2, j3, Eps);
    }

    public static TheoryData<Vector2D, Vector2D, Vector2D, double> TrianglesWithArea { get; } = new() {
        { new(0, 0), new(1, 0), new(0, 1), 0.5 },
        { new(-1, -1), new(1, -1), new(0, 10), 11 },
        { new(0, 0), new(4, 3), new(1, 7), 12.5 }
    };

    [Theory]
    [MemberData(nameof(TrianglesWithArea))]
    public void Jacobian_IsEqualToDoubledArea(Vector2D a, Vector2D b, Vector2D c, double area)
    {
        var cs = new BarycentricCoordinateSystem(a, b, c);
        Assert.Equal(2 * area, Math.Abs(cs.Jacobian(Vector2D.Zero)), Eps);
    }
}
