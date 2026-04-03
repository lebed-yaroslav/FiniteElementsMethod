using Model.Core.CoordinateSystem;
using Telma;

namespace UnitTests.Core.CoordinateSystem;

public class BarycentricCoordinateSystemTests
{
    private const double Eps = 1e-10;

    [Fact]
    public void InverseTransform_Vertices()
    {
        var a = new Vector2D(1, 2);
        var b = new Vector2D(5, 2);
        var c = new Vector2D(2, 6);

        var cs = new BarycentricCoordinateSystem(a, b, c);

        Assert.Equal(a, cs.InverseTransform(new Vector2D(0, 0)), Eps);
        Assert.Equal(b, cs.InverseTransform(new Vector2D(1, 0)), Eps);
        Assert.Equal(c, cs.InverseTransform(new Vector2D(0, 1)), Eps);
    }

    [Fact]
    public void Transform_Vertices()
    {
        var a = new Vector2D(1, 2);
        var b = new Vector2D(5, 2);
        var c = new Vector2D(2, 6);

        var cs = new BarycentricCoordinateSystem(a, b, c);

        Assert.Equal(new Vector2D(0, 0), cs.Transform(a), Eps);
        Assert.Equal(new Vector2D(1, 0), cs.Transform(b), Eps);
        Assert.Equal(new Vector2D(0, 1), cs.Transform(c), Eps);
    }

    [Fact]
    public void Transform_InverseTransform_Point()
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
    public void InverseTransform_Center()
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
    public void Jacobian_Constant()
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
}

public class QuadrangleCoordinateSystemTests
{
    private const double Eps = 1e-9;

    [Fact]
    public void InverseTransform_Corners()
    {
        var p00 = new Vector2D(1, 5);
        var p10 = new Vector2D(6, 1);
        var p11 = new Vector2D(7, 3);
        var p01 = new Vector2D(5, 6);

        var cs = new QuadrangleCoordinateSystem(p00, p10, p11, p01);

        Assert.Equal(p00, cs.InverseTransform(new Vector2D(0, 0)), Eps);
        Assert.Equal(p10, cs.InverseTransform(new Vector2D(1, 0)), Eps);
        Assert.Equal(p11, cs.InverseTransform(new Vector2D(1, 1)), Eps);
        Assert.Equal(p01, cs.InverseTransform(new Vector2D(0, 1)), Eps);
    }

    [Fact]
    public void Transform_Corners()
    {
        var p00 = new Vector2D(1, 5);
        var p10 = new Vector2D(6, 1);
        var p11 = new Vector2D(7, 3);
        var p01 = new Vector2D(5, 6);

        var cs = new QuadrangleCoordinateSystem(p00, p10, p11, p01);

        Assert.Equal(new Vector2D(0, 0), cs.Transform(p00), Eps);
        Assert.Equal(new Vector2D(1, 0), cs.Transform(p10), Eps);
        Assert.Equal(new Vector2D(1, 1), cs.Transform(p11), Eps);
        Assert.Equal(new Vector2D(0, 1), cs.Transform(p01), Eps);
    }

    [Fact]
    public void Transform_InverseTransform_Point()
    {
        var p00 = new Vector2D(1, 5);
        var p10 = new Vector2D(6, 1);
        var p11 = new Vector2D(7, 3);
        var p01 = new Vector2D(5, 6);

        var cs = new QuadrangleCoordinateSystem(p00, p10, p11, p01);

        var localPoint = new Vector2D(0.3, 0.4);
        var physicalPoint = cs.InverseTransform(localPoint);
        var restoredPoint = cs.Transform(physicalPoint);

        Assert.Equal(localPoint, restoredPoint, Eps);
    }

    [Fact]
    public void UnitSquare_SamePoint()
    {
        var p00 = new Vector2D(0, 0);
        var p10 = new Vector2D(1, 0);
        var p11 = new Vector2D(1, 1);
        var p01 = new Vector2D(0, 1);

        var cs = new QuadrangleCoordinateSystem(p00, p10, p11, p01);

        var point = new Vector2D(0.25, 0.75);

        Assert.Equal(point, cs.Transform(point), Eps);
        Assert.Equal(point, cs.InverseTransform(point), Eps);
    }

    [Fact]
    public void Jacobian_Positive()
    {
        var p00 = new Vector2D(1, 5);
        var p10 = new Vector2D(6, 1);
        var p11 = new Vector2D(7, 3);
        var p01 = new Vector2D(5, 6);

        var cs = new QuadrangleCoordinateSystem(p00, p10, p11, p01);

        Assert.True(cs.Jacobian(new Vector2D(0.2, 0.2)) > 0);
        Assert.True(cs.Jacobian(new Vector2D(0.5, 0.5)) > 0);
        Assert.True(cs.Jacobian(new Vector2D(0.8, 0.3)) > 0);
    }
}
