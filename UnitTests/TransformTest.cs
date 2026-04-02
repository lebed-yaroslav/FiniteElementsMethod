using Model.Core.CoordinateSystem;
using Telma;
using Xunit;

namespace UnitTests.Core.CoordinateSystem;

public class BarycentricCoordinateSystemTests
{
    private const double Eps = 1e-10;

    private static void AssertVectorEqual(Vector2D expected, Vector2D actual)
    {
        Assert.True(Math.Abs(expected.X - actual.X) < Eps);
        Assert.True(Math.Abs(expected.Y - actual.Y) < Eps);
    }

    [Fact]
    public void InverseTransform_Vertices()
    {
        var a = new Vector2D(1, 2);
        var b = new Vector2D(5, 2);
        var c = new Vector2D(2, 6);

        var cs = new BarycentricCoordinateSystem(a, b, c);

        AssertVectorEqual(a, cs.InverseTransform(new Vector2D(0, 0)));
        AssertVectorEqual(b, cs.InverseTransform(new Vector2D(1, 0)));
        AssertVectorEqual(c, cs.InverseTransform(new Vector2D(0, 1)));
    }

    [Fact]
    public void Transform_Vertices()
    {
        var a = new Vector2D(1, 2);
        var b = new Vector2D(5, 2);
        var c = new Vector2D(2, 6);

        var cs = new BarycentricCoordinateSystem(a, b, c);

        AssertVectorEqual(new Vector2D(0, 0), cs.Transform(a));
        AssertVectorEqual(new Vector2D(1, 0), cs.Transform(b));
        AssertVectorEqual(new Vector2D(0, 1), cs.Transform(c));
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

        AssertVectorEqual(localPoint, restoredPoint);
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

        AssertVectorEqual(expected, actual);
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

        Assert.True(Math.Abs(j1 - j2) < Eps);
        Assert.True(Math.Abs(j2 - j3) < Eps);
    }
}

public class QuadrangleCoordinateSystemTests
{
    private const double Eps = 1e-9;

    private static void AssertVectorEqual(Vector2D expected, Vector2D actual)
    {
        Assert.True(Math.Abs(expected.X - actual.X) < Eps);
        Assert.True(Math.Abs(expected.Y - actual.Y) < Eps);
    }

    [Fact]
    public void InverseTransform_Corners()
    {
        var p00 = new Vector2D(1, 5);
        var p10 = new Vector2D(6, 1);
        var p11 = new Vector2D(7, 3);
        var p01 = new Vector2D(5, 6);

        var cs = new QuadrangleCoordinateSystem(p00, p10, p11, p01);

        AssertVectorEqual(p00, cs.InverseTransform(new Vector2D(0, 0)));
        AssertVectorEqual(p10, cs.InverseTransform(new Vector2D(1, 0)));
        AssertVectorEqual(p11, cs.InverseTransform(new Vector2D(1, 1)));
        AssertVectorEqual(p01, cs.InverseTransform(new Vector2D(0, 1)));
    }

    [Fact]
    public void Transform_Corners()
    {
        var p00 = new Vector2D(1, 5);
        var p10 = new Vector2D(6, 1);
        var p11 = new Vector2D(7, 3);
        var p01 = new Vector2D(5, 6);

        var cs = new QuadrangleCoordinateSystem(p00, p10, p11, p01);

        AssertVectorEqual(new Vector2D(0, 0), cs.Transform(p00));
        AssertVectorEqual(new Vector2D(1, 0), cs.Transform(p10));
        AssertVectorEqual(new Vector2D(1, 1), cs.Transform(p11));
        AssertVectorEqual(new Vector2D(0, 1), cs.Transform(p01));
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

        AssertVectorEqual(localPoint, restoredPoint);
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

        AssertVectorEqual(point, cs.Transform(point));
        AssertVectorEqual(point, cs.InverseTransform(point));
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
