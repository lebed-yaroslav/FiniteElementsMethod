using Model.Core.CoordinateSystem;
using Telma;

namespace UnitTests.Core.CoordinateSystem;


public class QuadrangleCoordinateSystemTests
{
    private const double Eps = 1e-9;

    [Fact]
    public void InverseTransform_Vertices_MapsToUnit()
    {
        var p00 = new Vector2D(1, 5);
        var p10 = new Vector2D(6, 1);
        var p11 = new Vector2D(7, 3);
        var p01 = new Vector2D(5, 6);

        var cs = new QuadrangleCoordinateSystem(p00, p10, p11, p01);

        Assert.Equal(p00, cs.InverseTransform(new(0, 0)), Eps);
        Assert.Equal(p10, cs.InverseTransform(new(1, 0)), Eps);
        Assert.Equal(p11, cs.InverseTransform(new(1, 1)), Eps);
        Assert.Equal(p01, cs.InverseTransform(new(0, 1)), Eps);
    }

    [Fact]
    public void Transform_Unit_MapsToVertices()
    {
        var p00 = new Vector2D(1, 5);
        var p10 = new Vector2D(6, 1);
        var p11 = new Vector2D(7, 3);
        var p01 = new Vector2D(5, 6);

        var cs = new QuadrangleCoordinateSystem(p00, p10, p11, p01);

        Assert.Equal(new(0, 0), cs.Transform(p00), Eps);
        Assert.Equal(new(1, 0), cs.Transform(p10), Eps);
        Assert.Equal(new(1, 1), cs.Transform(p11), Eps);
        Assert.Equal(new(0, 1), cs.Transform(p01), Eps);
    }

    [Fact]
    public void Transform_InverseTransform_PreservesPoint()
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
    public void Transform_InverseTransform_Unit_PointIsPreserved()
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
    public void Jacobian_IsPositive()
    {
        var p00 = new Vector2D(1, 5);
        var p10 = new Vector2D(6, 1);
        var p11 = new Vector2D(7, 3);
        var p01 = new Vector2D(5, 6);

        var cs = new QuadrangleCoordinateSystem(p00, p10, p11, p01);

        Assert.True(cs.Jacobian(new(0.2, 0.2)) > 0);
        Assert.True(cs.Jacobian(new(0.5, 0.5)) > 0);
        Assert.True(cs.Jacobian(new(0.8, 0.3)) > 0);
    }
}
