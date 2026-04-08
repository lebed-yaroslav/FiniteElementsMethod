using Model.Core.CoordinateSystem;
using Telma;

namespace UnitTests.Core.CoordinateSystem;

public class SegmentParametrization2DTests
{
    private const double Eps = 1e-10;

    public static TheoryData<Vector2D, Vector2D, double> SegmentsWithLength { get; } = new()
    {
        { new(0, 0), new(1, 0), 1.0 },
        { new(5, 2), new(8, 6), 5.0 },
        { new(1, 5), new(-5, 5), 6.0 },
        { new(2, 3), new(-5, -5), Math.Sqrt(113) }
    };

    [Theory]
    [MemberData(nameof(SegmentsWithLength))]
    public void InverseTransform_Vertices_MapsToUnit(Vector2D a, Vector2D b, double _)
    {
        var param = new SegmentParametrization<Vector2D>(a, b);
        Assert.Equal(a, param.InverseTransform(0), Eps);
        Assert.Equal(b, param.InverseTransform(1), Eps);
    }

    [Theory]
    [MemberData(nameof(SegmentsWithLength))]
    public void Transform_Unit_MapsToVertices(Vector2D a, Vector2D b, double _)
    {
        var cs = new SegmentParametrization<Vector2D>(a, b);
        Assert.Equal(0, cs.Transform(a), Eps);
        Assert.Equal(1, cs.Transform(b), Eps);
    }

    [Theory]
    [MemberData(nameof(SegmentsWithLength))]
    public void Transform_InverseTransform_PreservesPoint(Vector2D a, Vector2D b, double _)
    {
        var cs = new SegmentParametrization<Vector2D>(a, b);
        var localPoint = 0.2;
        var physicalPoint = cs.InverseTransform(localPoint);
        var restoredPoint = cs.Transform(physicalPoint);
        Assert.Equal(localPoint, restoredPoint, Eps);
    }

    [Theory]
    [MemberData(nameof(SegmentsWithLength))]
    public void InverseTransform_MapsCentroidCorrectly(Vector2D a, Vector2D b, double _)
    {
        var cs = new SegmentParametrization<Vector2D>(a, b);
        var localCentroid = 0.5;
        var expected = 0.5 * (a + b);
        var actual = cs.InverseTransform(localCentroid);
        Assert.Equal(expected, actual, Eps);
    }

    [Theory]
    [MemberData(nameof(SegmentsWithLength))]
    public void Jacobian_IsConstant(Vector2D a, Vector2D b, double _)
    {
        var cs = new SegmentParametrization<Vector2D>(a, b);
        var j1 = cs.Jacobian(0.1);
        var j2 = cs.Jacobian(0.3);
        var j3 = cs.Jacobian(0.7);

        Assert.Equal(j1, j2, Eps);
        Assert.Equal(j2, j3, Eps);
    }

    [Theory]
    [MemberData(nameof(SegmentsWithLength))]
    public void Jacobian_IsEqualToLength(Vector2D a, Vector2D b, double length)
    {
        var cs = new SegmentParametrization<Vector2D>(a, b);
        Assert.Equal(length, Math.Abs(cs.Jacobian(0)), Eps);
    }
}
