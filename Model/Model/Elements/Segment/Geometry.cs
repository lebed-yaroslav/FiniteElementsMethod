using Model.Core.CoordinateSystem;
using Telma;
using Telma.Extensions;

namespace Model.Model.Elements.Segment;


public class SegmentGeometry<TSpace> : ElementGeometry<TSpace, Vector1D>
    where TSpace : struct, IVectorBase<TSpace>
{
    public SegmentGeometry(int[] vertexIndices) : base(vertexIndices)
    {
        if (vertexIndices.Length != 2)
            throw new ArgumentException($"Expected 2 vertices for {nameof(SegmentGeometry<>)} but got {vertexIndices.Length}");
    }

    public override IEnumerable<Edge> Edges => [new(Vertices[0], Vertices[1])];
    public override int EdgeCount => 1;

    public override bool ContainsPoint(TSpace point, double epsilon = 1E-12)
    {
        if (TSpace.Dimensions != 1) return false;
        var xi = MasterElementCoordinateSystem.Transform(point);
        return -epsilon <= xi && xi <= 1 + epsilon;
    }

    public override ICoordinateTransform<TSpace, Vector1D> MasterElementCoordinateSystem =>
        new SegmentParametrization<TSpace>(Mesh[Vertices[0]], Mesh[Vertices[1]]);
}
