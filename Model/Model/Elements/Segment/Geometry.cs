using Model.Core.CoordinateSystem;
using Telma;
using Telma.Extensions;

namespace Model.Model.Elements.Segment;


public class SegmentGeometry<TSpace> :
    ElementGeometry<TSpace>
    where TSpace : struct, IVectorBase<TSpace>
{
    public override IEnumerable<Edge> Edges => [new(Vertices[0], Vertices[1])];
    public override int EdgeCount => 1;

    private SegmentGeometry(int[] vertexIndices) : base(vertexIndices) { }

    public ICoordinateTransform<TSpace, Vector1D> MasterElementCoordinateSystem =>
        new SegmentParametrization<TSpace>(Mesh[Vertices[0]], Mesh[Vertices[1]]);

    public sealed class Boundary(int[] vertexIndices) :
        SegmentGeometry<Vector2D>(vertexIndices),
        IBoundaryElementGeometry2D;

    // For potential future usage (or for example how to turn triangle into 2D boundary)
    public sealed class Volume(int[] vertexIndices) :
        SegmentGeometry<Vector1D>(vertexIndices),
        IVolumeElementGeometry1D;
}
