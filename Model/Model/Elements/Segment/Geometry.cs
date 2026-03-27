using Model.Core.CoordinateSystem;
using Telma;
using Telma.Extensions;

namespace Model.Model.Elements.Segment;


public class SegmentGeometry<TSpace>(int[] vertexIndices) :
    ElementGeometry<TSpace, Vector1D>(vertexIndices)
    where TSpace : struct, IVectorBase<TSpace>
{
    public override IEnumerable<Edge> Edges => [new(Vertices[0], Vertices[1])];
    public override int EdgeCount => 1;

    public override ICoordinateTransform<TSpace, Vector1D> MasterElementCoordinateSystem =>
        new SegmentParametrization<TSpace>(Mesh[Vertices[0]], Mesh[Vertices[1]]);
}
