using Model.Core.CoordinateSystem;
using Telma;

namespace Model.Model.Elements.Quadrangle;


public sealed class QuadrangleGeometry(int[] vertexIndices) :
    ElementGeometry<Vector2D>(vertexIndices),
    IVolumeElementGeometry<Vector2D>
{
    public override IEnumerable<Edge> Edges
    {
        get
        {
            yield return new(Vertices[0], Vertices[1]);
            yield return new(Vertices[1], Vertices[2]);
            yield return new(Vertices[2], Vertices[3]);
            yield return new(Vertices[3], Vertices[0]);
        }
    }
    public override int EdgeCount => 4;

    public ICoordinateTransform<Vector2D, Vector2D> MasterElementCoordinateSystem =>
        new QuadrangleCoordinateSystem(
            Mesh[Vertices[0]], Mesh[Vertices[1]], Mesh[Vertices[2]], Mesh[Vertices[3]]
        );
}
