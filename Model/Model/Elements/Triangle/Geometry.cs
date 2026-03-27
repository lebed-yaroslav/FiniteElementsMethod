using Model.Core.CoordinateSystem;
using Telma;

namespace Model.Model.Elements.Triangle;


public sealed class TriangleGeometry(int[] vertexIndices) :
    VolumeElementGeometry2D(vertexIndices)
{
    public override IEnumerable<Edge> Edges
    {
        get
        {
            yield return new(Vertices[0], Vertices[1]);
            yield return new(Vertices[1], Vertices[2]);
            yield return new(Vertices[2], Vertices[0]);
        }
    }

    public override int EdgeCount => 3;

    public override ICoordinateTransform<Vector2D, Vector2D> MasterElementCoordinateSystem =>
        new BarycentricCoordinateSystem(
            Mesh[Vertices[0]], Mesh[Vertices[1]], Mesh[Vertices[2]]
        );
}
