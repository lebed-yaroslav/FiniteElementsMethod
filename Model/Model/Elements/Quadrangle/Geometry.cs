using Model.Core.CoordinateSystems;

namespace Model.Model.Elements.Rectangle;
public sealed class RectangleGeometry(int[] vertexIndices) : FiniteElementGeometry(vertexIndices)
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

    public override CartesianCoordinateSystem MasterElementCoordinateSystem => new(
        Mesh[Vertices[0]], Mesh[Vertices[1]], Mesh[Vertices[2]], Mesh[Vertices[3]]
    );
}
