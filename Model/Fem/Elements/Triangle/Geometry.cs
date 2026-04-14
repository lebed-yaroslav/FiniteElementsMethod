using Model.Core.CoordinateSystem;
using Telma;

namespace Model.Fem.Elements.Triangle;


public sealed class TriangleGeometry : VolumeElementGeometry2D
{
    public TriangleGeometry(int[] vertexIndices) : base(vertexIndices)
    {
        if (vertexIndices.Length != 3)
            throw new ArgumentException($"Expected 3 vertices for {nameof(TriangleGeometry)} but got {vertexIndices.Length}");
    }

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

    public override bool ContainsPoint(Vector2D point, double epsilon = 1E-12)
    {
        var (xi, eta) = MasterElementCoordinateSystem.Transform(point);
        return
            xi >= -epsilon && 
            eta >= -epsilon && 
            (xi + eta) <= 1 + epsilon;
    }

    public override ICoordinateTransform<Vector2D, Vector2D> MasterElementCoordinateSystem =>
        new BarycentricCoordinateSystem(
            Mesh[Vertices[0]], Mesh[Vertices[1]], Mesh[Vertices[2]]
        );
}
