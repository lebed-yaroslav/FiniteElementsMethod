using Model.Core.CoordinateSystem;
using Model.Model.Elements.Triangle;
using Telma;

namespace Model.Model.Elements.Quadrangle;


/// <summary>
/// Represents quadrangle with local numeration:
/// <code>
///  3 --[2]-- 2
///  |         |
/// [3]       [1] <- Edge
///  |         |
///  0 --[0]-- 1  <- Vertex
/// </code>
/// </summary>
/// <param name="vertexIndices">4 indices of vertices in following order</param>
public sealed class QuadrangleGeometry : VolumeElementGeometry2D
{
    public QuadrangleGeometry(int[] vertexIndices) : base(vertexIndices)
    {
        if (vertexIndices.Length != 4)
            throw new ArgumentException($"Expected 4 vertices for {nameof(QuadrangleGeometry)} but got {vertexIndices.Length}");
    }

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

    public override bool ContainsPoint(Vector2D point, double epsilon = 1E-12) {
        var (xi, eta) = MasterElementCoordinateSystem.Transform(point);
        return
            xi >= -epsilon &&
            xi <= 1 + epsilon &&
            eta >= -epsilon &&
            eta <= 1 + epsilon;
    }

    public override ICoordinateTransform<Vector2D, Vector2D> MasterElementCoordinateSystem =>
        new QuadrangleCoordinateSystem(
            Mesh[Vertices[0]], Mesh[Vertices[1]], Mesh[Vertices[2]], Mesh[Vertices[3]]
        );
}
