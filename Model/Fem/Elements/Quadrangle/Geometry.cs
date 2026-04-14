using Model.Core.CoordinateSystem;
using Model.Core.Util;
using Telma;

namespace Model.Fem.Elements.Quadrangle;


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

    public override bool ContainsPoint(Vector2D point, double epsilon = 1E-12)
    {
        var p00 = Mesh[Vertices[0]];
        var p01 = Mesh[Vertices[1]];
        var p11 = Mesh[Vertices[2]];
        var p10 = Mesh[Vertices[3]];

        int refOrientation = MathUtils.ToleratedSign((p01 - p00).Cross(point - p00), epsilon);

        // Checks if is orientation preserved
        bool Check(Vector2D a, Vector2D b)
        {
            int orientation = MathUtils.ToleratedSign((b - a).Cross(point - a), epsilon);
            if (orientation == 0) return true; // Near edge
            if (refOrientation != 0) return orientation == refOrientation;
            refOrientation = orientation;
            return true;
        }

        return Check(p01, p11) && Check(p11, p10) && Check(p10, p00);
    }

    public override ICoordinateTransform<Vector2D, Vector2D> MasterElementCoordinateSystem =>
        new QuadrangleCoordinateSystem(
            Mesh[Vertices[0]], Mesh[Vertices[1]], Mesh[Vertices[2]], Mesh[Vertices[3]]
        );
}
