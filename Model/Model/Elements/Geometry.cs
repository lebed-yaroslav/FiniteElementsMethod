using Model.Core.CoordinateSystem;
using Model.Model.Mesh;
using Telma.Extensions;

namespace Model.Model.Elements;


public readonly record struct Edge(int I, int J)
{
    public Edge Sorted() => Sorted(out _);
    public Edge Sorted(out bool isOrientationFlipped)
    {
        isOrientationFlipped = I > J;
        return isOrientationFlipped ? new(J, I) : this;
    }
}

public interface IElementGeometry<TSpace>
    where TSpace : IVectorBase<TSpace>
{
    IMesh<TSpace> Mesh { get; }

    ReadOnlySpan<int> Vertices { get; }
    IEnumerable<Edge> Edges { get; }
    int EdgeCount { get; }
}

public interface IElementGeometryBase<TSpace>
    where TSpace : IVectorBase<TSpace>
{
    IMesh<TSpace> Mesh { get; }
    ReadOnlySpan<int> Vertices { get; }
    IEnumerable<Edge> Edges { get; }
    int EdgeCount { get; }

    bool ContainsPoint(TSpace point, double epsilon = 1e-12);
}

public interface IElementGeometry<TSpace, TBoundary> : IElementGeometryBase<TSpace>
    where TSpace : IVectorBase<TSpace>
    where TBoundary : IVectorBase<TBoundary>
{
    ICoordinateTransform<TSpace, TBoundary> MasterElementCoordinateSystem { get; }
}

public abstract class ElementGeometry<TSpace, TBoundary>(int[] vertexIndices) : IElementGeometry<TSpace, TBoundary>
    where TSpace : IVectorBase<TSpace>
    where TBoundary : IVectorBase<TBoundary>
{
    public required IMesh<TSpace> Mesh { get; init; }

    protected int[] _vertexIndices = vertexIndices;
    public ReadOnlySpan<int> Vertices => _vertexIndices;
    public abstract IEnumerable<Edge> Edges { get; }
    public abstract int EdgeCount { get; }

    public abstract bool ContainsPoint(TSpace point, double epsilon = 1e-12);
    public abstract ICoordinateTransform<TSpace, TBoundary> MasterElementCoordinateSystem { get; }
}
