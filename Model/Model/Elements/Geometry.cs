global using IElementGeometry1D = Model.Model.Elements.IElementGeometry<Telma.Vector1D>;
global using IElementGeometry2D = Model.Model.Elements.IElementGeometry<Telma.Vector2D>;
global using IElementGeometry3D = Model.Model.Elements.IElementGeometry<Telma.Vector3D>;
global using VolumeElementGeometry1D = Model.Model.Elements.ElementGeometry<Telma.Vector1D, Telma.Vector1D>;
global using VolumeElementGeometry2D = Model.Model.Elements.ElementGeometry<Telma.Vector2D, Telma.Vector2D>;
global using VolumeElementGeometry3D = Model.Model.Elements.ElementGeometry<Telma.Vector3D, Telma.Vector3D>;

global using BoundaryElementGeometry2D = Model.Model.Elements.ElementGeometry<Telma.Vector2D, Telma.Vector1D>;
global using BoundaryElementGeometry3D = Model.Model.Elements.ElementGeometry<Telma.Vector3D, Telma.Vector2D>;

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

    public abstract ICoordinateTransform<TSpace, TBoundary> MasterElementCoordinateSystem { get; }
}
