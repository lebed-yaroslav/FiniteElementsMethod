using Model.Core.CoordinateSystem;
using Model.Model.Basis;
using Model.Model.Elements.Triangle;
using Telma;
using Telma.Extensions;

namespace Model.Model;


public interface IFiniteElement<TSpace> :
    IElementGeometry<TSpace>,
    IDofManager,
    IBasisSet<TSpace>
    where TSpace : IVectorBase<TSpace>
{
    int MaterialIndex { get; }
}


public interface IBoundaryElement<TSpace> :
    IElementGeometry<TSpace>,
    IDofManager,
    IBasisSet<TSpace>
    where TSpace: IVectorBase<TSpace>
{
    int BoundaryIndex { get; }
}


public interface IElementGeometry<TSpace>
    where TSpace : IVectorBase<TSpace>
{
    IMesh<TSpace> Mesh { get; }

    ReadOnlySpan<int> Vertices { get; }
    IEnumerable<Edge> Edges { get; }
    int EdgeCount { get; }
}

public interface IVolumeElementGeometry<TSpace> : IElementGeometry<TSpace>
    where TSpace : IVectorBase<TSpace>
{
    ICoordinateTransform<TSpace, TSpace> MasterElementCoordinateSystem { get; }
}

public interface IBoundaryElementGeometry<TSpace, TBoundary> : IElementGeometry<TSpace>
    where TSpace : IVectorBase<TSpace>
    where TBoundary : IVectorBase<TBoundary>
{
    ICoordinateTransform<TSpace, TBoundary> MasterElementCoordinateSystem { get; }
}

public interface IDofManager
{
    ReadOnlySpan<int> Dof { get; }
    int NumberOfDofOnVertex { get; }
    int NumberOfDofOnEdge { get; }
    int NumberOfDofOnElement { get; }
    public void Renumber(Func<int, int> renumberFunction);
    public void SetVertexDof(int localVertexIndex, int n, int dofIndex);
    public void SetEdgeDof(int localEdgeIndex, bool isOrientationFlipped, int n, int dofIndex);
    public void SetElementDof(int n, int dofIndex);
}


public interface IBasisSet<TSpace> where TSpace : IVectorBase<TSpace>
{
    IEnumerable<Quadratures.Node<TSpace>> Quadratures { get; }
    ReadOnlySpan<IBasisFunction<TSpace>> Basis { get; }
}

public readonly record struct Edge(int I, int J);

public interface IBasisSet2D : IBasisSet<Vector2D>;

public interface IFiniteElementFactory<TSpace>
    where TSpace: IVectorBase<TSpace>
{
    IFiniteElement<TSpace> CreateElement(IMesh<TSpace> mesh, int[] vertices, int materialIndex);
    IBoundaryElement<TSpace> CreateBoundary(IMesh<TSpace> mesh, int[] vertices, int boundaryIndex);
}


public static class FiniteElements
{
    public static readonly IFiniteElementFactory<Vector2D> LinearTriangle = new LinearTriangleFactory();
    public static readonly IFiniteElementFactory<Vector2D> HierarchicalQuadraticTriangle = new HierarchicalQuadraticTriangleFactory();
}
