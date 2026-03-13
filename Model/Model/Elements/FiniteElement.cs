using Model.Model.Basis;
using Telma;
using Telma.Extensions;

namespace Model.Model.Elements;


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
    where TSpace : IVectorBase<TSpace>
{
    int BoundaryIndex { get; }
}


public interface IFiniteElementFactory<TSpace>
    where TSpace : IVectorBase<TSpace>
{
    IFiniteElement<TSpace> CreateElement(IMesh<TSpace> mesh, int[] vertices, int materialIndex);
    IBoundaryElement<TSpace> CreateBoundary(IMesh<TSpace> mesh, int[] vertices, int boundaryIndex);
}


public sealed record class FiniteElement<TSpace>(
    IElementGeometry<TSpace> Geometry,
    IDofManager DOF,
    IBasisSet<TSpace> BasisSet,
    int MaterialIndex
) : IFiniteElement<TSpace>
    where TSpace : IVectorBase<TSpace>
{
    public IMesh<TSpace> Mesh => Geometry.Mesh;
    public ReadOnlySpan<int> Vertices => Geometry.Vertices;
    public IEnumerable<Edge> Edges => Geometry.Edges;
    public int EdgeCount => Geometry.EdgeCount;

    public ReadOnlySpan<int> Dof => DOF.Dof;

    public int NumberOfDofOnVertex => DOF.NumberOfDofOnVertex;
    public int NumberOfDofOnEdge => DOF.NumberOfDofOnEdge;
    public int NumberOfDofOnElement => DOF.NumberOfDofOnElement;

    public void Renumber(Func<int, int> renumberFunction)
        => DOF.Renumber(renumberFunction);

    public void SetVertexDof(int localVertexIndex, int n, int dofIndex)
        => DOF.SetVertexDof(localVertexIndex, n, dofIndex);

    public void SetEdgeDof(int localEdgeIndex, bool isOrientationFlipped, int n, int dofIndex)
        => DOF.SetEdgeDof(localEdgeIndex, isOrientationFlipped, n, dofIndex);

    public void SetElementDof(int n, int dofIndex)
        => DOF.SetElementDof(n, dofIndex);

    public ReadOnlySpan<IBasisFunction<TSpace>> Basis => BasisSet.Basis;
    public IEnumerable<Quadratures.Node<TSpace>> Quadratures => BasisSet.Quadratures;
}
