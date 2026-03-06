using Model.Core.CoordinateSystem;
using Model.Model.Basis;
using Telma;

namespace Model.Model.Elements;


public sealed record class FiniteElement(
    IFiniteElementGeometry Geometry,
    IDofManager DOF,
    IBasisSet BasisSet,
    int MaterialIndex
) : IFiniteElement
{
    public IMesh2D Mesh => Geometry.Mesh;
    public ReadOnlySpan<int> Vertices => Geometry.Vertices;
    public IEnumerable<Edge> Edges => Geometry.Edges;
    public int EdgeCount => Geometry.EdgeCount;
    public ICoordinateSystem2D MasterElementCoordinateSystem => Geometry.MasterElementCoordinateSystem;

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

    public ReadOnlySpan<IBasisFunction> Basis => BasisSet.Basis;
    public IEnumerable<Quadratures.QuadratureNode2D> Quadratures => BasisSet.Quadratures;
}

public abstract class DofManager(
    int dofCount
) : IDofManager
{
    protected int[] _dof = new int[dofCount];
    public ReadOnlySpan<int> Dof => _dof;

    public abstract int NumberOfDofOnVertex { get; }
    public abstract int NumberOfDofOnEdge { get; }
    public abstract int NumberOfDofOnElement { get; }

    public void Renumber(Func<int, int> renumberFunction)
    {
        for (int i = 0; i < _dof.Length; i++)
            _dof[i] = renumberFunction(_dof[i]);
    }

    public abstract void SetVertexDof(int localVertexIndex, int n, int dofIndex);
    public abstract void SetEdgeDof(int localEdgeIndex, bool isOrientationFlipped, int n, int dofIndex);
    public abstract void SetElementDof(int n, int dofIndex);
}

public abstract class FiniteElementGeometry(int[] vertexIndices) : IFiniteElementGeometry
{
    public required IMesh2D Mesh { get; init; }

    protected int[] _vertexIndices = vertexIndices;
    public ReadOnlySpan<int> Vertices => _vertexIndices;
    public abstract IEnumerable<Edge> Edges { get; }
    public abstract int EdgeCount { get; }
    public abstract ICoordinateSystem2D MasterElementCoordinateSystem { get; }
}

public readonly struct BasisSet(
    Func<IEnumerable<Quadratures.QuadratureNode2D>> quadratures,
    params IBasisFunction[] basis
) : IBasisSet
{
    public IEnumerable<Quadratures.QuadratureNode2D> Quadratures => quadratures();
    private readonly IBasisFunction[] _basis = basis;
    public ReadOnlySpan<IBasisFunction> Basis => _basis;
}
