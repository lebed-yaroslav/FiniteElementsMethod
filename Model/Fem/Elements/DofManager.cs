using System.Diagnostics;

namespace Model.Fem.Elements;


public interface IElementDof
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


public abstract class ElementDof(
    int dofCount
) : IElementDof
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

    protected void AssertIsValidVertexDofNumber(int n)
        => Debug.Assert(0 <= n && n < NumberOfDofOnVertex);

    protected void AssertIsValidEdgeDofNumber(int n)
        => Debug.Assert(0 <= n && n < NumberOfDofOnEdge);

    protected void AssertIsValidElementDofNumber(int n)
        => Debug.Assert(0 <= n && n < NumberOfDofOnElement);
}
