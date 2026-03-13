namespace Model.Model.Elements;


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
