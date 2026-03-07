using Model.Core.CoordinateSystem;
using Model.Model.Basis;
using Model.Model.Elements.Triangle;
using Telma;

namespace Model.Model;


public interface IFiniteElement :
    IFiniteElementGeometry,
    IDofManager,
    IBasisSet<Vector2D>
{
    int MaterialIndex { get; }
}

public interface IFiniteElementGeometry
{
    IMesh2D Mesh { get; }

    ReadOnlySpan<int> Vertices { get; }
    IEnumerable<Edge> Edges { get; }
    int EdgeCount { get; }

    ICoordinateSystem2D MasterElementCoordinateSystem { get; }
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

public interface IBasisSet<TVector> where TVector : struct
{
    IEnumerable<Quadratures.Node<TVector>> Quadratures { get; }
    ReadOnlySpan<IBasisFunction<TVector>> Basis { get; }
}

public readonly record struct Edge(int I, int J);

public interface IBasisSet2D : IBasisSet<Vector2D>;

public interface IFiniteElementFactory
{
    IFiniteElement Create(IMesh2D mesh, int[] vertices, int materialIndex);
}

public static class FiniteElements
{
    public static readonly IFiniteElementFactory LinearTriangle = new LinearTriangleFactory();
    public static readonly IFiniteElementFactory HierarchicalQuadraticTriangle = new HierarchicalQuadraticTriangleFactory();
}
