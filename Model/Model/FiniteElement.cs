using Model.Core.CoordinateSystem;
using Model.Model.Basis;
using Model.Model.Elements.Triangle;
using Telma;

namespace Model.Model;


public interface IFiniteElement :
    IFiniteElementGeometry,
    IDofManager,
    IBasisSet
{}

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

public interface IBasisSet
{
    IEnumerable<Quadratures.QuadratureNode2D> Quadratures { get; }
    ReadOnlySpan<IBasisFunction> Basis { get; }
}

public struct Edge
{
    public readonly int I { get; private init; }
    public readonly int J { get; private init; }

    public Edge(int i, int j)
    {
        I = i;
        J = j;
    }
}

public interface IFiniteElementFactory {
    IFiniteElement Create(IMesh2D mesh, int[] vertices);
}

public static class FiniteElements
{
    public static readonly IFiniteElementFactory LinearTriangle = new LinearTriangleFactory();
    public static readonly IFiniteElementFactory HierarchicalQuadraticTriangle = new HierarchicalQuadraticTriangleFactory();
}
