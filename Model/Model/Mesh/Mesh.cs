using Model.Core.CoordinateSystem;
using Model.Model.Elements;
using Telma.Extensions;

namespace Model.Model.Mesh;


public interface IMesh<TSpace>
    where TSpace : IVectorBase<TSpace>
{
    TSpace this[int i] { get; }
    int VertexCount { get; }
    ICoordinateTransform<TSpace, TSpace> CoordinateSystem { get; }
    IEnumerable<IFiniteElement<TSpace>> FiniteElements { get; }
}


// FIXME: This is dirty workaround because TBoundary cannot be deduced from TSpace
public interface IMeshWithBoundaries<TSpace, TBoundary> : IMesh<TSpace>
    where TSpace : IVectorBase<TSpace>
    where TBoundary : IVectorBase<TBoundary>
{
    IEnumerable<IBoundaryElement<TSpace, TBoundary>> BoundaryElements { get; }
}


public sealed class Mesh<TSpace, TBoundary>(ICoordinateTransform<TSpace, TSpace> coordinateSystem) :
    IMesh<TSpace>, IMeshWithBoundaries<TSpace, TBoundary>
    where TSpace : IVectorBase<TSpace>
    where TBoundary : IVectorBase<TBoundary>
{
    private readonly List<TSpace> _vertices = [];
    private readonly List<IFiniteElement<TSpace>> _finiteElements;
    private readonly List<IBoundaryElement<TSpace, TBoundary>> _boundaryElements;

    public TSpace this[int i] => _vertices[i];
    public int VertexCount => _vertices.Count;

    public IEnumerable<IFiniteElement<TSpace>> FiniteElements => _finiteElements;
    public IEnumerable<IBoundaryElement<TSpace, TBoundary>> BoundaryElements => _boundaryElements;

    public ICoordinateTransform<TSpace, TSpace> CoordinateSystem { get; } = coordinateSystem;

    // Mesh construction:

    public void AddVertex(TSpace vertex) => _vertices.Add(vertex);

    public void AddElement(IFiniteElementFactory<TSpace> factory, int[] vertices, int materialIndex) =>
         _finiteElements.Add(factory.CreateElement(this, vertices, materialIndex));

    public void AddBoundary(IBoundaryElementFactory<TSpace, TBoundary> factory, int[] vertices, int boundaryIndex) =>
         _boundaryElements.Add(factory.CreateBoundary(this, vertices, boundaryIndex));
}
