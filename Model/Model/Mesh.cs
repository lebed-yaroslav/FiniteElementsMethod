
using Model.Core.CoordinateSystem;
using Telma.Extensions;

namespace Model.Model;


public interface IMesh<TSpace>
    where TSpace : IVectorBase<TSpace>
{
    TSpace this[int i] { get; }
    ICoordinateTransform<TSpace, TSpace> CoordinateSystem { get; }
    IEnumerable<IFiniteElement<TSpace>> FiniteElements { get; }
    IEnumerable<IBoundaryElement<TSpace>> BoundaryElements { get; }
}

public sealed class Mesh<TSpace>(ICoordinateTransform<TSpace, TSpace> coordinateSystem) :
    IMesh<TSpace>
    where TSpace : IVectorBase<TSpace>
{
    private readonly List<TSpace> _vertices = [];
    private readonly List<IFiniteElement<TSpace>> _finiteElements = [];
    private readonly List<IBoundaryElement<TSpace>> _boundaryElements = [];

    public TSpace this[int i] => _vertices[i];
    public ICoordinateTransform<TSpace, TSpace> CoordinateSystem { get; } = coordinateSystem;
    public IEnumerable<IFiniteElement<TSpace>> FiniteElements => _finiteElements;
    public IEnumerable<IBoundaryElement<TSpace>> BoundaryElements => _boundaryElements;

    public void AddVertex(TSpace vertex) => _vertices.Add(vertex);
    public void AddElement(IFiniteElementFactory<TSpace> factory, int[] vertices, int materialIndex) =>
         _finiteElements.Add(factory.CreateElement(this, vertices, materialIndex));
    public void AddBoundary(IFiniteElementFactory<TSpace> factory, int[] vertices, int boundaryIndex) =>
        _boundaryElements.Add(factory.CreateBoundary(this, vertices, boundaryIndex));
}
