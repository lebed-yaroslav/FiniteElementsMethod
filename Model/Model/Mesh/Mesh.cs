global using IMesh1D = Model.Model.Mesh.IMesh<Telma.Vector1D>;
global using IMesh2D = Model.Model.Mesh.IMesh<Telma.Vector2D>;
global using IMesh3D = Model.Model.Mesh.IMesh<Telma.Vector3D>;

using Model.Core.CoordinateSystem;
using Model.Model.Elements;
using Telma;
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


public abstract class Mesh<TSpace>(ICoordinateTransform<TSpace, TSpace> coordinateSystem) :
    IMesh<TSpace>
    where TSpace : IVectorBase<TSpace>
{
    private readonly List<TSpace> _vertices = [];
    private readonly List<IFiniteElement<TSpace>> _finiteElements = [];

    public TSpace this[int i] => _vertices[i];
    public int VertexCount => _vertices.Count;
    public ICoordinateTransform<TSpace, TSpace> CoordinateSystem { get; } = coordinateSystem;
    public IEnumerable<IFiniteElement<TSpace>> FiniteElements => _finiteElements;

    public void AddVertex(TSpace vertex) => _vertices.Add(vertex);
    public void AddElement(IFiniteElementFactory<TSpace> factory, int[] vertices, int materialIndex) =>
         _finiteElements.Add(factory.CreateElement(this, vertices, materialIndex));
}


public sealed class Mesh2D(ICoordinateTransform<Vector2D, Vector2D> coordinateSystem) :
    Mesh<Vector2D>(coordinateSystem), IMeshWithBoundaries<Vector2D, Vector1D>
{
    private readonly List<IBoundaryElement2D> _boundaryElements = [];
    public IEnumerable<IBoundaryElement2D> BoundaryElements => _boundaryElements;

    public void AddBoundary(IBoundaryElementFactory2D factory, int[] vertices, int boundaryIndex) =>
         _boundaryElements.Add(factory.CreateBoundary(this, vertices, boundaryIndex));
}


public sealed class Mesh3D(ICoordinateTransform<Vector3D, Vector3D> coordinateSystem) :
    Mesh<Vector3D>(coordinateSystem), IMeshWithBoundaries<Vector3D, Vector2D>
{
    private readonly List<IBoundaryElement3D> _boundaryElements = [];
    public IEnumerable<IBoundaryElement3D> BoundaryElements => _boundaryElements;
    public void AddBoundary(IBoundaryElementFactory3D factory, int[] vertices, int boundaryIndex) =>
         _boundaryElements.Add(factory.CreateBoundary(this, vertices, boundaryIndex));
}
