using Model.Core.CoordinateSystem;
using Model.Model.Elements;
using Telma;
using Telma.Extensions;

namespace Model.Model;


public interface IMesh<TSpace>
    where TSpace : IVectorBase<TSpace>
{
    TSpace this[int i] { get; }
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
    public int VerticesCount => _vertices.Count;
    public ICoordinateTransform<TSpace, TSpace> CoordinateSystem { get; } = coordinateSystem;
    public IEnumerable<IFiniteElement<TSpace>> FiniteElements => _finiteElements;

    public void AddVertex(TSpace vertex) => _vertices.Add(vertex);
    public void AddElement(IFiniteElementFactory<TSpace> factory, int[] vertices, int materialIndex) =>
         _finiteElements.Add(factory.CreateElement(this, vertices, materialIndex));
}


public sealed class Mesh2D(ICoordinateTransform<Vector2D, Vector2D> coordinateSystem) :
    Mesh<Vector2D>(coordinateSystem), IMeshWithBoundaries<Vector2D, Vector1D>
{
    private readonly List<IBoundaryElement<Vector2D, Vector1D>> _boundaryElements = [];
    public IEnumerable<IBoundaryElement<Vector2D, Vector1D>> BoundaryElements => _boundaryElements;

    //По-идее вместо materialIndex должен был быть boundaryIndex, как в методе CreateBoundary.
    public void AddBoundary(IBoundaryElementFactory<Vector2D, Vector1D> factory, int[] vertices, int materialIndex) =>
         _boundaryElements.Add(factory.CreateBoundary(this, vertices, materialIndex));
}


public sealed class Mesh3D(ICoordinateTransform<Vector3D, Vector3D> coordinateSystem) :
    Mesh<Vector3D>(coordinateSystem), IMeshWithBoundaries<Vector3D, Vector2D>
{
    private readonly List<IBoundaryElement<Vector3D, Vector2D>> _boundaryElements = [];
    public IEnumerable<IBoundaryElement<Vector3D, Vector2D>> BoundaryElements => _boundaryElements;
    public void AddBoundary(IBoundaryElementFactory<Vector3D, Vector2D> factory, int[] vertices, int materialIndex) =>
         _boundaryElements.Add(factory.CreateBoundary(this, vertices, materialIndex));
}


public static class MeshExtensions
{
    extension<TSpace, TBoundary>(IMeshWithBoundaries<TSpace, TBoundary> self)
        where TSpace : IVectorBase<TSpace>
        where TBoundary : IVectorBase<TBoundary>
    {
        public IEnumerable<FiniteElementBase<TSpace>> AllElements => self.FiniteElements
            .Select(e => new FiniteElementBase<TSpace>(e.Geometry, e.DOF))
            .Concat(self.BoundaryElements.Select(e => new FiniteElementBase<TSpace>(e.Geometry, e.DOF)));

        public IEnumerable<IDofManager> ElementsDof => self.FiniteElements
            .Select(e => e.DOF)
            .Concat(self.BoundaryElements.Select(e => e.DOF));
    }
}
