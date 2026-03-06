using Model.Core.CoordinateSystem;
using Telma;

namespace Model.Model;

public interface IMesh2D
{
    Vector2D this[int i] { get; }
    ICoordinateSystem2D CoordinateSystem { get; }
    IEnumerable<IFiniteElement> FiniteElements { get; }
}

public sealed class Mesh2D(ICoordinateSystem2D coordinateSystem) : IMesh2D
{
    private List<Vector2D> _vertices = new();
    private List<IFiniteElement> _finiteElements = new();

    public ICoordinateSystem2D CoordinateSystem { get; } = coordinateSystem;
    public Vector2D this[int i] => _vertices[i];
    public IEnumerable<IFiniteElement> FiniteElements => _finiteElements;


    public void AddVertex(Vector2D vertex) => _vertices.Add(vertex);
    public void AddElement(IFiniteElementFactory factory, int[] vertices, int materialIndex) =>
        _finiteElements.Add(factory.Create(this, vertices, materialIndex));
}
