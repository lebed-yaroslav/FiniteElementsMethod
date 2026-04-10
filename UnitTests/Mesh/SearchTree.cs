using Model.Model.Elements;
using Model.Model.Mesh;
using Telma;
using SearchTree2D = Model.Model.Mesh.SearchTree<Telma.Vector2D>;

namespace UnitTests.Mesh;

public class SearchTreeTests
{
    [Fact]
    public void FindElementAt_Vertices()
    {
        var mesh = new Mesh2D();
        mesh.AddVertices(new(0, 5), new(3, -2), new(4, 6));
        var element = mesh.AddElement(FiniteElements.Triangle.Linear, [0, 1, 2], 0);
        var searchTree = SearchTree2D.BuildFor(mesh);

        Assert.Equal(element, searchTree.FindElementAt(new(0, 5)));
        Assert.Equal(element, searchTree.FindElementAt(new(3, -2)));
        Assert.Equal(element, searchTree.FindElementAt(new(4, 6)));
        Assert.Equal(element, searchTree.FindElementAt(new(2, 3)));
        Assert.Null(searchTree.FindElementAt(new(0, 0)));
    }

    [Fact]
    public void FindElementAt_EvenQuadrangles()
    {
        var mesh = new Mesh2D();
        Vector2D[] vertices = [
            new(0, 0), new(1, 0), new(1, 1), new(0, 1),
            new(2, 0), new(2, 1), new(2, 2), new(1, 2),
            new(0, 2)
        ];
        mesh.AddVertices(vertices);

        var q00 = mesh.AddElement(FiniteElements.Quadrangle.Bilinear, [0, 1, 2, 3], 0);
        var q10 = mesh.AddElement(FiniteElements.Quadrangle.Bilinear, [1, 4, 5, 2], 0);
        var q11 = mesh.AddElement(FiniteElements.Quadrangle.Bilinear, [2, 5, 6, 7], 0);
        var q01 = mesh.AddElement(FiniteElements.Quadrangle.Bilinear, [3, 2, 7, 8], 0);

        var searchTree = SearchTree2D.BuildFor(mesh);

        Assert.Equal(q00, searchTree.FindElementAt(new(0.5, 0.5)));
        Assert.Equal(q10, searchTree.FindElementAt(new(1.5, 0.5)));
        Assert.Equal(q11, searchTree.FindElementAt(new(1.5, 1.5)));
        Assert.Equal(q01, searchTree.FindElementAt(new(0.5, 1.5)));

        Assert.All(vertices, vertex =>
            Assert.NotNull(searchTree.FindElementAt(vertex))
        );
    }
}
