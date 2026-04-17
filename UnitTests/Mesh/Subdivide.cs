using System.Collections.Immutable;
using Model.Fem.Elements;
using Model.Fem.Mesh;

namespace UnitTests.Mesh;

public class SubdivideTests
{
    [Fact]
    public void Subdivide_SingleTriangle_IsCorrect()
    {
        var mesh = new Mesh2D();
        mesh.AddVertices(new(0, 0), new(2, 2), new(1, 4));
        mesh.AddElement(FiniteElements.Triangle.Linear, [0, 1, 2], 0);
        mesh.AddBoundary(FiniteElements.Segment.Linear, [0, 1], 0);
        mesh.AddBoundary(FiniteElements.Segment.Linear, [1, 2], 0);
        mesh.AddBoundary(FiniteElements.Segment.Linear, [2, 0], 0);

        var subdividedMesh = mesh.Subdivide(
            FiniteElements.Triangle.Linear,
            FiniteElements.Segment.Linear
        );

        Assert.Equal(6, subdividedMesh.VertexCount);
        Assert.Equal(4, subdividedMesh.FiniteElements.Count());
        Assert.Equal(6, subdividedMesh.BoundaryElements.Count());

        var vertices = Enumerable.Range(0, subdividedMesh.VertexCount)
            .Select(i => subdividedMesh[i])
            .ToList();

        Assert.Contains(new(1, 1), vertices);
        Assert.Contains(new(1.5, 3), vertices);
        Assert.Contains(new(0.5, 2), vertices);
    }

    [Fact]
    public void Subdivide_SingleQuadrangle_IsCorrect()
    {
        var mesh = new Mesh2D();
        mesh.AddVertices(new(0, 0), new(2, 2), new(1, 4), new (-2, 2));
        mesh.AddElement(FiniteElements.Quadrangle.Bilinear, [0, 1, 2, 3], 0);
        mesh.AddBoundary(FiniteElements.Segment.Linear, [0, 1], 0);
        mesh.AddBoundary(FiniteElements.Segment.Linear, [1, 2], 0);
        mesh.AddBoundary(FiniteElements.Segment.Linear, [2, 3], 0);
        mesh.AddBoundary(FiniteElements.Segment.Linear, [3, 0], 0);

        var subdividedMesh = mesh.Subdivide(
            FiniteElements.Quadrangle.Bilinear,
            FiniteElements.Segment.Linear
        );

        Assert.Equal(9, subdividedMesh.VertexCount);
        Assert.Equal(4, subdividedMesh.FiniteElements.Count());
        Assert.Equal(8, subdividedMesh.BoundaryElements.Count());

        var vertices = Enumerable.Range(0, subdividedMesh.VertexCount)
            .Select(i => subdividedMesh[i])
            .ToList();

        Assert.Contains(new(1, 1), vertices);
        Assert.Contains(new(1.5, 3), vertices);
        Assert.Contains(new(-0.5, 3), vertices);
        Assert.Contains(new(-1, 1), vertices);
        Assert.Contains(new(0.25, 2), vertices);
    }
}
