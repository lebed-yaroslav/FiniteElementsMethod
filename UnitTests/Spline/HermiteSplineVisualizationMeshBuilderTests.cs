using Model.Fem.Elements;
using Model.Fem.Splines;
using Telma;

namespace UnitTests.Spline;

public sealed class HermiteSplineVisualizationMeshBuilderTests
{
    private const double Eps = 1e-10;

    [Fact]
    /// subdivision = 1
    ///
    /// v01 ----- v11
    ///  | \     / |
    ///  |   \ /   |
    ///  |    S    |
    ///  |   / \   |
    ///  | /     \ |
    /// v00 ----- v10
    ///
    /// Вершины v00, v10, v01, v11 + S = 5
    ///
    /// Треугольники
    /// (v00, v10, S)
    /// (v10, v11, S)
    /// (v11, v01, S)
    /// (v01, v00, S)
    public void BuildMesh_при_разбиении_1_создает_5_вершин_и_4_треугольника()
    {
        var element = CreateHermiteElement(
            new Vector2D(0.0, 0.0),
            new Vector2D(1.0, 0.0),
            new Vector2D(1.0, 1.0),
            new Vector2D(0.0, 1.0));

        double[] solution = CreateSolutionFor(element);
        var builder = new HermiteSplineVisualizationMeshBuilder();

        VisualizationMesh2D mesh = builder.BuildMesh([element], solution, subdivision: 1);

        Assert.Equal(1, mesh.Subdivision);
        Assert.Equal(5, mesh.Vertices.Count);
        Assert.Equal(4, mesh.Triangles.Count);
    }

    [Fact]
    /// subdivision = 2
    ///
    /// v02 -- v12 -- v22
    ///  | \  / | \  / |
    ///  |  S3  |  S4  |
    ///  | /  \ | /  \ |
    /// v01 -- v11 -- v21
    ///  | \  / | \  / |
    ///  |  S1  |  S2  |
    ///  | /  \ | /  \ |
    /// v00 -- v10 -- v20
    public void BuildMesh_при_разбиении_2_создает_13_вершин_и_16_треугольников()
    {
        var element = CreateHermiteElement(
            new Vector2D(0.0, 0.0),
            new Vector2D(1.0, 0.0),
            new Vector2D(1.0, 1.0),
            new Vector2D(0.0, 1.0));

        double[] solution = CreateSolutionFor(element);
        var builder = new HermiteSplineVisualizationMeshBuilder();

        VisualizationMesh2D mesh = builder.BuildMesh([element], solution, subdivision: 2);

        Assert.Equal(2, mesh.Subdivision);
        Assert.Equal(13, mesh.Vertices.Count);
        Assert.Equal(16, mesh.Triangles.Count);
    }

    [Fact]
    /// subdivision = 1
    ///
    /// 
    /// v01 = 2 ----- v11 = 3
    ///    | \       / |
    ///    |   \   /   |
    ///    |     4     |  S = 4
    ///    |   /   \   |
    ///    | /       \ |
    /// v00 = 0 ----- v10 = 1
    ///
    /// 0: (0, 1, 4)
    /// 1: (1, 3, 4)
    /// 2: (3, 2, 4)
    /// 3: (2, 0, 4)
    public void BuildMesh_при_разбиении_1_создает_ожидаемые_треугольники()
    {
        var element = CreateHermiteElement(
            new Vector2D(0.0, 0.0),
            new Vector2D(1.0, 0.0),
            new Vector2D(1.0, 1.0),
            new Vector2D(0.0, 1.0));

        double[] solution = CreateSolutionFor(element);
        var builder = new HermiteSplineVisualizationMeshBuilder();

        VisualizationMesh2D mesh = builder.BuildMesh([element], solution, subdivision: 1);

        Assert.Equal(new VisualizationTriangle2D(0, 0, 0, 1, 4), mesh.Triangles[0]);
        Assert.Equal(new VisualizationTriangle2D(1, 0, 1, 3, 4), mesh.Triangles[1]);
        Assert.Equal(new VisualizationTriangle2D(2, 0, 3, 2, 4), mesh.Triangles[2]);
        Assert.Equal(new VisualizationTriangle2D(3, 0, 2, 0, 4), mesh.Triangles[3]);
    }

    [Fact]
    /// (0,1) -------- (1,1)
    ///   |              |
    ///   |      S       |   S = (0.5, 0.5)
    ///   |              |
    /// (0,0) -------- (1,0)
    ///
    /// u(x, y) = x^2 + y^2
    /// в центре u(0.5, 0.5) = 0.5
    public void BuildMesh_добавляет_центр_ячейки_со_значением_сплайна()
    {
        var element = CreateHermiteElement(
            new Vector2D(0.0, 0.0),
            new Vector2D(1.0, 0.0),
            new Vector2D(1.0, 1.0),
            new Vector2D(0.0, 1.0));

        double[] solution = CreateSolutionFor(element);
        var builder = new HermiteSplineVisualizationMeshBuilder();

        VisualizationMesh2D mesh = builder.BuildMesh([element], solution, subdivision: 1);

        VisualizationVertex2D center = mesh.Vertices[4];

        AssertClose(0.5, center.X);
        AssertClose(0.5, center.Y);
        AssertClose(Function(0.5, 0.5), center.Value);
    }

    [Fact]
    // мне стало лень дедать комменты, извините. может быть потом
    public void BuildMesh_индексы_треугольников_не_выходят_за_границы_списка_вершин()
    {
        var element = CreateHermiteElement(
            new Vector2D(0.0, 0.0),
            new Vector2D(1.0, 0.0),
            new Vector2D(1.0, 1.0),
            new Vector2D(0.0, 1.0));

        double[] solution = CreateSolutionFor(element);
        var builder = new HermiteSplineVisualizationMeshBuilder();

        VisualizationMesh2D mesh = builder.BuildMesh([element], solution, subdivision: 2);

        foreach (VisualizationTriangle2D triangle in mesh.Triangles)
        {
            Assert.True(triangle.V0 >= 0 && triangle.V0 < mesh.Vertices.Count);
            Assert.True(triangle.V1 >= 0 && triangle.V1 < mesh.Vertices.Count);
            Assert.True(triangle.V2 >= 0 && triangle.V2 < mesh.Vertices.Count);
        }
    }

    [Fact]
    public void BuildMesh_сохраняет_SourceElementId()
    {
        var firstElement = CreateHermiteElement(
            new Vector2D(0.0, 0.0),
            new Vector2D(1.0, 0.0),
            new Vector2D(1.0, 1.0),
            new Vector2D(0.0, 1.0));

        var secondElement = CreateHermiteElement(
            new Vector2D(2.0, 0.0),
            new Vector2D(3.0, 0.0),
            new Vector2D(3.0, 1.0),
            new Vector2D(2.0, 1.0));

        double[] solution = CreateSolutionFor(firstElement);
        var builder = new HermiteSplineVisualizationMeshBuilder();

        VisualizationMesh2D mesh = builder.BuildMesh([firstElement, secondElement], solution, subdivision: 1);

        Assert.Equal(10, mesh.Vertices.Count);
        Assert.Equal(8, mesh.Triangles.Count);

        Assert.All(mesh.Vertices.Take(5), vertex => Assert.Equal(0, vertex.SourceElementId));
        Assert.All(mesh.Vertices.Skip(5), vertex => Assert.Equal(1, vertex.SourceElementId));

        Assert.All(mesh.Triangles.Take(4), triangle => Assert.Equal(0, triangle.SourceElementId));
        Assert.All(mesh.Triangles.Skip(4), triangle => Assert.Equal(1, triangle.SourceElementId));
    }

    [Fact]
    public void BuildMesh_выбрасывает_исключение_если_разбиение_не_положительное()
    {
        var element = CreateHermiteElement(
            new Vector2D(0.0, 0.0),
            new Vector2D(1.0, 0.0),
            new Vector2D(1.0, 1.0),
            new Vector2D(0.0, 1.0));

        double[] solution = CreateSolutionFor(element);
        var builder = new HermiteSplineVisualizationMeshBuilder();

        Assert.Throws<ArgumentOutOfRangeException>(
            () => builder.BuildMesh([element], solution, subdivision: 0));
    }

    private static void AssertClose(double expected, double actual)
    {
        Assert.True(
            Math.Abs(expected - actual) <= Eps,
            $"Expected {expected}, actual {actual}, difference {Math.Abs(expected - actual)}.");
    }

    private static IFiniteElement2D CreateHermiteElement(
        Vector2D p00,
        Vector2D p10,
        Vector2D p11,
        Vector2D p01)
    {
        var mesh = new Mesh2D();

        mesh.AddVertex(p00);
        mesh.AddVertex(p10);
        mesh.AddVertex(p11);
        mesh.AddVertex(p01);

        IFiniteElement2D element = mesh.AddElement(
            FiniteElements.Quadrangle.Hermite,
            vertices: [0, 1, 2, 3],
            materialIndex: 0);

        int globalDof = 0;

        for (int vertex = 0; vertex < 4; vertex++)
        {
            for (int localFunctional = 0; localFunctional < 4; localFunctional++)
            {
                element.DOF.SetVertexDof(vertex, localFunctional, globalDof++);
            }
        }

        return element;
    }

    private static double[] CreateSolutionFor(IFiniteElement2D element)
    {
        double[] solution = new double[16];

        for (int vertex = 0; vertex < 4; vertex++)
        {
            Vector2D point = element.Geometry.Mesh[element.Geometry.Vertices[vertex]];
            int offset = vertex * 4;

            solution[offset + 0] = Function(point.X, point.Y);
            solution[offset + 1] = DerivativeX(point.X, point.Y);
            solution[offset + 2] = DerivativeY(point.X, point.Y);
            solution[offset + 3] = MixedDerivativeXY(point.X, point.Y);
        }

        return solution;
    }

    private static double Function(double x, double y) => x * x + y * y;

    private static double DerivativeX(double x, double y) => 2.0 * x;

    private static double DerivativeY(double x, double y) => 2.0 * y;

    private static double MixedDerivativeXY(double x, double y) => 0.0;
}
