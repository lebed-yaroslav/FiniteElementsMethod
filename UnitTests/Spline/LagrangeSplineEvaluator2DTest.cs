using Model.Fem.Elements;
using Model.Fem.Splines;
using Telma;

namespace UnitTests.Spline;

public sealed class Q_LagrangeSplineEvaluator2DTests
{
    private const double Eps = 1e-10;

    [Fact]
    public void Evaluate_на_билинейном_элементе_воспроизводит_линейную_функцию()
    {
        var element = CreateBilinearElement(
            new Vector2D(0.0, 0.0),
            new Vector2D(1.0, 0.0),
            new Vector2D(1.0, 1.0),
            new Vector2D(0.0, 1.0));

        double[] solution = CreateBilinearSolutionFor(element, LinearFunction);
        var evaluator = new FiniteElementFieldEvaluator2D();

        double ξ = 0.35;
        double η = 0.70;

        double actual = evaluator.Evaluate(ξ, η, solution, element);
        double expected = LinearFunction(ξ, η);

        AssertClose(expected, actual);
    }

    [Fact]
    public void Evaluate_на_квадратичном_лагранже_воспроизводит_квадратичную_функцию()
    {
        var element = CreateQuadraticLagrangeElement(
            new Vector2D(0.0, 0.0),
            new Vector2D(1.0, 0.0),
            new Vector2D(1.0, 1.0),
            new Vector2D(0.0, 1.0));

        double[] solution = CreateQuadraticLagrangeSolutionFor(element, QuadraticFunction);
        var evaluator = new FiniteElementFieldEvaluator2D();

        double ξ = 0.35;
        double η = 0.70;

        double actual = evaluator.Evaluate(ξ, η, solution, element);
        double expected = QuadraticFunction(ξ, η);

        AssertClose(expected, actual);
    }

    [Fact]
    public void EvaluateSample_на_квадратичном_лагранже_возвращает_физическую_точку_и_значение()
    {
        var element = CreateQuadraticLagrangeElement(
            new Vector2D(2.0, 3.0),
            new Vector2D(4.0, 3.0),
            new Vector2D(4.0, 7.0),
            new Vector2D(2.0, 7.0));

        double[] solution = CreateQuadraticLagrangeSolutionFor(element, QuadraticFunction);
        var evaluator = new FiniteElementFieldEvaluator2D();

        double ξ = 0.25;
        double η = 0.75;

        SplineSample2D actual = evaluator.EvaluateSample(ξ, η, solution, element);

        double expectedX = 2.0 + 2.0 * ξ;
        double expectedY = 3.0 + 4.0 * η;
        double expectedValue = QuadraticFunction(expectedX, expectedY);

        AssertClose(ξ, actual.ξ);
        AssertClose(η, actual.η);
        AssertClose(expectedX, actual.X);
        AssertClose(expectedY, actual.Y);
        AssertClose(expectedValue, actual.Value);
    }

    [Fact]
    public void EvaluateGrid_на_квадратичном_лагранже_строит_регулярную_сетку()
    {
        var element = CreateQuadraticLagrangeElement(
            new Vector2D(0.0, 0.0),
            new Vector2D(1.0, 0.0),
            new Vector2D(1.0, 1.0),
            new Vector2D(0.0, 1.0));

        double[] solution = CreateQuadraticLagrangeSolutionFor(element, QuadraticFunction);
        var evaluator = new FiniteElementFieldEvaluator2D();

        const int subdivision = 2;

        List<SplineSample2D> samples = evaluator.EvaluateGrid(solution, element, subdivision);

        Assert.Equal(9, samples.Count);

        AssertClose(0.0, samples[0].ξ);
        AssertClose(0.0, samples[0].η);

        AssertClose(0.5, samples[1].ξ);
        AssertClose(0.0, samples[1].η);

        AssertClose(1.0, samples[2].ξ);
        AssertClose(0.0, samples[2].η);

        AssertClose(0.0, samples[3].ξ);
        AssertClose(0.5, samples[3].η);

        AssertClose(0.5, samples[4].ξ);
        AssertClose(0.5, samples[4].η);

        AssertClose(1.0, samples[8].ξ);
        AssertClose(1.0, samples[8].η);
    }

    [Fact]
    public void VisualizationMeshBuilder_на_квадратичном_лагранже_строит_вершины_и_треугольники()
    {
        var element = CreateQuadraticLagrangeElement(
            new Vector2D(0.0, 0.0),
            new Vector2D(1.0, 0.0),
            new Vector2D(1.0, 1.0),
            new Vector2D(0.0, 1.0));

        double[] solution = CreateQuadraticLagrangeSolutionFor(element, QuadraticFunction);

        var builder = new FiniteElementVisualizationMeshBuilder2D();

        VisualizationMesh2D mesh = builder.BuildMesh([element], solution, subdivision: 2);

        Assert.Equal(2, mesh.Subdivision);
        Assert.Equal(13, mesh.Vertices.Count);
        Assert.Equal(16, mesh.Triangles.Count);

        foreach (VisualizationTriangle2D triangle in mesh.Triangles)
        {
            Assert.True(triangle.V0 >= 0 && triangle.V0 < mesh.Vertices.Count);
            Assert.True(triangle.V1 >= 0 && triangle.V1 < mesh.Vertices.Count);
            Assert.True(triangle.V2 >= 0 && triangle.V2 < mesh.Vertices.Count);
        }
    }

    private static IFiniteElement2D CreateBilinearElement(Vector2D p00, Vector2D p10, Vector2D p11, Vector2D p01)
    {
        var mesh = new Mesh2D();

        mesh.AddVertex(p00);
        mesh.AddVertex(p10);
        mesh.AddVertex(p11);
        mesh.AddVertex(p01);

        IFiniteElement2D element = mesh.AddElement(
            FiniteElements.Quadrangle.Bilinear,
            vertices: [0, 1, 2, 3],
            materialIndex: 0);

        for (int vertex = 0; vertex < 4; vertex++)
        {
            element.DOF.SetVertexDof(vertex, 0, vertex);
        }

        return element;
    }

    private static IFiniteElement2D CreateQuadraticLagrangeElement(Vector2D p00, Vector2D p10, Vector2D p11, Vector2D p01)
    {
        var mesh = new Mesh2D();

        mesh.AddVertex(p00);
        mesh.AddVertex(p10);
        mesh.AddVertex(p11);
        mesh.AddVertex(p01);

        IFiniteElement2D element = mesh.AddElement(
            FiniteElements.Quadrangle.LagrangeQuadratic,
            vertices: [0, 1, 2, 3],
            materialIndex: 0);

        // Q2 порядок базисов 
        //
        // 6 ----- 7 ----- 8
        // |       |       |
        // 3 ----- 4 ----- 5
        // |       |       |
        // 0 ----- 1 ----- 2

        element.DOF.SetVertexDof(0, 0, 0);
        element.DOF.SetVertexDof(1, 0, 2);
        element.DOF.SetVertexDof(2, 0, 8);
        element.DOF.SetVertexDof(3, 0, 6);

        element.DOF.SetEdgeDof(0, isOrientationFlipped: false, 0, 1);
        element.DOF.SetEdgeDof(1, isOrientationFlipped: false, 0, 5);
        element.DOF.SetEdgeDof(2, isOrientationFlipped: false, 0, 7);
        element.DOF.SetEdgeDof(3, isOrientationFlipped: false, 0, 3);

        element.DOF.SetElementDof(0, 4);

        return element;
    }

    private static double[] CreateBilinearSolutionFor(IFiniteElement2D element, Func<double, double, double> u)
    {
        double[] solution = new double[4];

        for (int vertex = 0; vertex < 4; vertex++)
        {
            Vector2D point = element.Geometry.Mesh[element.Geometry.Vertices[vertex]];
            solution[vertex] = u(point.X, point.Y);
        }

        return solution;
    }

    private static double[] CreateQuadraticLagrangeSolutionFor( IFiniteElement2D element,Func<double, double, double> u)
    {
        double[] solution = new double[9];

        Vector2D[] localNodes =
        [
            new Vector2D(0.0, 0.0),
            new Vector2D(0.5, 0.0),
            new Vector2D(1.0, 0.0),

            new Vector2D(0.0, 0.5),
            new Vector2D(0.5, 0.5),
            new Vector2D(1.0, 0.5),

            new Vector2D(0.0, 1.0),
            new Vector2D(0.5, 1.0),
            new Vector2D(1.0, 1.0),
        ];

        for (int i = 0; i < localNodes.Length; i++)
        {
            Vector2D physicalPoint = element.Geometry.MasterElementCoordinateSystem.InverseTransform(localNodes[i]);

            solution[i] = u(physicalPoint.X, physicalPoint.Y);
        }

        return solution;
    }

    private static void AssertClose(double expected, double actual)
    {
        Assert.True(
            Math.Abs(expected - actual) <= Eps,
            $"Expected {expected}, actual {actual}, difference {Math.Abs(expected - actual)}.");
    }

    private static double LinearFunction(double x, double y)
        => 2.0 * x - 3.0 * y + 5.0;

    private static double QuadraticFunction(double x, double y)
        => x * x + y * y;
}

public sealed class Tri_LagrangeSplineEvaluator2DTests
{
    private const double Eps = 1e-10;

    [Fact]
    public void Evaluate_на_линейном_треугольнике_воспроизводит_линейную_функцию()
    {
        var element = CreateLinearTriangleElement(
            new Vector2D(0.0, 0.0),
            new Vector2D(1.0, 0.0),
            new Vector2D(0.0, 1.0));

        double[] solution = CreateLinearTriangleSolutionFor(element, LinearFunction);
        var evaluator = new FiniteElementFieldEvaluator2D();

        double ξ = 0.25;
        double η = 0.50;

        double actual = evaluator.Evaluate(ξ, η, solution, element);
        double expected = LinearFunction(ξ, η);

        AssertClose(expected, actual);
    }

    [Fact]
    public void Evaluate_на_кубическом_лагранжевом_треугольнике_воспроизводит_кубическую_функцию()
    {
        var element = CreateLagrangeCubicTriangleElement(
            new Vector2D(0.0, 0.0),
            new Vector2D(1.0, 0.0),
            new Vector2D(0.0, 1.0));

        double[] solution = CreateCubicTriangleSolutionFor(element, CubicFunction);
        var evaluator = new FiniteElementFieldEvaluator2D();

        double ξ = 0.20;
        double η = 0.35;

        double actual = evaluator.Evaluate(ξ, η, solution, element);
        double expected = CubicFunction(ξ, η);

        AssertClose(expected, actual);
    }

    [Fact]
    public void EvaluateSample_на_кубическом_лагранжевом_треугольнике_возвращает_физическую_точку_и_значение()
    {
        var element = CreateLagrangeCubicTriangleElement(
            new Vector2D(2.0, 3.0),
            new Vector2D(5.0, 3.0),
            new Vector2D(2.0, 7.0));

        double[] solution = CreateCubicTriangleSolutionFor(element, CubicFunction);
        var evaluator = new FiniteElementFieldEvaluator2D();

        double ξ = 0.25;
        double η = 0.50;

        SplineSample2D sample = evaluator.EvaluateSample(ξ, η, solution, element);

        double expectedX = 2.0 + 3.0 * ξ;
        double expectedY = 3.0 + 4.0 * η;
        double expectedValue = CubicFunction(expectedX, expectedY);

        AssertClose(ξ, sample.ξ);
        AssertClose(η, sample.η);
        AssertClose(expectedX, sample.X);
        AssertClose(expectedY, sample.Y);
        AssertClose(expectedValue, sample.Value);
    }

    [Fact]
    public void EvaluateGrid_на_треугольнике_строит_треугольную_локальную_сетку()
    {
        var element = CreateLinearTriangleElement(
            new Vector2D(0.0, 0.0),
            new Vector2D(1.0, 0.0),
            new Vector2D(0.0, 1.0));

        double[] solution = CreateLinearTriangleSolutionFor(element, LinearFunction);
        var evaluator = new FiniteElementFieldEvaluator2D();

        const int subdivision = 2;

        List<SplineSample2D> samples = evaluator.EvaluateGrid(solution, element, subdivision);

        // subdivision = 2
        //
        // (0,1)
        //   *
        //   | \
        //   |  \
        //   *---*
        //   | \ | \
        //   |  \|  \
        //   *---*---*
        // (0,0)   (1,0)
        //
        // количество точек:
        // (subdivision + 1) * (subdivision + 2) / 2 = 3 * 4 / 2 = 6

        Assert.Equal(6, samples.Count);

        AssertClose(0.0, samples[0].ξ);
        AssertClose(0.0, samples[0].η);

        AssertClose(0.5, samples[1].ξ);
        AssertClose(0.0, samples[1].η);

        AssertClose(1.0, samples[2].ξ);
        AssertClose(0.0, samples[2].η);

        AssertClose(0.0, samples[3].ξ);
        AssertClose(0.5, samples[3].η);

        AssertClose(0.5, samples[4].ξ);
        AssertClose(0.5, samples[4].η);

        AssertClose(0.0, samples[5].ξ);
        AssertClose(1.0, samples[5].η);
    }

    private static IFiniteElement2D CreateLinearTriangleElement(Vector2D p0, Vector2D p1, Vector2D p2)
    {
        var mesh = new Mesh2D();

        mesh.AddVertex(p0);
        mesh.AddVertex(p1);
        mesh.AddVertex(p2);

        IFiniteElement2D element = mesh.AddElement(
            FiniteElements.Triangle.Linear,
            vertices: [0, 1, 2],
            materialIndex: 0);

        element.DOF.SetVertexDof(0, 0, 0);
        element.DOF.SetVertexDof(1, 0, 1);
        element.DOF.SetVertexDof(2, 0, 2);

        return element;
    }

    private static IFiniteElement2D CreateLagrangeCubicTriangleElement(Vector2D p0, Vector2D p1, Vector2D p2)
    {
        var mesh = new Mesh2D();

        mesh.AddVertex(p0);
        mesh.AddVertex(p1);
        mesh.AddVertex(p2);

        IFiniteElement2D element = mesh.AddElement(
            FiniteElements.Triangle.LagrangeCubic,
            vertices: [0, 1, 2],
            materialIndex: 0);

        // Порядок базисов у LagrangeCubicTriangle:
        //
        // 2
        // |\
        // 8 6
        // |  \
        // 7 9 5
        // |    \
        // 0-3-4-1

        element.DOF.SetVertexDof(0, 0, 0);
        element.DOF.SetVertexDof(1, 0, 1);
        element.DOF.SetVertexDof(2, 0, 2);

        element.DOF.SetEdgeDof(0, isOrientationFlipped: false, 0, 3);
        element.DOF.SetEdgeDof(0, isOrientationFlipped: false, 1, 4);

        element.DOF.SetEdgeDof(1, isOrientationFlipped: false, 0, 5);
        element.DOF.SetEdgeDof(1, isOrientationFlipped: false, 1, 6);

        element.DOF.SetEdgeDof(2, isOrientationFlipped: false, 0, 7);
        element.DOF.SetEdgeDof(2, isOrientationFlipped: false, 1, 8);

        element.DOF.SetElementDof(0, 9);

        return element;
    }

    private static double[] CreateLinearTriangleSolutionFor(IFiniteElement2D element, Func<double, double, double> u)
    {
        double[] solution = new double[3];

        for (int vertex = 0; vertex < 3; vertex++)
        {
            Vector2D point = element.Geometry.Mesh[element.Geometry.Vertices[vertex]];
            solution[vertex] = u(point.X, point.Y);
        }

        return solution;
    }

    private static double[] CreateCubicTriangleSolutionFor(IFiniteElement2D element, Func<double, double, double> u)
    {
        double[] solution = new double[10];

        Vector2D[] localNodes =
        [
            new Vector2D(0.0, 0.0),
            new Vector2D(1.0, 0.0),
            new Vector2D(0.0, 1.0),

            new Vector2D(1.0 / 3.0, 0.0),
            new Vector2D(2.0 / 3.0, 0.0),

            new Vector2D(2.0 / 3.0, 1.0 / 3.0),
            new Vector2D(1.0 / 3.0, 2.0 / 3.0),

            new Vector2D(0.0, 2.0 / 3.0),
            new Vector2D(0.0, 1.0 / 3.0),

            new Vector2D(1.0 / 3.0, 1.0 / 3.0),
        ];

        for (int i = 0; i < localNodes.Length; i++)
        {
            Vector2D physicalPoint =
                element.Geometry.MasterElementCoordinateSystem.InverseTransform(localNodes[i]);

            solution[i] = u(physicalPoint.X, physicalPoint.Y);
        }

        return solution;
    }

    private static void AssertClose(double expected, double actual)
    {
        Assert.True(
            Math.Abs(expected - actual) <= Eps,
            $"Expected {expected}, actual {actual}, difference {Math.Abs(expected - actual)}.");
    }

    private static double LinearFunction(double x, double y) => 2.0 * x - 3.0 * y + 5.0;

    private static double CubicFunction(double x, double y) => x * x * x + y * y * y + x * y + 2.0 * x - y + 3.0;
}
