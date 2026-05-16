using System;
using System.Collections.Generic;
using System.Text;
using Model.Fem.Elements;
using Model.Fem.Splines;
using Telma;

namespace UnitTests.Spline;

public sealed class HermiteSplineEvaluator2DTests
{
    private const double Eps = 1e-10;

    [Fact]
    public void Evaluate_воспроизводит_квадратичную_функцию_на_единичном_квадрате() 
    {
        var element = CreateHermiteElement(
            new Vector2D(0.0, 0.0),
            new Vector2D(1.0, 0.0),
            new Vector2D(1.0, 1.0),
            new Vector2D(0.0, 1.0));

        double[] solution = CreateSolutionFor(element, Function, DerivativeX, DerivativeY, MixedDerivativeXY);
        var evaluator = new FiniteElementFieldEvaluator2D();

        double ξ = 0.35;
        double η = 0.70;

        double actual = evaluator.Evaluate(ξ, η, solution, element);

        double expected = Function(ξ, η);
        AssertClose(expected, actual);
    }

    [Fact]
    public void Evaluate_возвращает_физическую_точку_и_значение_сплайна()
    {
        var element = CreateHermiteElement(
            new Vector2D(2.0, 3.0),
            new Vector2D(4.0, 3.0),
            new Vector2D(4.0, 7.0),
            new Vector2D(2.0, 7.0));

        double[] solution = CreateSolutionFor(element, Function, DerivativeX, DerivativeY, MixedDerivativeXY);
        var evaluator = new FiniteElementFieldEvaluator2D();

        double ξ = 0.25;
        double η = 0.75;

        SplineSample2D actual = evaluator.EvaluateSample(ξ, η, solution, element);

        double expectedX = 2.0 + 2.0 * ξ;
        double expectedY = 3.0 + 4.0 * η;
        double expectedValue = Function(expectedX, expectedY);

        AssertClose(ξ, actual.ξ);
        AssertClose(η, actual.η);
        AssertClose(expectedX, actual.X);
        AssertClose(expectedY, actual.Y);
        AssertClose(expectedValue, actual.Value);
    }

    [Fact]
    public void Evaluate_возвращает_регулярную_локальную_сеть()
    {
        var element = CreateHermiteElement(
            new Vector2D(0.0, 0.0),
            new Vector2D(1.0, 0.0),
            new Vector2D(1.0, 1.0),
            new Vector2D(0.0, 1.0));

        double[] solution = CreateSolutionFor(element, Function, DerivativeX, DerivativeY, MixedDerivativeXY);
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

        AssertClose(1.0, samples[5].ξ);
        AssertClose(0.5, samples[5].η);

        AssertClose(0.0, samples[6].ξ);
        AssertClose(1.0, samples[6].η);

        AssertClose(0.5, samples[7].ξ);
        AssertClose(1.0, samples[7].η);

        AssertClose(1.0, samples[8].ξ);
        AssertClose(1.0, samples[8].η);
    }

    [Theory]
    [InlineData(-0.1, 0.5)]
    [InlineData(1.1, 0.5)]
    [InlineData(0.5, -0.1)]
    [InlineData(0.5, 1.1)]
    public void Evaluate_выбрасывает_исключение_когда_локальная_точка_вне_единичного_квадрата(double ξ, double η)
    {
        var element = CreateHermiteElement(
            new Vector2D(0.0, 0.0),
            new Vector2D(1.0, 0.0),
            new Vector2D(1.0, 1.0),
            new Vector2D(0.0, 1.0));

        double[] solution = new double[16];
        var evaluator = new FiniteElementFieldEvaluator2D();
        
        Assert.Throws<ArgumentOutOfRangeException>(() => evaluator.Evaluate(ξ, η, solution, element));
    }

    [Fact]
    public void Evaluate_выбрасывает_исключение_когда_вектор_решения_слишком_короткий()
    {
        var element = CreateHermiteElement(
            new Vector2D(0.0, 0.0),
            new Vector2D(1.0, 0.0),
            new Vector2D(1.0, 1.0),
            new Vector2D(0.0, 1.0));

        double[] tooShortSolution = new double[15];
        var evaluator = new FiniteElementFieldEvaluator2D();

        Assert.Throws<ArgumentOutOfRangeException>(() => evaluator.Evaluate(0.5, 0.5, tooShortSolution, element));
    }

    [Fact]
    public void Evaluate_выбрасывает_исключение_когда_разбиение_не_положительное()
    {
        var element = CreateHermiteElement(
            new Vector2D(0.0, 0.0),
            new Vector2D(1.0, 0.0),
            new Vector2D(1.0, 1.0),
            new Vector2D(0.0, 1.0));

        double[] solution = new double[16];
        var evaluator = new FiniteElementFieldEvaluator2D();

        Assert.Throws<ArgumentOutOfRangeException>(() => evaluator.EvaluateGrid(solution, element, 0));
    }


    private static void AssertClose(double expected, double actual)
    {
        Assert.True(
            Math.Abs(expected - actual) <= Eps,
            $"Expected {expected}, actual {actual}, difference {Math.Abs(expected - actual)}.");
    }

    private static IFiniteElement2D CreateHermiteElement(Vector2D p00, Vector2D p10, Vector2D p11, Vector2D p01)
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

    private static double[] CreateSolutionFor(IFiniteElement2D element,
        Func<double, double, double> u,
        Func<double, double, double> ux,
        Func<double, double, double> uy,
        Func<double, double, double> uxy)
    {
        double[] solution = new double[16];

        for (int vertex = 0; vertex < 4; vertex++)
        {
            Vector2D point = element.Geometry.Mesh[element.Geometry.Vertices[vertex]];
            int offset = vertex * 4;

            solution[offset + 0] = u(point.X, point.Y);
            solution[offset + 1] = ux(point.X, point.Y);
            solution[offset + 2] = uy(point.X, point.Y);
            solution[offset + 3] = uxy(point.X, point.Y);
        }

        return solution;
    }

    private static double Function(double x, double y) => x * x + y * y;

    private static double DerivativeX(double x, double y) => 2.0 * x;

    private static double DerivativeY(double x, double y) => 2.0 * y;

    private static double MixedDerivativeXY(double x, double y) => 0.0;
}
