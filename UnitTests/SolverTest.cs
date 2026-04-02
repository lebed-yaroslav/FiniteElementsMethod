using Model.Core.Matrix;
using Model.Core.Solver;
using Model.Core.Solver.Precondition;
using Xunit;

namespace UnitTests.Core.Solver;

public class PCGSolverTests
{
    private const double Eps = 1e-8;

    private static void AssertClose(double expected, double actual)
    {
        Assert.True(Math.Abs(expected - actual) < Eps,
            $"Expected: {expected}, Actual: {actual}");
    }

    [Fact]
    public void Solve_SimpleSystem() //тест на решение простой системы
    {
        var matrix = new TestMatrix(new double[,]
        {
            { 4, 1 },
            { 1, 3 }
        });

        var preconditioner = new IdentityPreconditioner();
        var solver = new PCGSolver(matrix, preconditioner);

        double[] rhs = [6, 7];
        double[] solution = [0, 0];

        solver.Solve(rhs, solution, new ISolver.SolverParams(1e-10, 1000));

        AssertClose(1.0, solution[0]);
        AssertClose(2.0, solution[1]);
    }

    [Fact]
    public void Solve_ZeroRhs() //тест на нулевую правую часть
    {
        var matrix = new TestMatrix(new double[,]
        {
            { 4, 1 },
            { 1, 3 }
        });

        var preconditioner = new IdentityPreconditioner();
        var solver = new PCGSolver(matrix, preconditioner);

        double[] rhs = [0, 0];
        double[] solution = [5, -3];

        var result = solver.Solve(rhs, solution, new ISolver.SolverParams(1e-10, 1000));

        AssertClose(0.0, solution[0]);
        AssertClose(0.0, solution[1]);
        Assert.Equal(0, result.iterations);
        AssertClose(0.0, result.residual);
    }

    [Fact]
    public void Solve_SmallResidual() //тест на малую невязку после решения
    {
        var matrix = new TestMatrix(new double[,]
        {
            { 4, 1 },
            { 1, 3 }
        });

        var preconditioner = new IdentityPreconditioner();
        var solver = new PCGSolver(matrix, preconditioner);

        double[] rhs = [6, 7];
        double[] solution = [0, 0];

        var result = solver.Solve(rhs, solution, new ISolver.SolverParams(1e-10, 1000));

        Assert.True(result.residual < 1e-6,
            $"Residual is too large: {result.residual}");
    }

    [Fact]
    public void Solve_GoodInitialGuess() //тест с хорошим начальным приближением
    {
        var matrix = new TestMatrix(new double[,]
        {
            { 4, 1 },
            { 1, 3 }
        });

        var preconditioner = new IdentityPreconditioner();
        var solver = new PCGSolver(matrix, preconditioner);

        double[] rhs = [6, 7];
        double[] solution = [0.9, 2.1];

        solver.Solve(rhs, solution, new ISolver.SolverParams(1e-10, 1000));

        AssertClose(1.0, solution[0]);
        AssertClose(2.0, solution[1]);
    }

    [Fact]
    public void Solve_RespectsMaxIterations() //тест на ограничение числа итераций
    {
        var matrix = new TestMatrix(new double[,]
        {
            { 4, 1 },
            { 1, 3 }
        });

        var preconditioner = new IdentityPreconditioner();
        var solver = new PCGSolver(matrix, preconditioner);

        double[] rhs = [6, 7];
        double[] solution = [0, 0];

        var result = solver.Solve(rhs, solution, new ISolver.SolverParams(1e-30, 1));

        Assert.True(result.iterations <= 1,
            $"Iterations: {result.iterations}");
    }

    private class IdentityPreconditioner : IPreconditioner
    {
        public void Apply(ReadOnlySpan<double> input, Span<double> output)
        {
            for (int i = 0; i < input.Length; i++)
                output[i] = input[i];
        }
    }

    private class TestMatrix : IGlobalMatrix
    {
        private readonly double[,] _data;

        public TestMatrix(double[,] data)
        {
            _data = data;
        }

        public int Size => _data.GetLength(0);

        public void AddLocalMatrix(LocalMatrix matrix, ReadOnlySpan<int> indices) => throw new NotImplementedException();

        public void MulVec(ReadOnlySpan<double> vector, Span<double> result)
        {
            for (int i = 0; i < Size; i++)
            {
                double sum = 0;
                for (int j = 0; j < Size; j++)
                    sum += _data[i, j] * vector[j];

                result[i] = sum;
            }
        }
    }
}
