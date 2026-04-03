using System.Diagnostics;
using Model.Core.Matrix;
using Model.Core.Solver;
using Model.Core.Solver.Precondition;
using Model.Util;

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

        double[] rhs = [6, 7];
        double[] solution = [0, 0];

        var solver = new PCGSolver(m => new IdentityPreconditioner())
        {
            Matrix = matrix
        };
        solver.Solve(rhs, solution, new ISolver.Params(1e-10, 1000));

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


        double[] rhs = [0, 0];
        double[] solution = [5, -3];

        var solver = new PCGSolver(m => new IdentityPreconditioner())
        {
            Matrix = matrix
        };
        var (residual, iterations) = solver.Solve(rhs, solution, new ISolver.Params(1e-10, 1000));

        AssertClose(0.0, solution[0]);
        AssertClose(0.0, solution[1]);
        Assert.Equal(0, iterations);
        AssertClose(0.0, residual);
    }

    [Fact]
    public void Solve_SmallResidual() //тест на малую невязку после решения
    {
        var matrix = new TestMatrix(new double[,]
        {
            { 4, 1 },
            { 1, 3 }
        });


        double[] rhs = [6, 7];
        double[] solution = [0, 0];

        var solver = new PCGSolver(_ => new IdentityPreconditioner())
        {
            Matrix = matrix
        };
        var (residual, iterations) = solver.Solve(rhs, solution, new ISolver.Params(1e-10, 1000));

        Assert.True(residual < 1e-6,
            $"Residual is too large: {residual}");
    }

    [Fact]
    public void Solve_GoodInitialGuess() //тест с хорошим начальным приближением
    {
        var matrix = new TestMatrix(new double[,]
        {
            { 4, 1 },
            { 1, 3 }
        });


        double[] rhs = [6, 7];
        double[] solution = [0.9, 2.1];

        var solver = new PCGSolver(_ => new IdentityPreconditioner())
        {
            Matrix = matrix
        };
        solver.Solve(rhs, solution, new ISolver.Params(1e-10, 1000));

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


        double[] rhs = [6, 7];
        double[] solution = [0, 0];

        var solver = new PCGSolver(_ => new IdentityPreconditioner())
        {
            Matrix = matrix
        };

        var (residual, iterations) = solver.Solve(rhs, solution, new ISolver.Params(1e-30, 1));

        Assert.True(iterations <= 1,
            $"Iterations: {iterations}");
    }

    private class IdentityPreconditioner : IPreconditioner
    {
        public void Apply(ReadOnlySpan<double> input, Span<double> output)
        {
            for (int i = 0; i < input.Length; i++)
                output[i] = input[i];
        }
    }

    private class TestMatrix(double[,] data) : IGlobalMatrix
    {
        private readonly double[,] _data = data;

        public int Size => _data.GetLength(0);

        public void AddLocalMatrix(LocalMatrix matrix, ReadOnlySpan<int> indices) => throw new NotImplementedException();

        public void MulVec(ReadOnlySpan<double> vector, Span<double> result)
        {
            Debug.Assert(vector.Length == Size);
            Debug.Assert(result.Length == Size);
            for (int i = 0; i < Size; i++)
            {
                double sum = 0;
                for (int j = 0; j < Size; j++)
                    sum += _data[i, j] * vector[j];

                result[i] = sum;
            }
        }

        public object Clone() => new TestMatrix((double[,])_data.Clone());
        public void Fill(double value) => _data.AsFlatSpan<double>().Fill(value);
        public void Scale(double factor) {
            var flatten = _data.AsFlatSpan<double>();
            for (int i = 0; i < flatten.Length; ++i)
                flatten[i] *= factor;
        }
    }
}
