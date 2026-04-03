using Model.Core.Matrix;
using Model.Core.Solver;
using Model.Core.Solver.Precondition;

namespace UnitTests.Core.Solver;

public class PCGSolverTests
{
    private const double Eps = 1e-8;

    [Fact]
    public void Solve_SimpleSystem() //тест на решение простой системы
    {
        var matrix = new DenseMatrix(new double[,]
        {
            { 4, 1 },
            { 1, 3 }
        });

        double[] rhs = [6, 7];
        double[] solution = [0, 0];

        var solver = new PCGSolver(_ => IdentityPreconditioner.Instance)
        {
            Matrix = matrix
        };
        solver.Solve(rhs, solution, new ISolver.Params(1e-10, 1000));

        Assert.Equal(1.0, solution[0], Eps);
        Assert.Equal(2.0, solution[1], Eps);
    }

    [Fact]
    public void Solve_ZeroRhs() //тест на нулевую правую часть
    {
        var matrix = new DenseMatrix(new double[,]
        {
            { 4, 1 },
            { 1, 3 }
        });


        double[] rhs = [0, 0];
        double[] solution = [5, -3];

        var solver = new PCGSolver(_ => IdentityPreconditioner.Instance)
        {
            Matrix = matrix
        };
        var (residual, iterations) = solver.Solve(rhs, solution, new ISolver.Params(1e-10, 1000));

        Assert.Equal(0.0, solution[0], Eps);
        Assert.Equal(0.0, solution[1], Eps);
        Assert.Equal(0, iterations);
        Assert.Equal(0.0, residual, Eps);
    }

    [Fact]
    public void Solve_SmallResidual() //тест на малую невязку после решения
    {
        var matrix = new DenseMatrix(new double[,]
        {
            { 4, 1 },
            { 1, 3 }
        });


        double[] rhs = [6, 7];
        double[] solution = [0, 0];

        var solver = new PCGSolver(_ => IdentityPreconditioner.Instance)
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
        var matrix = new DenseMatrix(new double[,]
        {
            { 4, 1 },
            { 1, 3 }
        });


        double[] rhs = [6, 7];
        double[] solution = [0.9, 2.1];

        var solver = new PCGSolver(_ => IdentityPreconditioner.Instance)
        {
            Matrix = matrix
        };
        solver.Solve(rhs, solution, new ISolver.Params(1e-10, 1000));

        Assert.Equal(1.0, solution[0], Eps);
        Assert.Equal(2.0, solution[1], Eps);
    }

    [Fact]
    public void Solve_RespectsMaxIterations() //тест на ограничение числа итераций
    {
        var matrix = new DenseMatrix(new double[,]
        {
            { 4, 1 },
            { 1, 3 }
        });


        double[] rhs = [6, 7];
        double[] solution = [0, 0];

        var solver = new PCGSolver(_ => IdentityPreconditioner.Instance)
        {
            Matrix = matrix
        };

        var (residual, iterations) = solver.Solve(rhs, solution, new ISolver.Params(1e-30, 1));

        Assert.True(iterations <= 1,
            $"Iterations: {iterations}");
    }
}
