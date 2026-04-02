using System;
using System.Collections.Generic;
using System.Text;
using Model.Core.Matrix;
using Model.Core.Solver;
using Model.Core.Solver.Precondition;

namespace UnitTests.Core.PCGTest;

public class PCGSolverTest
{
    
    private class IdentityPreconditioner : IPreconditioner
    {
        public void Apply(ReadOnlySpan<double> rhs, Span<double> res)
        {
            rhs.CopyTo(res); 
        }
    }
    private class TestMatrix2x2 : IGlobalMatrix
    {
        public int Size => 2;

        public void AddLocalMatrix(LocalMatrix matrix, ReadOnlySpan<int> indices) => throw new NotImplementedException();
        public object Clone() => new TestMatrix2x2();
        public void Fill(double value) => throw new NotImplementedException();

        public void MulVec(ReadOnlySpan<double> vec, Span<double> res)
        {
            res[0] = 4.0 * vec[0] + 1.0 * vec[1];
            res[1] = 1.0 * vec[0] + 3.0 * vec[1];
        }

        public void Scale(double factor) => throw new NotImplementedException();

    }

    [Fact]
    public void PCGSolverWorksCorrectly()
    {
       
        var matrix = new TestMatrix2x2();

        
        var solver = new PCGSolver(_ => new IdentityPreconditioner())
        {
            Matrix = matrix
        };
        double[] trueSolution = [1.5, -2.0];
        double[] rhsVector = new double[2];
        matrix.MulVec(trueSolution, rhsVector);
        double[] calculatedSolution = new double[2];

        var solverParams = new ISolver.Params
        {
            Eps = 1e-10,
            MaxIterations = 100
        };

        var (residual, iterations) = solver.Solve(rhsVector, calculatedSolution, solverParams);
        
        Assert.True(residual < solverParams.Eps, $"Невязка слишком велика: {residual}");

        Assert.Equal(trueSolution[0], calculatedSolution[0], 1e-2);
        Assert.Equal(trueSolution[1], calculatedSolution[1], 1e-2);
    }
}


