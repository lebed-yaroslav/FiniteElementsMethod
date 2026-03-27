using System.Diagnostics;
using Model.Core.Matrix;
using Quasar.Native;

namespace Model.Core.Solver;


[Obsolete("This feature is incomplete and should not be used yet.")]
public sealed class PardisoSolver : ISolver
{
    private Pardiso<double>? _solver;
    public IGlobalMatrix? Matrix
    {
        set
        {
            if (value is not IPardisoMatrix<double> pardisoMatrix)
                throw new ArgumentException($"Expected matrix of type {nameof(IPardisoMatrix<>)}", nameof(value));
            _solver = new(pardisoMatrix);
            _solver.Analysis();
            _solver.Factorization();
            value = field;
        }
        private get;
    }

    public (double residual, int iterations) Solve(
        ReadOnlySpan<double> rhsVector,
        Span<double> solution,
        ISolver.SolverParams paramz = default
    )
    {
        if (Matrix == null)
            throw new InvalidOperationException("Cannot solve system: matrix has not been properly initialized");

        Debug.Assert(rhsVector.Length == Matrix.Size);
        Debug.Assert(solution.Length == Matrix.Size);

        _solver!.Solve(rhsVector, solution);
        return (-1, -1); // TODO
    }

    public void Dispose() => _solver?.Dispose();
}
