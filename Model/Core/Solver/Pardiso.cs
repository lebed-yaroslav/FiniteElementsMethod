using Model.Core.Matrix;
using Quasar.Native;

namespace Model.Core.Solver;

[Obsolete("This feature is incomplete and should not be used yet.")]
public sealed class PardisoSolver : ISolver
{
    private readonly Pardiso<double> _solver;

    public PardisoSolver(IGlobalMatrix matrix)
    {
        if (matrix is not IPardisoMatrix<double> pardisoMatrix)
            throw new ArgumentException($"Expected matrix of type {nameof(IPardisoMatrix<>)}", nameof(matrix));
        _solver = new(pardisoMatrix);
        _solver.Analysis();
        _solver.Factorization();
    }

    public (double residual, int iterations) Solve(
        ReadOnlySpan<double> rhsVector,
        Span<double> solution,
        ISolver.SolverParams paramz = default
    )
    {
        _solver.Solve(rhsVector, solution);
        return (-1, -1); // TODO
    }
}
