namespace Model.Core.Solver;


public interface ISolver
{
    public record struct SolverParams(
        double Eps = 1e-10,
        int MaxIterations = 1000
    );

    (double residual, int iterations) Solve(ReadOnlySpan<double> rhsVector, Span<double> solution, SolverParams paramz = new());
}
