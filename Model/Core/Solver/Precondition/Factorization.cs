using Model.Core.Matrix;

namespace Model.Core.Solver.Precondition;

public interface ILUFactorization : IMatrix, IPreconditioner
{
    void LInvMul(ReadOnlySpan<double> vec, Span<double> res);
    void UInvMul(ReadOnlySpan<double> vec, Span<double> res);

    void IPreconditioner.Apply(ReadOnlySpan<double> vec, Span<double> res)
    {
        LInvMul(vec, res);
        UInvMul(res, res);
    }
}
