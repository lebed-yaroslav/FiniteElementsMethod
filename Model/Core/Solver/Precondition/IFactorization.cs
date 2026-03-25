using Model.Core.Matrix;

namespace Model.Core.Solver.Precondition;

public interface ILUFactorization : IMatrix 
{
    void LInvMul(ReadOnlySpan<double> vec, Span<double> res);
    void UInvMul(ReadOnlySpan<double> vec, Span<double> res);
}

public sealed record LUPreconditioner(ILUFactorization Factorization) : IPreconditioner
{
    public void Apply(ReadOnlySpan<double> vec, Span<double> res)
    {
        Factorization.LInvMul(vec, res);
        Factorization.UInvMul(res, res);
    }
}

public interface ILUFactorizer
{
    ILUFactorization Factorize(CsrMatrix sparseMatrix, double eps);
}
