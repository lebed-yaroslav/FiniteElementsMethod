using Model.Core.Matrix;

namespace Model.Core.Solver;

public interface IFactorization : IMatrix 
{
    void LInvMul(ReadOnlySpan<double> vec, Span<double> res);
    void UInvMul(ReadOnlySpan<double> vec, Span<double> res);
}

public interface IFactorizationPreconditioner
{
    IFactorization Factorize(CsrMatrix sparseMatrix, double eps);
}
