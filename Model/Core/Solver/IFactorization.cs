using Model.Core.Matrix;

namespace Model.Core.Solver;

public interface IFactorizationType : IMatrix 
{

    void LMult(ReadOnlySpan<double> to, Span<double> res);
    void UMult(ReadOnlySpan<double> to, Span<double> res);

    public ReadOnlySpan<int> Ig { get; }
    public ReadOnlySpan<int> Jg { get; }
    public Span<double> Ggl { get; }
    public Span<double> Di { get; }
}
public interface IFactorizationPrediconditioner 
{
    IFactorizationType CreateFactor(CsrMatrix sparseMatrix, double eps);
}
