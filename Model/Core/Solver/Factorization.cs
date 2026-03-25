using Model.Core.Matrix;

namespace Model.Core.Solver;

public class LU(CsrMatrix matrix) : IFactorizationType
{
    public int Size => Di.Length;

    public ReadOnlySpan<int> Ig => matrix.Ig;
    public ReadOnlySpan<int> Jg => matrix.Jg;
    public Span<double> Ggl => matrix.Ggl.ToArray();
    public Span<double> Di => matrix.Di.ToArray();

    public void LMult(ReadOnlySpan<double> to, Span<double> res)
    {

        to.CopyTo(res);
        for (int i = 0; i < res.Length; ++i)
        {
            double sum = 0.0;
            for (int k = Ig[i]; k < Ig[i + 1]; ++k)
            {
                sum += Ggl[k] * res[Jg[k]];
            }
            res[i] -= sum;
        }
    }

    public void UMult(ReadOnlySpan<double> to, Span<double> res)
    {

        to.CopyTo(res);
        for (int i = res.Length - 1; i >= 0; --i)
        {
            double d = Di[i];
            if (d == 0) throw new ArithmeticException($"Bad diagonal in U: Di[{i}]=0");

            res[i] /= d;
            for (int k = Ig[i]; k < Ig[i + 1]; ++k)
                res[Jg[k]] -= res[i] * Ggl[k]; // важно: Ggl, и правим res, не to
        }
    }

}

public class Factorization : IFactorizationPrediconditioner
{
    public IFactorizationType CreateFactor(CsrMatrix sparseMatrix, double eps)
    {
        var factorization = new LU(sparseMatrix);

        for (int k = 0; k < sparseMatrix.Size; ++k)
        {
            double sumD = 0.0;

            for (int i = sparseMatrix.Ig[k]; i < sparseMatrix.Ig[k + 1]; ++i)
            {
                double sumLU1 = 0.0;
                double sumLU2 = 0.0;
                for (int j = sparseMatrix.Ig[k]; j < i; j++)
                {
                    int comparison = sparseMatrix.Ig[sparseMatrix.Jg[i]];
                    while (comparison < sparseMatrix.Ig[sparseMatrix.Jg[i] + 1])
                    {
                        if (sparseMatrix.Jg[j] == sparseMatrix.Jg[comparison])
                        {
                            sumLU1 += factorization.Ggl[j] * factorization.Ggl[comparison];
                            sumLU2 += factorization.Ggl[comparison] * factorization.Ggl[j];
                        }
                        comparison++;
                    }
                }
                double d = factorization.Di[sparseMatrix.Jg[i]];
                if (!double.IsFinite(d) || Math.Abs(d) < eps)
                    throw new ArithmeticException($"Bad diagonal in ILU: Di[{sparseMatrix.Jg[i]}]={d}");

                factorization.Ggl[i] = (factorization.Ggl[i] - sumLU1) / factorization.Di[sparseMatrix.Jg[i]];
                factorization.Ggl[i] -= sumLU2;
                sumD += factorization.Ggl[i] * factorization.Ggl[i];
            }
            factorization.Di[k] -= sumD;
            if (!double.IsFinite(factorization.Di[k]) || Math.Abs(factorization.Di[k]) < eps)
                throw new ArithmeticException($"Bad diagonal after ILU: Di[{k}]={factorization.Di[k]}");
        }
        return factorization;
    }
}
