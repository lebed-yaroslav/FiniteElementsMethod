using Model.Core.Matrix;

namespace Model.Core.Solver.Precondition;

public sealed record ILUFactorization(CsrMatrix Storage) : IFactorization
{
    public int Size => Di.Length;

    public ReadOnlySpan<int> Ig => Storage.Ig;
    public ReadOnlySpan<int> Jg => Storage.Jg;
    public Span<double> Ggl => Storage.Ggl;
    public Span<double> Ggu => Storage.Ggu;
    public Span<double> Di => Storage.Di;

    public void LInvMul(ReadOnlySpan<double> vec, Span<double> res)
    {
        for (int i = 0; i < res.Length; ++i)
        {
            double sum = 0.0;
            for (int p = Ig[i]; p < Ig[i + 1]; ++p)
            {
                int j = Jg[p];
                sum += Ggl[p] * res[j];
            }
            res[i] = vec[i] - sum;
        }
    }

    public void UInvMul(ReadOnlySpan<double> vec, Span<double> res)
    {
        vec.CopyTo(res);
        for (int i = res.Length - 1; i >= 0; --i)
        {
            double d = Di[i];
            if (d == 0) throw new ArithmeticException($"Bad diagonal in U: Di[{i}]=0");
            res[i] /= d;
            for (int p = Ig[i]; p < Ig[i + 1]; ++p)
            {
                int j = Jg[p];
                res[j] -= res[i] * Ggu[p];
            }
        }
    }
}

public class ILUPreconditioner : IFactorizationPreconditioner
{
    public IFactorization Factorize(CsrMatrix matrix, double eps)
    {
        int n = matrix.Size;
        var lu = new ILUFactorization(Storage: matrix);

        for (int i = 0; i < n; ++i)
        {
            double sumD = 0.0;
            double d;
            for (int p = matrix.Ig[i]; p < matrix.Ig[i + 1]; ++p)
            {
                int j = matrix.Jg[p];

                double sumL = 0.0;
                double sumU = 0.0;

                int lp = matrix.Ig[i];
                int up = matrix.Ig[j];

                while (lp < p && up < matrix.Ig[j + 1])
                {
                    int lj = matrix.Jg[lp];
                    int uj = matrix.Jg[up];

                    if (lj > uj) up++;
                    else if (lj < uj) lp++;
                    else
                    {
                        sumL += matrix.Ggl[lp] * matrix.Ggu[up];
                        sumU += matrix.Ggl[up] * matrix.Ggu[lp];
                        up++; lp++;
                    }
                }

                d = lu.Di[j];
                if (!double.IsFinite(d) || Math.Abs(d) < eps)
                    throw new ArithmeticException($"Bad diagonal in ILLt: Di[{j}]={d}");

                lu.Ggl[p] = (matrix.Ggl[p] - sumL) / d;
                lu.Ggu[p] = matrix.Ggu[p] - sumU;
                sumD += lu.Ggl[p] * lu.Ggu[p];
            }
            d = matrix.Di[i] - sumD;
            if (!double.IsFinite(d) || Math.Abs(d) < eps)
                throw new ArithmeticException($"Bad diagonal after ILLt: Di[{i}]={d}");
            lu.Di[i] = d;
        }
        return lu;
    }
}
