namespace Model.Core.Matrix;


public class CsrMatrix(CsrMatrix.Portrait portrait) : IGlobalMatrix
{
    public sealed class Portrait
    {
        public required int[] ig;
        public required int[] jg;

        public int Size => ig.Length;
        public int TriangleElementCount => ig[^1];
    }
    private readonly Portrait _portrait = portrait;
    private double[] _di = new double[portrait.Size];
    private double[] _ggl = new double[portrait.TriangleElementCount];

    public int Size => _portrait.Size;
    public ReadOnlySpan<int> Ig => _portrait.ig;
    public ReadOnlySpan<int> Jg => _portrait.jg;
    public ReadOnlySpan<double> Di => _di;
    public ReadOnlySpan<double> Ggl => _ggl;
    public ReadOnlySpan<double> Ggu => _ggl; // Symmetrical

    public void AddLocalMatrix(LocalMatrix matrix, ReadOnlySpan<int> indices)
    {
        int m = indices.Length;
        ArgumentOutOfRangeException.ThrowIfNotEqual(matrix.Size, m);

        for (int i = 0; i < m; i++)
        {
            int gi = indices[i];
            _di[gi] += matrix[i, i];
        }

        for (int i = 0; i < m; i++)
        {
            int gi = indices[i];
            for (int j = 0; j < m; j++)
            {
                int gj = indices[j];
                if (gi <= gj) continue;
                int k = FindPosition(row: gi, col: gj);
                _ggl[k] += matrix[i, j];
            }
        }
    }

    public void MulVec(ReadOnlySpan<double> vec, Span<double> res)
    {
        if (vec.Length != Size || res.Length != Size)
            throw new ArgumentException("The vec/res length must be equal of the matrix Size");
        for (int i = 0; i < Size; ++i)
        {
            res[i] = Di[i] * vec[i];
            for (int p = Ig[i]; p < Ig[i + 1]; ++p)
            {
                int j = Jg[p];
                res[i] += Ggl[p] * vec[j];
                res[j] += Ggu[p] * vec[i];
            }
        }
    }

    private int FindPosition(int row, int col)
    {
        (int start, int end) = (Ig[row], Ig[row + 1]);
        return Array.BinarySearch(_portrait.jg, start, end - start, col);
    }
}
