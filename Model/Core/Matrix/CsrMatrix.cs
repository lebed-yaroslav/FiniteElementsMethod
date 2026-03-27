using System.Diagnostics;

namespace Model.Core.Matrix;


public class CsrMatrix(CsrMatrix.Portrait portrait) : IGlobalMatrix
{
    public sealed record Portrait(int[] Ig, int[] Jg)
    {
        public int Size => Ig.Length - 1;
        public int TriangleElementCount => Ig[^1];

        public static Portrait Create(IList<HashSet<int>> adjacencyList)
        {
            int n = adjacencyList.Count;

            // 1. Initialize ig (row indices)
            int sum = 0;
            int[] ig = new int[n + 1];
            for (int i = 0; i < n; i++)
            {
                ig[i] = sum;
                sum += adjacencyList[i].Count;
            }
            ig[n] = sum;

            // 2. Initialize jg (col indices)
            int[] jg = new int[sum];
            int addr = 0;
            for (int i = 0; i < n; i++)
                foreach (var k in adjacencyList[i].OrderBy(j => j))
                    jg[addr++] = k;

            return new(ig, jg);
        }
    }

    private readonly Portrait _portrait = portrait;
    private readonly double[] _di = new double[portrait.Size];
    private readonly double[] _ggl = new double[portrait.TriangleElementCount];

    public int Size => _portrait.Size;
    public ReadOnlySpan<int> Ig => _portrait.Ig;
    public ReadOnlySpan<int> Jg => _portrait.Jg;
    public Span<double> Di => _di;
    public Span<double> Ggl => _ggl;
    public Span<double> Ggu => _ggl; // Symmetrical

    public CsrMatrix(CsrMatrix other) : this(other._portrait)
    {
        _di = [.. other._di];
        _ggl = [.. other._ggl];
    }

    public object Clone() => new CsrMatrix(this);

    public void AddLocalMatrix(LocalMatrix matrix, ReadOnlySpan<int> indices)
    {
        int m = indices.Length;
        ArgumentOutOfRangeException.ThrowIfNotEqual(matrix.Size, m);

        for (int i = 0; i < m; i++)
        {
            int gi = indices[i];
            if (gi < 0) continue;
            _di[gi] += matrix[i, i];
        }

        for (int i = 0; i < m; i++)
        {
            int gi = indices[i];
            if (gi < 0) continue;
            for (int j = 0; j < m; j++)
            {
                int gj = indices[j];
                if (gj < 0 || gi <= gj) continue;
                int k = FindPosition(row: gi, col: gj);
                Debug.Assert(k >= 0, "Local matrix does not match the portrait");
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
        return Array.BinarySearch(_portrait.Jg, start, end - start, col);
    }
}
