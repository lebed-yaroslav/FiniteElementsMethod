using System.Collections;
using System.Diagnostics;
using Model.Core.Vector;
using Quasar.Native;

namespace Model.Core.Matrix;

//[Obsolete("This feature is incomplete and should not be used yet.")]
public sealed class PardisoMatrix(PardisoMatrix.Portrait portrait) : IPardisoMatrix<double>, IGlobalMatrix
{
    public static IMatrixFactory Factory { get; } = new PardisoMatrixFactory();

    public sealed record Portrait(int[] Ia, int[] Ja)
    {
        public int Size => Ia.Length - 1;
        public int ElementCount => Ia[^1];
    }

    private readonly Portrait _portrait = portrait;
    private readonly double[] _a = new double[portrait.ElementCount];

    public PardisoMatrixType MatrixType => PardisoMatrixType.SymmetricIndefinite;
    public int Size => _portrait.Size;
    public int n => _portrait.Size;

    public ReadOnlySpan<double> a => _a;
    public ReadOnlySpan<int> ia => _portrait.Ia;
    public ReadOnlySpan<int> ja => _portrait.Ja;

    public PardisoMatrix(PardisoMatrix other) : this(other._portrait)
        => _a = [.. other._a];

    public object Clone() => new PardisoMatrix(this);

    // TODO: Implement IGlobalMatrix
    public void AddLocalMatrix(LocalMatrix matrix, ReadOnlySpan<int> indices) 
    {
        int m = indices.Length;
        ArgumentOutOfRangeException.ThrowIfNotEqual(matrix.Size, m);

        for (int i = 0; i < m; i++)
        {
            int gi = indices[i];
            if (gi < 0) continue;
            _a[gi] += matrix[i, i];
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
                _a[k] += matrix[i, j];
            }
        }
    }
    private int FindPosition(int row, int col)
    {
        if (row > col) (col, row) = (col, row);
        return ja.Slice(ia[row], ia[row + 1] - ia[row]).BinarySearch(col) + ia[row];
    }

    public void MulVec(ReadOnlySpan<double> vec, Span<double> res) => throw new NotImplementedException();
    public void Fill(double value) => Array.Fill(_a, value);
    public void Scale(double factor)
    {
        for (int i = 0; i < _a.Length; ++i)
            _a[i] *= factor;
    }
}


//[Obsolete("This feature is incomplete and should not be used yet.")]
public sealed class PardisoMatrixFactory : IMatrixFactory
{
    public IGlobalMatrix Create(IList<HashSet<int>> adjacencyList)
    {

        var NUknownws = adjacencyList.Count;
        var ig = new int[NUknownws + 1];
        ig[0] = 0;
        for (int i = 0; i < NUknownws; i++) 
        {
            ig[i + 1] = ig[i] + 1;
            foreach (var l in adjacencyList[i]) 
            {
                if (l > i) ig[i + 1]++;
            }
        }
        var jg = new int[ig[NUknownws]];
        for (int i = 0; i < NUknownws; i++) 
        {
            var jgaddr = ig[i];
            jg[jgaddr++] = i;
            foreach (var j in adjacencyList[i]) 
            {
                if (j > i) jg[jgaddr++] = j;
            }
        }

        return new PardisoMatrix(new PardisoMatrix.Portrait(ig, jg));
    }
}
