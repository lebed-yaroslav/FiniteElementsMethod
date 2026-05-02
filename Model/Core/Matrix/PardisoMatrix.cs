using System.Collections;
using System.Diagnostics;
using System.Diagnostics.Tracing;
using System.Text.Json.Serialization;
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
            for (int j = 0; j < m; j++)
            {
                int gj = indices[j];
                if (gj < gi || gj < 0 ) continue;
                if (gi == gj)
                {
                    _a[ia[gi]] += matrix[i, j];
                }
                else 
                {
                    int k = FindPosition(i: gi, j: gj);
                    _a[k] += matrix[i, j];
                }
                //else if (gj > gi) 
                //{
                //    int k = FindPosition(i: gj, j: gi);
                //    _a[k] += matrix[i, j];
                //}
                //else
                //{
                //    _a[ia[gi]] += matrix[i, j];
                //}
            }
        }
    }
    private int FindPosition(int i, int j)
    {
        if (i > j) (i, j) = (j, i);
        return ja.Slice(ia[i], ia[i + 1] - ia[i]).BinarySearch(j) + ia[i];
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

        var N = adjacencyList.Count;

        var ig = new int[N + 1];
        ig[0] = 0;
        for (int i = 0; i < N; i++)
        {
            ig[i + 1] = ig[i] + 1;
            foreach (var l in adjacencyList[i])
            {
                if (l > i) ig[i + 1]++;
            }
        }
        var jg = new int[ig[N]];
        for (int i = 0; i < N; i++)
        {
            var jgnext = ig[i];
            jg[jgnext++] = i;
            foreach (var j in adjacencyList[i].OrderBy(j=>j))
            {
                if (j > i) jg[jgnext++] = j;
            }
        }

        return new PardisoMatrix(new PardisoMatrix.Portrait(ig, jg));
    }
}
