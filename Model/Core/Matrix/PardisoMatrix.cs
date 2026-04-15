using System.Diagnostics;
using Quasar.Native;

namespace Model.Core.Matrix;

[Obsolete("This feature is incomplete and should not be used yet.")]
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

    public bool HasSamePortrait(IGlobalMatrix other)
    {
        if (other is not PardisoMatrix pardiso)
            return false;
        return ReferenceEquals(_portrait, pardiso._portrait) ||
            (_portrait.Ia.SequenceEqual(pardiso._portrait.Ia) &&
            _portrait.Ja.SequenceEqual(pardiso._portrait.Ja));
    }

    public void CopyTo(IGlobalMatrix other)
    {
        Debug.Assert(HasSamePortrait(other));
        var pardiso = (PardisoMatrix)other;
        _a.CopyTo(pardiso._a);
    }

    public object Clone() => new PardisoMatrix(this);

    // TODO: Implement IGlobalMatrix
    public void AddLocalMatrix(LocalMatrix matrix, ReadOnlySpan<int> indices) => throw new NotImplementedException();
    public void MulVec(ReadOnlySpan<double> vec, Span<double> res) => throw new NotImplementedException();
    public void Fill(double value) => Array.Fill(_a, value);
    public void Scale(double factor)
    {
        for (int i = 0; i < _a.Length; ++i)
            _a[i] *= factor;
    }
}


[Obsolete("This feature is incomplete and should not be used yet.")]
public sealed class PardisoMatrixFactory : IMatrixFactory
{
    public IGlobalMatrix Create(IList<HashSet<int>> adjacencyList)
    {
        var n = adjacencyList.Count;
        var ia = new int[n + 1];
        ia[0] = 0;
        for (int i = 0; i < n; i++)
        {
            ia[i + 1] = ia[i] + 1;
            foreach (var l in adjacencyList[i])
            {
                if (l > i) ia[i + 1]++;
            }
        }
        var ja = new int[ia[n]];
        for (int i = 0; i < n; i++)
        {
            var jgaddr = ia[i];
            ja[jgaddr++] = i;
            foreach (var j in adjacencyList[i].OrderBy(ll => ll))
            {
                if (j > i) ja[jgaddr++] = j;
            }
        }
        return new PardisoMatrix(new PardisoMatrix.Portrait(ia, ja));
    }
}
