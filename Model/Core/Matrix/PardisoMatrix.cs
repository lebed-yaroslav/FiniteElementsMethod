using Quasar.Native;

namespace Model.Core.Matrix;

[Obsolete("This feature is incomplete and should not be used yet.")]
public sealed class PardisoMatrix(PardisoMatrix.Portrait portrait) : IPardisoMatrix<double>, IGlobalMatrix
{
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
    public void AddLocalMatrix(LocalMatrix matrix, ReadOnlySpan<int> indices) => throw new NotImplementedException();
    public void MulVec(ReadOnlySpan<double> vec, Span<double> res) => throw new NotImplementedException();
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
