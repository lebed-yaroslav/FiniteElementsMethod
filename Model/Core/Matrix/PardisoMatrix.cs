using Quasar.Native;
#pragma warning disable CS0618 // Type or member is obsolete
using static Model.Core.Matrix.PardisoMatrix;
#pragma warning restore CS0618 // Type or member is obsolete

namespace Model.Core.Matrix;

[Obsolete("This feature is incomplete and should not be used yet.")]
public sealed class PardisoMatrix(Portrait adjacencyList) : IPardisoMatrix<double>, IGlobalMatrix
{
    public sealed record Portrait(int[] Ia, int[] Ja)
    {
        public int Size => Ia.Length - 1;
        public int ElementCount => Ia[^1];

        public static Portrait Create(IList<HashSet<int>> adjacencyList)
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
            return new(ia, ja);
        }
    }

    private readonly Portrait _portrait = adjacencyList;
    private readonly double[] _a = new double[adjacencyList.ElementCount];

    public PardisoMatrixType MatrixType => PardisoMatrixType.SymmetricIndefinite;
    public int Size => _portrait.Size;
    public int n => _portrait.Size;

    public ReadOnlySpan<double> a => _a;
    public ReadOnlySpan<int> ia => _portrait.Ia;
    public ReadOnlySpan<int> ja => _portrait.Ja;

    // TODO: Implement IGlobalMatrix
    public void AddLocalMatrix(LocalMatrix matrix, ReadOnlySpan<int> indices) => throw new NotImplementedException();
    public void MulVec(ReadOnlySpan<double> vec, Span<double> res) => throw new NotImplementedException();
}
