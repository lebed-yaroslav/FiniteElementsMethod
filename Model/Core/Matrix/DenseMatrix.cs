using System.Diagnostics;
using Model.Core.Util;

namespace Model.Core.Matrix;


public sealed class DenseMatrix : IGlobalMatrix
{
    public static IMatrixFactory Factory { get; } = new DenseMatrixFactory();

    private readonly double[,] _data;
    public int Size => _data.GetLength(0);

    public DenseMatrix(int n)
    {
        _data = new double[n, n];
    }

    public DenseMatrix(double[,] data)
    {
        Debug.Assert(data.GetLength(0) == data.GetLength(1));
        _data = data;
    }

    public bool HasSamePortrait(IGlobalMatrix other) => other is DenseMatrix;

    public object Clone() => new DenseMatrix((double[,])_data.Clone());

    public void AddLocalMatrix(LocalMatrix matrix, ReadOnlySpan<int> indices)
    {
        int m = indices.Length;
        ArgumentOutOfRangeException.ThrowIfNotEqual(matrix.Size, m);

        for (int i = 0; i < m; ++i)
        {
            int gi = indices[i];
            if (gi < 0) continue;
            for (int j = 0; j < m; ++j)
            {
                int gj = indices[j];
                if (gj < 0) continue;
                _data[gi, gj] += matrix[i, j];
            }
        }
    }

    public void MulVec(ReadOnlySpan<double> vector, Span<double> result)
    {
        Debug.Assert(vector.Length == Size);
        Debug.Assert(result.Length == Size);
        for (int i = 0; i < Size; i++)
        {
            double sum = 0;
            for (int j = 0; j < Size; j++)
                sum += _data[i, j] * vector[j];

            result[i] = sum;
        }
    }

    public void Fill(double value) => _data.AsFlatSpan<double>().Fill(value);

    public void Scale(double factor)
    {
        var flatten = _data.AsFlatSpan<double>();
        for (int i = 0; i < flatten.Length; ++i)
            flatten[i] *= factor;
    }
}

public sealed class DenseMatrixFactory : IMatrixFactory
{
    public IGlobalMatrix Create(IList<HashSet<int>> adjacencyList) => new DenseMatrix(adjacencyList.Count);
}
