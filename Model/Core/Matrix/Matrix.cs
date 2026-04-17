using System.Diagnostics;
using Model.Core.Util;
using Model.Core.Vector;

namespace Model.Core.Matrix;

public interface IMatrix
{
    public int Size { get; }
}

public sealed class LocalMatrix(int n) : IMatrix
{
    public int Size { get; } = n;
    private double[,] _values = new double[n, n];
    public double this[int i, int j] { get => _values[i, j]; set => _values[i, j] = value; }

    public void operator *=(double alpha) =>
        _values.AsFlatSpan<double>().Scale(alpha);

    public void AddMatVec(
        ReadOnlySpan<double> vec,
        ReadOnlySpan<int> indices,
        Span<double> res,
        double scale = 1.0
    ) {
        Debug.Assert(indices.Length == Size);

        for (int i = 0; i < Size; ++i)
        {
            var gi = indices[i];
            if (indices[i] < 0) continue;
            for (int j = 0; j < Size; ++j)
            {
                var gj = indices[j];
                if (indices[j] < 0) continue;
                res[gi] += scale * _values[i, j] * vec[gj];
            }
        }
    }
}

/// <summary>
/// Global sparse matrix for FEM assembly
/// </summary>
public interface IGlobalMatrix : IMatrix, ICloneable
{
    /// <summary>
    /// Assembles local matrix into global using index mapping.
    /// Skips entries where indices[i] < 0 (e.g., constrained DOFs).
    /// </summary>
    /// <param name="matrix">Local element matrix</param>
    /// <param name="indices">Global DOF indices; negative = skip</param>
    void AddLocalMatrix(LocalMatrix matrix, ReadOnlySpan<int> indices);

    /// <summary>
    /// Matrix-vector multiplication: res = this * vec
    /// </summary>
    void MulVec(ReadOnlySpan<double> vec, Span<double> res);

    /// <summary>
    /// Fills all stored elements with given value.
    /// (Affect only "Portrait" elements)
    /// </summary>
    void Fill(double value);

    /// <summary>
    /// Scales matrix by given factor
    /// </summary>
    void Scale(double factor);
}

public interface IMatrixFactory
{
    IGlobalMatrix Create(IList<HashSet<int>> adjacencyList);
}
