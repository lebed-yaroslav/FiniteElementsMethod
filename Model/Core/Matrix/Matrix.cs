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
}

/// <summary>
/// Global sparse matrix for FEM assembly
/// </summary>
public interface IGlobalMatrix : IMatrix, ICloneable
{
    static abstract IMatrixFactory Factory { get; }

    /// <summary>
    /// Checks if matrix portraits are same.
    /// Perform component-wise array check if portraits are not reference equals.
    /// </summary>
    bool HasSamePortrait(IGlobalMatrix other);

    /// <summary>
    /// Copies matrix contents into other matrix with same portrait <see cref="HasSamePortrait(IGlobalMatrix)"/>
    /// </summary>
    void CopyTo(IGlobalMatrix other);

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
