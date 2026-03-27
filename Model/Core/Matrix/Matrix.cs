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


public interface IGlobalMatrix : IMatrix, ICloneable
{
    void AddLocalMatrix(LocalMatrix matrix, ReadOnlySpan<int> indices);
    void MulVec(ReadOnlySpan<double> vec, Span<double> res);
}
