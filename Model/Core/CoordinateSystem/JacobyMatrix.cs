using System.Diagnostics;
using Telma;
using Telma.Extensions;

namespace Model.Core.CoordinateSystem;

public interface IJacobyMatrix<TSource, TTarget>
    where TSource : IVectorBase<TSource>
    where TTarget : IVectorBase<TTarget>
{
    static int Rows => TTarget.Dimensions;
    static int Columns => TSource.Dimensions;
    static abstract bool IsConstant { get; }

    double Det(TSource sourcePoint);
    double this[int i, int j, TSource sourcePoint] { get; }
    double this[int i, int j] { get; }
}

public static class ICoordinateTransformExtensions
{
    extension<TCoordinateTransform, TSource, TTarget>(TCoordinateTransform self)
        where TCoordinateTransform : ICoordinateTransform<TSource, TTarget>
        where TSource : IVectorBase<TSource>
        where TTarget : IVectorBase<TTarget>
    {
        public double Jacobian()
        {
            Debug.Assert(TCoordinateTransform.IsLinear);
            return self.Jacobian(TTarget.Zero);
        }
    }
}

public static class IJacobyMatrixExtensions
{
    extension<TJacobyMatrix, TSource, TTarget>(TJacobyMatrix self)
        where TJacobyMatrix : IJacobyMatrix<TSource, TTarget>
        where TSource : IVectorBase<TSource>
        where TTarget : IVectorBase<TTarget>
    {
        public double Det()
        {
            Debug.Assert(TJacobyMatrix.IsConstant);
            return self.Det(TSource.Zero);
        }
    }
}

public sealed class ConstantJacobyMatrix2D(double[,] j) : IJacobyMatrix<Vector2D, Vector2D>
{
    public static bool IsConstant => true;
    private readonly double[,] _j = j;

    public double this[int i, int j, Vector2D _] => _j[i, j];
    public double this[int i, int j] => _j[i, j];

    public double Det(Vector2D _) => _j[0, 0] * _j[1, 1] - _j[0, 1] * _j[1, 0];

    public ConstantJacobyMatrix2D Inverse()
    {
        var detJ = this.Det(Vector2D.Zero);
        return new(new[,] {
            {_j[1, 1] / detJ, -_j[0, 1] / detJ},
            {-_j[1, 0] / detJ, _j[0, 0] / detJ}
        });
    }

    // TODO: Generalize IJacobyMatrix operations
    public static Vector2D operator *(ConstantJacobyMatrix2D a, Vector2D x)
        => new(
            a[0, 0] * x.X + a[0, 1] * x.Y,
            a[1, 0] * x.X + a[1, 1] * x.Y
        );
}
