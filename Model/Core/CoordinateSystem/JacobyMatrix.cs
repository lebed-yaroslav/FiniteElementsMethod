global using IJacobyMatrix1x1 = Model.Core.CoordinateSystem.IJacobyMatrix<Telma.Vector1D, Telma.Vector1D>;
global using IJacobyMatrix2x2 = Model.Core.CoordinateSystem.IJacobyMatrix<Telma.Vector2D, Telma.Vector2D>;
global using IJacobyMatrix3x3 = Model.Core.CoordinateSystem.IJacobyMatrix<Telma.Vector3D, Telma.Vector3D>;
global using IJacobyMatrix1x2 = Model.Core.CoordinateSystem.IJacobyMatrix<Telma.Vector1D, Telma.Vector2D>;
global using IJacobyMatrix2x3 = Model.Core.CoordinateSystem.IJacobyMatrix<Telma.Vector2D, Telma.Vector3D>;

global using ConstantJacobyMatrix1x1 = Model.Core.CoordinateSystem.ConstantJacobyMatrix<Telma.Vector1D, Telma.Vector1D>;
global using ConstantJacobyMatrix2x2 = Model.Core.CoordinateSystem.ConstantJacobyMatrix<Telma.Vector2D, Telma.Vector2D>;
global using ConstantJacobyMatrix3x3 = Model.Core.CoordinateSystem.ConstantJacobyMatrix<Telma.Vector3D, Telma.Vector3D>;
global using ConstantJacobyMatrix1x2 = Model.Core.CoordinateSystem.ConstantJacobyMatrix<Telma.Vector1D, Telma.Vector2D>;
global using ConstantJacobyMatrix2x3 = Model.Core.CoordinateSystem.ConstantJacobyMatrix<Telma.Vector2D, Telma.Vector3D>;

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

public class ConstantJacobyMatrix<TSource, TTarget>(double[,] j) : IJacobyMatrix<TSource, TTarget>
    where TSource : IVectorBase<TSource>
    where TTarget : IVectorBase<TTarget>
{
    public static bool IsConstant => true;
    protected readonly double[,] _j = j;

    public double this[int i, int j, TSource _] => _j[i, j];
    public double this[int i, int j] => _j[i, j];

    public virtual double Det(TSource sourcePoint) => this.Det();

    public static TSource operator *(ConstantJacobyMatrix<TSource, TTarget> lhs, TTarget rhs)
        => lhs.Mul(rhs);

    public static TSource operator *(TTarget lhs, ConstantJacobyMatrix<TSource, TTarget> rhs)
        => rhs.Mul(lhs);
}

// FIXME: Use specialization
public sealed class ConstantJacobyMatrix2D(double[,] j) : ConstantJacobyMatrix2x2(j)
{
    public ConstantJacobyMatrix2D Inverse()
    {
        var detJ = this.Det(Vector2D.Zero);
        Debug.Assert(Math.Abs(detJ) >= 1e-14, "The Jacobian matrix is ​​singular at given point");
        return new(new[,] {
            {_j[1, 1] / detJ, -_j[0, 1] / detJ},
            {-_j[1, 0] / detJ, _j[0, 0] / detJ}
        });
    }
}
