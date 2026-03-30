global using IJacobyMatrix1X1 = Model.Core.CoordinateSystem.IJacobyMatrix<Telma.Vector1D, Telma.Vector1D>;
global using IJacobyMatrix2X2 = Model.Core.CoordinateSystem.IJacobyMatrix<Telma.Vector2D, Telma.Vector2D>;
global using IJacobyMatrix3X3 = Model.Core.CoordinateSystem.IJacobyMatrix<Telma.Vector3D, Telma.Vector3D>;
global using IJacobyMatrix1X2 = Model.Core.CoordinateSystem.IJacobyMatrix<Telma.Vector1D, Telma.Vector2D>;
global using IJacobyMatrix2X3 = Model.Core.CoordinateSystem.IJacobyMatrix<Telma.Vector2D, Telma.Vector3D>;

global using ConstantJacobyMatrix1X1 = Model.Core.CoordinateSystem.ConstantJacobyMatrix<
    Telma.Vector1D, Telma.Vector1D, Model.Core.CoordinateSystem.MatrixOperations.Ops1X1
>;
global using ConstantJacobyMatrix2X2 = Model.Core.CoordinateSystem.ConstantJacobyMatrix<
    Telma.Vector2D, Telma.Vector2D, Model.Core.CoordinateSystem.MatrixOperations.Ops2X2
>;
global using ConstantJacobyMatrix3X3 = Model.Core.CoordinateSystem.ConstantJacobyMatrix<
    Telma.Vector3D, Telma.Vector3D, Model.Core.CoordinateSystem.MatrixOperations.Ops3X3
>;

global using ConstantJacobyMatrix1X2 = Model.Core.CoordinateSystem.ConstantJacobyMatrix<
    Telma.Vector1D, Telma.Vector2D, Model.Core.CoordinateSystem.MatrixOperations.Ops1X2
>;
global using ConstantJacobyMatrix2X3 = Model.Core.CoordinateSystem.ConstantJacobyMatrix<
    Telma.Vector2D, Telma.Vector3D, Model.Core.CoordinateSystem.MatrixOperations.Ops2X3
>;

using System.Diagnostics;
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

public sealed class ConstantJacobyMatrix<TSource, TTarget, TOps>(double[,] j) : IJacobyMatrix<TSource, TTarget>
    where TSource : IVectorBase<TSource>
    where TTarget : IVectorBase<TTarget>
    where TOps : IMatrixOperations<TSource, TTarget, TOps>
{
    public static bool IsConstant => true;
    protected readonly double[,] _j = j;

    public double this[int i, int j, TSource _] => _j[i, j];
    public double this[int i, int j] => _j[i, j];

    public double Det(TSource sourcePoint) => TOps.Det(this);
    public double Det() => TOps.Det(this);

    public ConstantJacobyMatrix<TSource, TTarget, TOps> Inverse()
        => TOps.Inverse(this);

    public static TSource operator *(ConstantJacobyMatrix<TSource, TTarget, TOps> lhs, TTarget rhs)
        => TOps.Mul(lhs, rhs);

    public static TTarget operator *(TSource lhs, ConstantJacobyMatrix<TSource, TTarget, TOps> rhs)
        => TOps.Mul(lhs, rhs);
}
